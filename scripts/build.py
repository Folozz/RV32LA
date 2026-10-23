#!/usr/bin/env python3
# 脚本名称：build.py
# 功能与设计思路：
#   本脚本实现 RISC-V 片上固件跨平台自动化构建、HEX 镜像生成与片上存储模板回写。
#   1. 工具链自动探测：支持环境变量 RISCV_PREFIX、系统 PATH 及本地 .tools 工具链。
#   2. 目标灵活装配：支持 demo 综合控制台、isa 指令自测、interrupt 中断自测及 CoreMark 模式。
#   3. 自动同步回写：编译生成 64KB 对齐镜像后，自动更新 rtl/soc_ram.v 内联初始化数据。
#   4. 工作区清理：构建完成后自动清理 .elf, .bin, .o 等中间临时产物，仅保留目标 HEX 文件。

import argparse, hashlib, json, os, pathlib, re, shutil, subprocess, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

def compiler_prefix():
    """
    自动探测 RISC-V GCC 交叉编译器前缀
    优先级：1. 环境变量 RISCV_PREFIX -> 2. 系统 PATH -> 3. 本地 .tools/ 内置工具链
    """
    env = os.environ.get('RISCV_PREFIX')
    if env: return env
    for p in ['riscv-none-elf-', 'riscv32-unknown-elf-', 'riscv64-unknown-elf-']:
        if shutil.which(p + 'gcc'): return p
    found = list((ROOT / '.tools').glob('*/bin/riscv-none-elf-gcc.exe'))
    if found:
        return str(found[0])[:-len('gcc.exe')]
    found_u = list((ROOT / '.tools').glob('*/bin/riscv32-unknown-elf-gcc.exe'))
    if found_u:
        return str(found_u[0])[:-len('gcc.exe')]
    raise SystemExit('No compiler found. Please ensure RISC-V GCC is in PATH or under .tools/')

def run(cmd):
    """打印并同步执行命令行子进程"""
    print(' '.join(map(str, cmd)), flush=True)
    subprocess.run(list(map(str, cmd)), check=True)

def valid_output_name(name):
    """校验目标输出名称合法性，杜绝路径穿越与 Windows 保留设备名"""
    reserved = {'con', 'prn', 'aux', 'nul'}
    return bool(re.fullmatch(r'[A-Za-z0-9_-]{1,64}', name)) and name.lower() not in reserved and not re.fullmatch(r'(com|lpt)[1-9]', name.lower())

def update_soc_ram(padded):
    """
    自动同步片上 RAM：
    将编译生成的 64KB 机器码逐字 (16,384 字) 转换为 Verilog 显式赋值语句，
    替换 rtl/soc_ram.v 中的 initial 块，确保 FPGA 综合器上电即可直接固化运行。
    """
    soc_ram_path = ROOT / 'rtl' / 'soc_ram.v'
    if not soc_ram_path.exists():
        print(f"Warning: {soc_ram_path} not found, skipping RAM update.")
        return
    text = soc_ram_path.read_text(encoding='utf-8')
    m = re.search(r'initial\s+begin\s*.*?\s*end', text, re.DOTALL)
    if not m:
        print("Warning: initial begin ... end not found in rtl/soc_ram.v")
        return
    lines = ['initial begin\n']
    lines.append('    // 完整固件：demo；由 build.py 自动编译并同步内联生成\n')
    lines.append('    // 64 KiB RAM 逐字显式初始化；每个地址均有明确定义，供 FPGA 综合上电即行\n')
    for i in range(16384):
        w = int.from_bytes(padded[i * 4:(i + 1) * 4], 'little')
        lines.append(f'    ram[{i:5d}] = 32\'h{w:08x};\n')
    lines.append('  end')
    new_init = ''.join(lines)
    new_text = text[:m.start()] + new_init + text[m.end():]
    soc_ram_path.write_text(new_text, encoding='utf-8')
    print('SUCCESS: Updated rtl/soc_ram.v with 16384 words of compiled machine code!')

def main():
    os.chdir(ROOT)
    ap = argparse.ArgumentParser(description="Pango RV32 Firmware Build Script")
    ap.add_argument('target', nargs='?', default='demo', choices=['demo', 'demo_sim', 'isa', 'coremark', 'coremark_smoke', 'coremark_validation', 'interrupt'])
    ap.add_argument('--iterations', type=int, default=None)
    ap.add_argument('--output')
    ap.add_argument('--no-ram-update', action='store_true', help='Do not update rtl/soc_ram.v')
    a = ap.parse_args()

    if a.iterations is not None and not 0 <= a.iterations <= 2147483647:
        ap.error('--iterations must be 0..2147483647')
    if a.iterations is not None and not (a.target.startswith('coremark') or a.target in ['demo', 'demo_sim']):
        ap.error('--iterations is only for CoreMark or demo')

    out = ROOT / 'build'
    out.mkdir(exist_ok=True)
    prefix = compiler_prefix()
    target = a.target
    name = a.output or target

    if not valid_output_name(name):
        raise SystemExit('Unsafe output name: use 1..64 ASCII letters/digits/_/- and avoid Windows device names')

    # 针对 RV32I 基础整数架构优化的编译选项
    flags = [
        '-march=rv32i_zicsr', '-mabi=ilp32', '-O2', '-g',
        '-ffreestanding', '-fno-builtin', '-fno-common',
        '-ffunction-sections', '-fdata-sections',
        '-msmall-data-limit=0', '-mno-relax', '-Wall', '-Wextra'
    ]
    src = ['firmware/start.S', 'firmware/platform.c']
    defs = []
    includes = ['-Ifirmware']

    # 根据编译目标选择源代码文件与宏定义
    if target in ['demo', 'demo_sim']:
        src += ['firmware/demo.c', 'firmware/command.c', 'tests/isa.S', 'tests/interrupt_test.S']
        if target == 'demo_sim':
            defs += ['-DSIM_TEST=1', '-DDISABLE_COREMARK=1']
        else:
            cm = 'third_party/coremark/'
            if (ROOT / cm / 'core_main.c').exists():
                iters = a.iterations if a.iterations is not None else 200
                cm_defs = [f'-DITERATIONS={iters}', '-DTOTAL_DATA_SIZE=2000', '-DPERFORMANCE_RUN=1']
                cm_includes = ['-Ifirmware', '-Ifirmware/coremark_port', '-I' + cm]
                core_main_obj = 'build/core_main_renamed.o'
                cmd_cm = [prefix + 'gcc', *flags, *cm_defs, *cm_includes, '-Dmain=coremark_main_renamed', '-c', cm + 'core_main.c', '-o', core_main_obj]
                run(cmd_cm)
                cm_src = [cm + x for x in ['core_list_join.c', 'core_matrix.c', 'core_state.c', 'core_util.c']]
                cm_src += ['firmware/coremark_port/core_portme.c', 'firmware/coremark_port/ee_printf.c']
                src += cm_src + [core_main_obj]
                includes += ['-Ifirmware/coremark_port', '-I' + cm]
                defs += cm_defs
            else:
                defs += ['-DDISABLE_COREMARK=1']
    elif target == 'isa':
        src = ['tests/isa.S']
    elif target == 'interrupt':
        src = ['tests/interrupt_test.S']
    else:
        cm = 'third_party/coremark/'
        if not (ROOT / cm / 'core_main.c').exists():
            raise SystemExit('Missing third_party/coremark; see THIRD_PARTY.md')
        src += [cm + x for x in ['core_main.c', 'core_list_join.c', 'core_matrix.c', 'core_state.c', 'core_util.c']]
        src += ['firmware/coremark_port/core_portme.c', 'firmware/coremark_port/ee_printf.c']
        includes += ['-Ifirmware/coremark_port', '-I' + cm]
        iters = a.iterations if a.iterations is not None else (1 if target == 'coremark_smoke' else 0)
        defs += ['-DITERATIONS=' + str(iters), '-DTOTAL_DATA_SIZE=2000', '-DVALIDATION_RUN=1' if target == 'coremark_validation' else '-DPERFORMANCE_RUN=1']

    elf = 'build/' + name + '.elf'
    libgcc = subprocess.check_output([prefix + 'gcc', '-march=rv32i_zicsr', '-mabi=ilp32', '-print-libgcc-file-name'], encoding='utf-8').strip()
    if (ROOT / 'build/libgcc.a').exists():
        libgcc = 'build/libgcc.a'
    else:
        try:
            libgcc = os.path.relpath(libgcc, ROOT)
        except ValueError:
            pass

    # 调用 GCC 链接生成 ELF 镜像
    cmd = [prefix + 'gcc', *flags, *defs, *includes, *src, '-nostdlib', '-Wl,--gc-sections', '-Wl,--no-relax', '-Tfirmware/link.ld', libgcc, '-o', elf]
    run(cmd)
    
    # 提取裸机二进制文件并填充对齐至 64KB (65536 字节)
    bin_file = pathlib.Path('build/' + name + '.bin')
    run([prefix + 'objcopy', '-O', 'binary', elf, str(bin_file)])

    data = bin_file.read_bytes()
    if len(data) > 65536:
        raise SystemExit('Image too large: exceeds 64 KiB')
    padded = data + b'\0' * (65536 - len(data))
    
    # 生成供仿真与加载使用的标准 32 位十六进制 .hex 文件
    hex_path = pathlib.Path('build/' + name + '.hex')
    hex_path.write_text(''.join(f'{int.from_bytes(padded[i:i+4],"little"):08x}\n' for i in range(0, len(padded), 4)), encoding='ascii')

    # 清理所有中间编译临时产物，仅保留 .hex 文件
    for f in [pathlib.Path(elf), bin_file, ROOT / 'build' / 'core_main_renamed.o']:
        if f.exists():
            f.unlink()
    for map_f in (ROOT / 'build').glob('*.map'):
        map_f.unlink()

    print(f"\n[OK] 固件编译成功！已生成: {hex_path}")

    # 若为默认 demo 目标，自动同步机器码至 FPGA BRAM 硬件模型 rtl/soc_ram.v
    if not a.no_ram_update and target == 'demo':
        update_soc_ram(padded)

if __name__ == '__main__':
    main()
