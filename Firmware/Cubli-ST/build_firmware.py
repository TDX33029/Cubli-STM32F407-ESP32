# -*- coding: utf-8 -*-
import os
import sys
import time
import subprocess
import ctypes
from pathlib import Path
import argparse
import json
import re

def rerun_from_ascii_drive_if_needed():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir.isascii() or os.name != "nt":
        return None

    used_drives = ctypes.windll.kernel32.GetLogicalDrives()
    for letter in "ZYXWVUTSRQPONMLKJIHGFED":
        if used_drives & (1 << (ord(letter) - ord("A"))):
            continue
        drive = letter + ":"
        mapped_script = drive + "\\build_firmware.py"
        subprocess.run(["subst", drive, script_dir], check=True)
        try:
            return subprocess.run([sys.executable, mapped_script] + sys.argv[1:]).returncode
        finally:
            subprocess.run(["subst", drive, "/D"], check=False)

    print("Error: no free drive letter is available for the ASCII build path.")
    return 1

def build(encoder=None, ppr=None, ppr_axes=(None, None, None), output_variant=None):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    mdk_dir = "MDK-ARM"
    variant = output_variant or ("CubeMX-quad" if encoder == "quad" else "CubeMX")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,48}", variant):
        raise ValueError("Output variant must contain only ASCII letters, digits, '_' or '-'")
    obj_dir = "MDK-ARM/" + variant

    tool_candidates = [
        os.environ.get("KEIL_ARMCC_BIN"),
        r"D:\Program Files\Keil5\ARM\ARMCC\Bin",
        r"C:\Keil_v5\ARM\ARMCC\Bin",
    ]
    tool_dir = next((p for p in tool_candidates if p and os.path.isfile(os.path.join(p, "armcc.exe"))), None)
    if tool_dir is None:
        print("Error: ARMCC not found. Set KEIL_ARMCC_BIN to the ARMCC Bin directory.")
        return False
    armcc = os.path.join(tool_dir, "armcc.exe")
    armasm = os.path.join(tool_dir, "armasm.exe")
    armlink = os.path.join(tool_dir, "armlink.exe")
    fromelf = os.path.join(tool_dir, "fromelf.exe")

    if not os.path.exists(armcc):
        print(f"Error: ARMCC not found at {armcc}")
        return False

    os.makedirs(obj_dir, exist_ok=True)

    inc_paths = [
        "Core/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
        "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
        "Drivers/CMSIS/Include",
        "Hardware",
        "SimpleFOC"
    ]

    inc_args = []
    for p in inc_paths:
        inc_args.extend(["-I", p])

    defines = ["-D__UVISION_VERSION=542", "-D_RTE_", "-DSTM32F407xx", "-DUSE_HAL_DRIVER"]
    if encoder is not None:
        defines.append("-DENC_USE_HW_QUAD=" + ("1" if encoder == "quad" else "0"))
    if ppr is not None:
        defines.append(f"-DENC_QUAD_PPR={ppr}")
    for i, value in enumerate(ppr_axes, 1):
        if value is not None:
            defines.append(f"-DENC_QUAD_PPR_M{i}={value}")
    cflags = ["--cpu=Cortex-M4.fp.sp", "-g", "-O2", "--apcs=interwork", "--split_sections", "--c99"] + defines + inc_args

    srcs = [
        ("c", "Core/Src/main.c", "MDK-ARM/CubeMX/main.o"),
        ("c", "Core/Src/gpio.c", "MDK-ARM/CubeMX/gpio.o"),
        ("c", "Core/Src/i2c.c", "MDK-ARM/CubeMX/i2c.o"),
        ("c", "Core/Src/tim.c", "MDK-ARM/CubeMX/tim.o"),
        ("c", "Core/Src/usart.c", "MDK-ARM/CubeMX/usart.o"),
        ("c", "Core/Src/stm32f4xx_it.c", "MDK-ARM/CubeMX/stm32f4xx_it.o"),
        ("c", "Core/Src/stm32f4xx_hal_msp.c", "MDK-ARM/CubeMX/stm32f4xx_hal_msp.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c", "MDK-ARM/CubeMX/stm32f4xx_hal_i2c.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_i2c_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c", "MDK-ARM/CubeMX/stm32f4xx_hal_rcc.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_rcc_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c", "MDK-ARM/CubeMX/stm32f4xx_hal_flash.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_flash_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c", "MDK-ARM/CubeMX/stm32f4xx_hal_flash_ramfunc.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c", "MDK-ARM/CubeMX/stm32f4xx_hal_gpio.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_dma_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c", "MDK-ARM/CubeMX/stm32f4xx_hal_dma.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c", "MDK-ARM/CubeMX/stm32f4xx_hal_pwr.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_pwr_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_cortex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c", "MDK-ARM/CubeMX/stm32f4xx_hal.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c", "MDK-ARM/CubeMX/stm32f4xx_hal_exti.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c", "MDK-ARM/CubeMX/stm32f4xx_hal_tim.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_tim_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c", "MDK-ARM/CubeMX/stm32f4xx_hal_uart.o"),
        ("c", "Core/Src/system_stm32f4xx.c", "MDK-ARM/CubeMX/system_stm32f4xx.o"),
        ("c", "SimpleFOC/foc_utils.c", "MDK-ARM/CubeMX/foc_utils.o"),
        ("c", "SimpleFOC/FOCMotor.c", "MDK-ARM/CubeMX/focmotor.o"),
        ("c", "SimpleFOC/BLDCMotor.c", "MDK-ARM/CubeMX/bldcmotor.o"),
        ("c", "SimpleFOC/CurrentSense.c", "MDK-ARM/CubeMX/currentsense.o"),
        ("c", "Hardware/timer.c", "MDK-ARM/CubeMX/timer.o"),
        ("c", "Hardware/delay.c", "MDK-ARM/CubeMX/delay.o"),
        ("c", "Hardware/spi.c", "MDK-ARM/CubeMX/spi.o"),
        ("c", "Hardware/lsm6dsr.c", "MDK-ARM/CubeMX/lsm6dsr.o"),
        ("c", "Hardware/mpu6050.c", "MDK-ARM/CubeMX/mpu6050.o"),
        ("c", "Hardware/lqr_balance.c", "MDK-ARM/CubeMX/lqr_balance.o"),
        ("c", "Hardware/enc_quad.c", "MDK-ARM/CubeMX/enc_quad.o"),
        ("s", "MDK-ARM/startup_stm32f407xx.s", "MDK-ARM/CubeMX/startup_stm32f407xx.o"),
    ]

    print(f"Building Cubli-F407ZGT6 firmware ({len(srcs)} files)...")
    # Every object referenced by the linker must be rebuilt; never reuse old HAL objects.
    link_text = Path("MDK-ARM/CubeMX/CubeMX.lnp").read_text(encoding="utf-8-sig")
    known = {Path(obj).name.lower() for _, _, obj in srcs}
    for obj in re.findall(r'"([^"\n]+\.o)"', link_text):
        basename = obj.replace("\\", "/").split("/")[-1].lower()
        if basename not in known:
            source = Path("Drivers/STM32F4xx_HAL_Driver/Src") / basename.replace(".o", ".c")
            if not source.exists():
                raise FileNotFoundError(f"No source for linker object: {obj}")
            srcs.append(("c", str(source), "MDK-ARM/CubeMX/" + basename))
    srcs = [(kind, src, obj_dir + "/" + Path(obj).name) for kind, src, obj in srcs]
    print(f"Compiling all {len(srcs)} linker inputs; encoder={encoder or 'header default'}; output={obj_dir}")
    errors = 0
    warnings = 0
    for stype, src, obj in srcs:
        src_path = os.path.normpath(src)
        obj_path = os.path.normpath(obj)
        if os.path.exists(obj_path):
            try: os.remove(obj_path)
            except Exception: pass
        if stype == "c":
            cmd = [armcc, "-c"] + cflags + [src_path, "-o", obj_path]
        else:
            cmd = [armasm, "--cpu=Cortex-M4.fp.sp", "-g", "--apcs=interwork", "--pd", "__MICROLIB SETA 1", src_path, "-o", obj_path]
        
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if proc.returncode != 0:
            time.sleep(0.05)
            if os.path.exists(obj_path):
                try: os.remove(obj_path)
                except Exception: pass
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        diagnostics = (proc.stdout + proc.stderr).decode("gbk", errors="replace").strip()
        if diagnostics:
            print(f"{src}: {diagnostics}")
            warnings += diagnostics.lower().count("warning")
        if proc.returncode != 0:
            print(f"FAILED: {os.path.basename(src)}")
            print(proc.stderr.decode("gbk", errors="ignore") or proc.stdout.decode("gbk", errors="ignore"))
            errors += 1

    if errors > 0:
        print(f"Build failed with {errors} compilation error(s).")
        return False

    print(f"Compilation complete: {errors} errors, {warnings} warning messages. Linking...")
    orig_cwd = os.getcwd()
    os.chdir(mdk_dir)
    try:
        link_text = re.sub(r'"cubemx\\([^"\n]+\.o)"', lambda m: '"' + variant + '\\' + m[1] + '"', link_text, flags=re.I)
        link_text = link_text.replace('CubeMX\\CubeMX.axf', variant + '\\CubeMX.axf')
        # Keep a separate link response file so a QUAD build cannot corrupt the default build.
        response_file = Path(variant) / 'build_firmware.lnp'
        response_file.write_text(link_text, encoding='utf-8')
        link_cmd = [armlink, '--via=' + str(response_file)]
        proc = subprocess.run(link_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out = proc.stdout.decode("gbk", errors="ignore") + proc.stderr.decode("gbk", errors="ignore")
        if proc.returncode != 0:
            print("Link failed:")
            print(out)
            return False
        
        print("Link Output:")
        print(out.strip())

        hex_cmd = [fromelf, "--i32", f"--output={variant}/CubeMX.hex", f"{variant}/CubeMX.axf"]
        subprocess.run(hex_cmd, check=True)
        Path(variant, 'build_config.json').write_text(json.dumps({'encoder_override': encoder, 'ppr_override': ppr, 'axis_ppr_overrides': list(ppr_axes), 'defines': defines, 'input_count': len(srcs), 'warnings': warnings}, indent=2) + '\n', encoding='utf-8')
        print(f"Successfully generated {variant}/CubeMX.hex!")
        return True
    finally:
        os.chdir(orig_cwd)

if __name__ == "__main__":
    mapped_result = rerun_from_ascii_drive_if_needed()
    if mapped_result is not None:
        sys.exit(mapped_result)

    parser = argparse.ArgumentParser(description='Build STM32 firmware; QUAD artifacts use a separate directory.')
    parser.add_argument('--encoder', choices=('i2c', 'quad'), help='Override ENC_USE_HW_QUAD for every source file')
    parser.add_argument('--variant', help='Use a separate ASCII-named output directory instead of CubeMX')
    parser.add_argument('--ppr', type=int, help='Common AB pulses/revolution BEFORE x4 (1..1024)')
    for i in range(1, 4):
        parser.add_argument(f'--ppr-m{i}', type=int, help=f'Override M{i} PPR')
    args = parser.parse_args()
    axes = (args.ppr_m1, args.ppr_m2, args.ppr_m3)
    if any(v is not None and not 1 <= v <= 1024 for v in (args.ppr,) + axes):
        parser.error('PPR must be 1..1024; the timer counts four edges per AB pulse')
    success = build(args.encoder, args.ppr, axes, args.variant)
    sys.exit(0 if success else 1)
