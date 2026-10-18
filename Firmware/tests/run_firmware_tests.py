"""Compile and execute production control C with mocked peripherals using MSVC on Windows."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import contextlib
import shutil
ROOT = Path(__file__).resolve().parents[1]
ST = ROOT / 'Cubli-ST'
TESTS = ROOT / 'tests'
vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
vs = subprocess.check_output([str(vswhere), '-latest', '-property', 'installationPath'], text=True).strip()
# MSVC/antivirus may briefly hold generated files; cleanup must not recurse on WinError 32.
with contextlib.nullcontext(tempfile.mkdtemp(prefix='cubli-control-')) as tmp:
    tmp = Path(tmp)
    (tmp / 'main.h').write_text('#include "mock_hw.h"\n')
    main = (ST / 'Core/Src/main.c').read_text(encoding='utf-8-sig')
    # Exclude vendor startup/HAL init and main scheduling; preserve globals, parser and safety functions verbatim.
    variables = main.split('/* USER CODE BEGIN PV */')[1].split('/* USER CODE END PV */')[0]
    prototypes = main.split('/* USER CODE BEGIN PFP */')[1].split('/* USER CODE END PFP */')[0]
    control = main.split('/* USER CODE BEGIN 4 */')[1].split('/* USER CODE END 4 */')[0]
    (tmp / 'control_under_test.c').write_text(variables + prototypes + control, encoding='utf-8')
    for name, source in [('utils','SimpleFOC/foc_utils.c'), ('foc','SimpleFOC/FOCMotor.c'), ('motor','SimpleFOC/BLDCMotor.c'), ('balance','Hardware/lqr_balance.c')]:
        code = (ST / source).read_text(encoding='utf-8-sig')
        # Replace hardware umbrella/includes only. All function bodies are unchanged.
        code = re.sub(r'^#include "(?:MyProject|timer|delay)\.h".*$', '#include "mock_hw.h"', code, flags=re.M)
        (tmp / f'{name}_under_test.c').write_text(code, encoding='utf-8')
    uart = (ST / 'Core/Src/usart.c').read_text(encoding='utf-8-sig')
    uart = uart.split('/* USER CODE BEGIN 0 */')[1].split('/* USER CODE END 0 */')[0] + uart.split('/* USER CODE BEGIN 1 */')[1].split('/* USER CODE END 1 */')[0]
    uart = uart.replace('#include "stm32f4xx.h"', '#include "mock_hw.h"').replace('fputc(', 'cubli_fputc(')
    (tmp / 'uart_under_test.c').write_text(uart, encoding='utf-8')
    for f in ['mock_hw.h', 'firmware_harness.c', 'uart_harness.c']:
        (tmp / f).write_bytes((TESTS / f).read_bytes())
    compile_line = ['cl', '/nologo', '/std:c11', '/utf-8', '/W3', '/D_CRT_SECURE_NO_WARNINGS', '/I'+str(tmp), '/I'+str(ST/'Hardware'), '/I'+str(ST/'SimpleFOC'), 'firmware_harness.c', '/Fe:harness.exe']
    batch = tmp / 'build.cmd'
    # Pin the batch codepage before reading the UTF-8 command; vcvars may change it.
    batch.write_text('@echo off\nchcp 65001 >nul\ncall "' + vs + '\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul\nchcp 65001 >nul\n' + subprocess.list2cmdline(compile_line) + '\n', encoding='utf-8')
    subprocess.run(['cmd.exe', '/c', str(batch)], cwd=tmp, check=True)
    result = subprocess.run([str(tmp/'harness.exe')], cwd=tmp, capture_output=True, text=True)
    if result.returncode: print(result.stdout[-8000:], result.stderr)
    result.check_returncode()
    for line in result.stdout.splitlines():
        if line.startswith('Observer fixed-pivot simulation RMS:'):
            print(line, flush=True)
    print(result.stdout.strip().splitlines()[-1], flush=True)
    compile_line[compile_line.index('firmware_harness.c')] = 'uart_harness.c'
    batch.write_text('@echo off\nchcp 65001 >nul\ncall "' + vs + '\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul\nchcp 65001 >nul\n' + subprocess.list2cmdline(compile_line) + '\n', encoding='utf-8')
    subprocess.run(['cmd.exe', '/c', str(batch)], cwd=tmp, check=True)
    subprocess.run([str(tmp/'harness.exe')], cwd=tmp, check=True)

shutil.rmtree(tmp, ignore_errors=True)
