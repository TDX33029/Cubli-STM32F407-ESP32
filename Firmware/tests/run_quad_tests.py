"""Run real encoder/timebase/FOC sources against mocked STM32 registers (MSVC)."""
from pathlib import Path
import contextlib
import os
import re
import shutil
import subprocess
import tempfile
import sys
sys.stdout.reconfigure(errors="backslashreplace")
sys.stderr.reconfigure(errors="backslashreplace")
ROOT=Path(__file__).resolve().parents[1]
ST=ROOT/'Cubli-ST'
TESTS=ROOT/'tests'
vswhere=Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
vs=subprocess.check_output([str(vswhere),'-latest','-property','installationPath'],text=True).strip()
with contextlib.nullcontext(Path(tempfile.mkdtemp(prefix='cubli-quad-'))) as tmp:
    (tmp/'main.h').write_text('#include "mock_quad_hw.h"\n')
    for src in ('mock_quad_hw.h','quad_harness.c'):
        (tmp/src).write_bytes((TESTS/src).read_bytes())
    for target,source in [('quad','Hardware/enc_quad.c'),('clock','Hardware/timer.c'),('quad_utils','SimpleFOC/foc_utils.c'),('quad_foc','SimpleFOC/FOCMotor.c')]:
        code=(ST/source).read_text(encoding='utf-8-sig')
        code=re.sub(r'^#include "(?:MyProject|i2c)\.h".*$', '#include "mock_quad_hw.h"',code,flags=re.M)
        (tmp/f'{target}_under_test.c').write_text(code,encoding='utf-8')
    def compile_case(defines,expect_success=True):
        cmd=['cl','/nologo','/std:c11','/utf-8','/W3','/D_CRT_SECURE_NO_WARNINGS']+['/D'+d for d in defines]+['/I'+str(tmp),'/I'+str(ST/'Hardware'),'/I'+str(ST/'SimpleFOC'),'quad_harness.c','/Fe:quad.exe']
        batch=tmp/'build.cmd'
        # Pin the batch codepage before reading the UTF-8 command; vcvars may change it.
        batch.write_text('@echo off\nchcp 65001 >nul\ncall "'+vs+'\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul\nchcp 65001 >nul\n'+subprocess.list2cmdline(cmd)+'\n',encoding='utf-8')
        result=subprocess.run(['cmd.exe','/c',str(batch)],cwd=tmp,capture_output=True,text=True,encoding="utf-8",errors='replace')
        if expect_success:
            if result.returncode: print(result.stdout,result.stderr)
            result.check_returncode()
            if 'warning' in result.stdout.lower(): print(result.stdout)
            subprocess.run([str(tmp/'quad.exe')],cwd=tmp,check=True)
        elif not result.returncode:
            raise AssertionError('Invalid PPR unexpectedly compiled')
    compile_case(['ENC_USE_HW_QUAD=1'])
    compile_case(['ENC_USE_HW_QUAD=1','ENC_QUAD_PPR_M2=1000','ENC_QUAD_PPR_M3=512'])
    compile_case(['ENC_USE_HW_QUAD=0'])
    compile_case(['ENC_USE_HW_QUAD=1','ENC_QUAD_PPR=0'],False)
    compile_case(['ENC_USE_HW_QUAD=1','ENC_QUAD_PPR_M3=1025'],False)
    print('PASS: invalid zero/oversize PPR rejected at compile time',flush=True)
shutil.rmtree(tmp,ignore_errors=True)
