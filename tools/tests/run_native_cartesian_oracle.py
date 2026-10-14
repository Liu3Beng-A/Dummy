"""Compile real firmware math/planner/Curve and run independent numerical oracle.

Requires Python3 + NumPy and MSVC (Windows) or a C++17 host compiler.
All build products/logs are written to the explicit --output directory.
No firmware build, serial, STLink, flashing, motion or source modification.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True,help='Scratch directory for host executable, curves and reports')
parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
parser.add_argument('--compiler',choices=('auto','msvc','c++'),default='auto')
parser.add_argument('--timeout',type=int,default=180,help='Per executable/oracle timeout seconds')
args=parser.parse_args()
repo=args.repo.resolve();out=args.output.resolve()
if out==repo or out in repo.parents or out==repo/'tools/tests':
    parser.error('--output must be a scratch directory, not a repository root or tests source directory')
out.mkdir(parents=True,exist_ok=True)
tests=repo/'tools/tests';stub=out/'stubs';stub.mkdir(exist_ok=True)
shutil.copyfile(tests/'fixtures/native_cartesian_arm_math.h',stub/'arm_math.h')
core=repo/'firmware_new/dummy-ref-core-fw';motor=repo/'firmware_new/dummy-motor-fw'
sources=[core/'Robot/instances/cartesian_planner.cpp',core/'Robot/instances/cartesian_planner.h',core/'Robot/algorithms/kinematic/6dof_kinematic.cpp',core/'Robot/algorithms/kinematic/6dof_kinematic.h',motor/'Ctrl/Motor/stream_curve.cpp',motor/'Ctrl/Motor/stream_curve.h',repo/'firmware_new/stream_protocol.h']
def pin():return {str(path.relative_to(repo)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
before=pin()
env=dict(os.environ)
msvc=args.compiler=='msvc' or (args.compiler=='auto' and os.name=='nt')
executable=out/('test.exe' if os.name=='nt' else 'test')
includes=[stub,core/'Robot/instances',motor/'Ctrl/Motor']
cpp=[tests/'native_cartesian_oracle_driver.cpp']+[p for p in sources if p.suffix=='.cpp']
def run(command,log,timeout):
    process=subprocess.run(command,cwd=out,env=env,text=True,encoding='utf-8',errors='replace',stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout)
    (out/log).write_text(process.stdout,encoding='utf-8')
    print(process.stdout,flush=True)
    if process.returncode:raise subprocess.CalledProcessError(process.returncode,command)
if msvc:
    cl=shutil.which('cl.exe')
    if not cl:
        vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
        installation=None
        if vswhere.exists():
            location=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
            if location:installation=Path(location)
        if installation is None:
            fallback=Path('D:/Visual Studio/2022/Community')
            if fallback.exists():installation=fallback
        if installation is None:raise RuntimeError('MSVC not found; run from an x64 Developer shell or select --compiler c++')
        vcvars=installation/'VC/Auxiliary/Build/vcvars64.bat'
        if any(c in str(vcvars) for c in '"%\r\n'):raise ValueError('invalid Visual Studio path for environment batch')
        env_batch=out/'msvc_environment.cmd'
        env_batch.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\nif errorlevel 1 exit /b 1\nset\n',encoding='utf-8')
        raw=subprocess.check_output(['cmd.exe','/d','/c',str(env_batch)],text=True,errors='replace')
        for line in raw.splitlines():
            if '=' in line and not line.startswith('='):
                key,value=line.split('=',1);env[key.upper()]=value
        cl=shutil.which('cl.exe',path=env.get('PATH'))
    if not cl:raise RuntimeError('MSVC cl.exe not found in initialized environment')
    command=[cl,'/nologo','/std:c++17','/O2','/fp:fast','/DCMSIS_INTERPOLATED','/EHsc','/utf-8']+[f'/I{p}' for p in includes]+[str(p) for p in cpp]+[f'/Fe:{executable}',f'/Fo:{out}\\']
else:
    compiler=shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')
    if not compiler:raise RuntimeError('C++17 host compiler not found')
    command=[compiler,'-std=c++17','-O2','-ffast-math','-DCMSIS_INTERPOLATED']+[f'-I{p}' for p in includes]+[str(p) for p in cpp]+['-o',str(executable)]
try:
    run(command,'build.log',args.timeout)
    run([str(executable),str(out/'curves.json')],'result.log',args.timeout)
    run([sys.executable,str(tests/'native_cartesian_oracle.py'),'--output',str(out)],'oracle.log',args.timeout)
finally:
    after=pin()
    (out/'source-pin.json').write_text(json.dumps(dict(before=before,after=after,unchanged=before==after),indent=2),encoding='utf-8')
if before!=after:raise RuntimeError('Production sources changed during the oracle run; result is not pinned')
report=json.loads((out/'oracle.json').read_text(encoding='utf-8'))
print('oracle PASS' if report['passed'] else 'oracle FAIL')
raise SystemExit(0 if report['passed'] else 1)
