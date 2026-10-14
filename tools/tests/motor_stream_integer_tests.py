"""Run production integer Curve + frozen floating + exact rational tests.

No serial, CAN, STLink or target firmware compilation is used.
  python tools/tests/motor_stream_integer_tests.py --output <native scratch>
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output','--build-dir',dest='output',type=Path)
    p.add_argument('--samples',type=int,default=100000)
    args=p.parse_args()
    if args.samples<1000:p.error('--samples must be >=1000')
    here=Path(__file__).resolve().parent;repo=here.parents[1]
    root=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor'
    out=args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix='motor-integer-tests-'))
    out.mkdir(parents=True,exist_ok=True)
    proof=subprocess.run([sys.executable,str(here/'motor_stream_integer_proof.py'),
        '--source',str(root/'stream_curve.cpp'),'--output',str(out)],capture_output=True)
    (out/'proof.log').write_bytes(proof.stdout+proof.stderr)
    print((proof.stdout+proof.stderr).decode(errors='replace'))
    if proof.returncode:raise SystemExit(proof.returncode)
    files=[here/'motor_stream_integer_test.cpp',root/'stream_curve.cpp',
        here/'fixtures/stream_curve_hold_oracle.cpp',here/'fixtures/stream_curve_float_1401_oracle.cpp']
    includes=[root,here/'fixtures']
    executable=out/('motor_stream_integer_test.exe' if os.name=='nt' else 'motor_stream_integer_test')
    if os.name=='nt':
        setup=''
        if not shutil.which('cl'):
            vs=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
            found=subprocess.check_output([str(vs),'-latest','-products','*',
                '-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
                '-property','installationPath'],text=True).strip()
            setup='call "'+str(Path(found)/'VC/Auxiliary/Build/vcvars64.bat')+'" > nul\nif errorlevel 1 exit /b 1\n'
        cmd='@echo off\n'+setup+'cl /nologo /std:c++17 /O2 /W4 /EHsc '
        cmd+=' '.join('/I"'+str(x)+'"' for x in includes)+' '
        cmd+=' '.join('"'+str(x)+'"' for x in files)+' /Fe:"'+str(executable)+'"\nif errorlevel 1 exit /b 1\n'
        cmd+='"'+str(executable)+'" "'+str(out/'exact_quintic.bin')+'" '+str(args.samples)+'\nexit /b %errorlevel%\n'
        (out/'build.cmd').write_text(cmd,encoding='utf8')
        result=subprocess.run(['cmd.exe','/d','/c',str(out/'build.cmd')],cwd=out,capture_output=True)
    else:
        compiler=shutil.which('c++') or shutil.which('clang++') or shutil.which('g++')
        if not compiler:raise SystemExit('Install a C++17 native compiler.')
        build=subprocess.run([compiler,'-std=c++17','-O2','-Wall','-Wextra']+
            ['-I'+str(x) for x in includes]+list(map(str,files))+['-o',str(executable)],cwd=out,capture_output=True)
        (out/'build.log').write_bytes(build.stdout+build.stderr)
        if build.returncode:
            print((build.stdout+build.stderr).decode(errors='replace'));raise SystemExit(build.returncode)
        result=subprocess.run([str(executable),str(out/'exact_quintic.bin'),str(args.samples)],cwd=out,capture_output=True)
    (out/'result.log').write_bytes(result.stdout+result.stderr)
    identity={str(x.relative_to(repo)):hashlib.sha256(x.read_bytes()).hexdigest()
              for x in files+[root/'stream_curve.h',here/'motor_stream_integer_proof.py']}
    (out/'identity.json').write_text(json.dumps(identity,indent=2),encoding='utf8')
    print((result.stdout+result.stderr).decode(errors='replace'))
    raise SystemExit(result.returncode)

if __name__=='__main__':main()
