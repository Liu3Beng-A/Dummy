"""Compare production Curve::Build with frozen pre-fast-path13.04 oracle.
No USB/CAN/STLink or hardware; compile only native executables in build-dir.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,tempfile

p=argparse.ArgumentParser();p.add_argument('--build-dir');args=p.parse_args()
here=Path(__file__).resolve().parent;repo=here.parents[1]
root=repo/'firmware_new/dummy-motor-fw/Ctrl/Motor'
w=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='motor-hold-oracle-'))
w.mkdir(parents=True,exist_ok=True)
for name in ('stream_curve_hold_oracle.cpp','stream_curve_hold_oracle.h'):shutil.copyfile(here/'fixtures'/name,w/name)
shutil.copyfile(here/'motor_stream_hold_test.cpp',w/'motor_stream_hold_test.cpp')
identity={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (root/'stream_curve.cpp',root/'stream_curve.h',here/'fixtures/stream_curve_hold_oracle.cpp',here/'fixtures/stream_curve_hold_oracle.h',here/'motor_stream_hold_test.cpp')}
(w/'source-identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
if os.name=='nt':
 setup=''
 if not shutil.which('cl'):
  vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
  found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
  setup='call "'+str(Path(found)/'VC/Auxiliary/Build/vcvars64.bat')+'" > nul\nif errorlevel 1 exit /b 1\n'
 command='@echo off\n'+setup
 command+='cl /nologo /utf-8 /std:c++17 /O2 /EHsc /c stream_curve_hold_oracle.cpp /Fo:oracle.obj\nif errorlevel 1 exit /b 1\n'
 command+=f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /I"{root}" motor_stream_hold_test.cpp "{root/"stream_curve.cpp"}" oracle.obj /Fe:motor_stream_hold_test.exe\nif errorlevel 1 exit /b 1\nmotor_stream_hold_test.exe\nexit /b %errorlevel%\n'
 (w/'run.cmd').write_text(command,encoding='utf-8')
 result=subprocess.run(['cmd.exe','/d','/c',str(w/'run.cmd')],cwd=w,capture_output=True)
else:
 result=subprocess.run(['c++','-std=c++17','-O2','-I'+str(root),'-I'+str(w),str(w/'motor_stream_hold_test.cpp'),str(root/'stream_curve.cpp'),str(w/'stream_curve_hold_oracle.cpp'),'-o',str(w/'motor_stream_hold_test')],cwd=w,capture_output=True)
 if result.returncode==0:result=subprocess.run([str(w/'motor_stream_hold_test')],cwd=w,capture_output=True)
(w/'result.log').write_bytes(result.stdout+result.stderr)
print((result.stdout+result.stderr).decode('utf-8',errors='replace'))
raise SystemExit(result.returncode)
