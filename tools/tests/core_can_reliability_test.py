"""Compile actual CAN adapter against mailbox/IRQ stubs; no hardware access."""
from pathlib import Path
import argparse,subprocess,shutil,tempfile,os
here=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path(tempfile.gettempdir())/'core-can-reliability');args=parser.parse_args()
repo=here.parents[1];core=repo/'firmware_new/dummy-ref-core-fw';out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
for source,name in [(core/'Bsp/communication/interface_can.cpp','interface_can_under_test.inc'),(core/'Bsp/communication/interface_can.hpp','interface_can_header_under_test.inc')]:
 out.joinpath(name).write_text('\n'.join(line for line in source.read_text(encoding='utf-8-sig').splitlines() if not line.lstrip().startswith('#include')),encoding='utf-8')
shutil.copyfile(repo/'firmware_new/stream_protocol.h',out/'stream_protocol_under_test.h')
setup=''
if not shutil.which('cl'):
 vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
 found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
 setup='call "'+str(Path(found)/'VC/Auxiliary/Build/vcvars64.bat')+'" > nul\nif errorlevel 1 exit /b 1'
cmd=f'@echo off\n{setup}\ncl /nologo /utf-8 /std:c++20 /O2 /EHsc /I"{out}" "{here/"core_can_reliability_test.cpp"}" /Fe:core_can_reliability.exe > build.log 2>&1\nif errorlevel 1 exit /b 1\ncore_can_reliability.exe > result.txt 2>&1\nexit /b %errorlevel%\n'
(out/'run.cmd').write_text(cmd,encoding='utf-8');result=subprocess.run(['cmd','/d','/c',str(out/'run.cmd')],cwd=out)
for name in ['build.log','result.txt']:
 f=out/name
 if f.exists(): print(f.read_text(encoding='utf-8',errors='replace')[-9000:])
raise SystemExit(result.returncode)
