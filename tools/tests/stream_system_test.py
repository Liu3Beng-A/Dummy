"""Windows/MSVC cross-layer offline replay, with deterministic fault injection.
The plant and retained CAN frames are idealized: this test does NOT exercise
production mailbox ownership, NART retries, queue capacity or physical timing.
"""
from pathlib import Path
from motor_entry_test_helpers import completion_source
import subprocess,re,sys,argparse,tempfile,shutil,os
here=Path(__file__).resolve().parent
args=argparse.ArgumentParser(description='Offline actual core + seven motor adapters. No hardware access.')
args.add_argument('--build-dir');args=args.parse_args()
p=Path(args.build_dir).resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix='stream-system-'))
p.mkdir(parents=True,exist_ok=True)
root=here.parents[1]/'firmware_new'
shutil.copyfile(here/'stream_system_test.cpp',p/'combined_sim.cpp')
motor=root/'dummy-motor-fw/Ctrl/Motor'
core=root/'dummy-ref-core-fw/Robot/instances'
shim=(here/'motor_stream_test_shim.h').read_text()
shim='\n'.join(line for line in shim.splitlines() if not line.startswith(('#include','#pragma once')))
production=(motor/'stream_controller.cpp').read_text(encoding='utf-8')
production='\n'.join(line for line in production.splitlines() if not line.startswith('#include'))
namespaces=[]
for node in range(1,8):
    text=f'namespace node{node} {{\n'+shim+'\n'
    text+='std::vector<ReplyFrame> replies; BoardConfig boardConfig; uint32_t irqMask;\n'
    text+='bool CAN_TrySend(CAN_TxHeaderTypeDef*h,const uint8_t*b){return ::motorSend(h->StdId,b); }\n'
    # Compatibility only: the legacy model retains unlimited replies reliably.
    # Actual bounded retry/ownership behavior is covered by actual CAN link tests.
    text+='bool CAN_QueueStreamReply(CAN_TxHeaderTypeDef*h,const uint8_t*b){return ::motorSend(h->StdId,b); }\n'
    text+=f'void CAN_CancelStreamReplies(){{::motorCancel({node});}}\n'
    text+='bool CAN_StreamReplyFault(){return false;} uint32_t CAN_StreamReplyDiagnostic(uint8_t){return 0;}\n' 
    text+=completion_source(here.parents[1])+'\n'+production+'\n'
    text+='Motor device; Motor::Controller controller; unsigned stableTicks;\n'
    text+=f'''void Reset(int32_t q){{stream=MotorStream::Runtime{{}};packetRead=packetWrite=0;packetOverflow=false;generation=0;irqMask=0;memset(entrySnapshot,0,sizeof(entrySnapshot));
controller=Motor::Controller{{}};controller.context=&device;controller.realPosition=controller.goalPosition=controller.softPosition=q;boardConfig.canNodeId={node};stableTicks=0;controller.ResetPositionCompletion();for(unsigned i=0;i<1000;++i){{controller.TickStreamClock();controller.TickPositionCompletion();}}}}
void Receive(uint8_t cmd,const uint8_t*p){{controller.ProcessStreamCan(cmd,p,8);}}
void Poll(){{for(unsigned i=0;i<3;++i)controller.PollStreamMain();}}
void Tick(){{controller.TickStreamClock();controller.modeRunning=controller.requestMode;if(controller.modeRunning==Motor::MODE_STREAM_JOINT)controller.TickStream();
controller.realPosition=controller.softPosition;controller.estVelocity=controller.softVelocity;
controller.TickPositionCompletion();}}
uint32_t Anchor(){{return stream.armed?stream.anchor:0;}}uint16_t Completed(){{return stream.completed;}}
uint8_t State(){{return stream.state;}}uint8_t Reason(){{return stream.reason;}}int32_t Position(){{return controller.goalPosition;}}bool Locked(){{return controller.locked;}}
void Stop(){{controller.RequestSoftStop(SoftStopProtocol::STOP,123);}}
}}\n'''
    namespaces.append(text)
# _estack is a linkersymbol on target; the fake numeric address is not a RAM measurement.
(p/'seven_motor_nodes.inc').write_text('extern "C" uint32_t _estack=0;\n'+'\n'.join(namespaces),encoding='utf-8')
setup=''
if not shutil.which('cl'):
    vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
    setup='call "'+str(Path(found)/'VC/Auxiliary/Build/vcvars64.bat')+'" > nul\nif errorlevel 1 exit /b 1'
cmd=f'''@echo off
{setup}
cl /nologo /utf-8 /std:c++17 /O2 /EHsc /I"{root}" /I"{core}" /I"{motor}" combined_sim.cpp "{core/'core_stream_engine.cpp'}" "{motor/'stream_curve.cpp'}" "{motor/'stream_runtime.cpp'}" /Fe:combined_sim.exe > combined_compile.log 2>&1
if errorlevel 1 exit /b 1
combined_sim.exe "{here/'fixtures/seven_axis_25mm_35pct.txt'}" > combined_result.txt 2>&1
exit /b %errorlevel%
'''
(p/'run_combined.cmd').write_text(cmd,encoding='utf-8')
r=subprocess.run(['cmd.exe','/d','/c',str(p/'run_combined.cmd')],cwd=p)
for name in ['combined_compile.log','combined_result.txt']:
    f=p/name
    if f.exists():print(f.read_text(errors='replace')[-9000:])
raise SystemExit(r.returncode)
