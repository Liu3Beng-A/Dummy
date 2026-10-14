"""Compile actual native Cartesian producer/queue/settling implementations.
All HAL/CAN/RTOS/physical responses are bounded host scaffolds; no hardware.
Windows requires installed Visual Studio x64 C++ tools. Outputs remain outside
the repository unless --output is explicitly provided. --fast uses the host
CMSIS512 interpolation shim and MSVC fast FP, not ARM timing/library identity.
"""
from pathlib import Path
import os,subprocess,hashlib,json,argparse,tempfile
p=argparse.ArgumentParser();p.add_argument('--fast',action='store_true');p.add_argument('--output',type=Path);args=p.parse_args()
here=Path(__file__).resolve().parent;repo=here.parents[1];out=(args.output or Path(tempfile.mkdtemp(prefix='native-cartesian-robot-'))).resolve();out.mkdir(parents=True,exist_ok=True)
core=repo/'firmware_new/dummy-ref-core-fw/Robot/instances';math=core.parent/'algorithms/kinematic'
production=[core/n for n in ['core_stream.cpp','core_stream.h','core_stream_engine.cpp','core_stream_engine.h','native_cartesian_robot.cpp','dummy_robot.cpp','dummy_robot.h','cartesian_planner.cpp','cartesian_planner.h','motion_values.cpp','motion_values.h']]+[math/'6dof_kinematic.cpp',math/'6dof_kinematic.h',repo/'firmware_new/stream_protocol.h',repo/'firmware_new/motion_completion_protocol.h',repo/'firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in production}
def function(text,name):
 a=text.index(name);b=text.index('{',a);d=1;c=b+1
 while d:d+=(text[c]=='{')-(text[c]=='}');c+=1
 return text[a:c]
adapter=(core/'core_stream.cpp').read_text(encoding='utf-8')
(out/'adapter.inc').write_text('\n'.join(x for x in adapter.splitlines() if not x.lstrip().startswith('#include')),encoding='utf-8')
robot=(core/'dummy_robot.cpp').read_text(encoding='utf-8');native=(core/'native_cartesian_robot.cpp').read_text(encoding='utf-8')
parts=[function(robot,s) for s in [
 'void DummyRobot::MoveJoints(', 'void DummyRobot::UpdateJointAngles()',
 'uint16_t DummyRobot::FreshMotionCompletionMask()', 'bool DummyRobot::MotionCompletionCallback(',
 'void DummyRobot::BeginMotionRound()', 'void DummyRobot::PollMotionCompletion()',
 'void DummyRobot::CancelMotion()', 'bool DummyRobot::WaitMotionCompletion(',
 'uint32_t DummyRobot::CommandHandler::Push(', 'std::string DummyRobot::CommandHandler::Pop(',
 'uint32_t DummyRobot::CommandHandler::GetSpace()', 'uint32_t DummyRobot::CommandHandler::ParseCommand(',
 'bool DummyRobot::CommandHandler::IsAnnounced(', 'void DummyRobot::CommandHandler::Announce(']]
namespace=native[native.index('namespace {'):native.index('bool Equal(')]+'}\n'
if 'bool ExactMotorR(' in native:
    namespace=namespace[:-2]+function(native,'bool ExactMotorR(')+'\n}\n'
(out/'robot.inc').write_text(namespace+function(native,'bool DummyRobot::RunCartesian(')+'\n'+'\n'.join(parts),encoding='utf-8')
header=(core/'dummy_robot.h').read_text(encoding='utf-8')
start=header.index('    enum NativePhase') if '    enum NativePhase' in header else header.index('    volatile bool motionActive =')
fields=header[start:header.index('    volatile bool stopNeedsSend =')]
fields=fields.replace('    void FailMotionDispatch(uint32_t epoch, uint32_t generation, const char* reason, uint16_t wait);','    void FailMotionDispatch(uint32_t,uint32_t,const char* reason,uint16_t){motionFailureReason=reason;motionPhase=MOTION_FAILED;motionActive=false;RequestSoftStop();}')
(out/'fields.inc').write_text(fields,encoding='utf-8')
motor=(repo/'firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp').read_text(encoding='utf-8')
probe=function(motor,'uint8_t Motor::Controller::GetPositionCompletionStatus(').replace('Motor::Controller::GetPositionCompletionStatus','MotorProbe::Status')
(out/'motor_completion.inc').write_text(probe,encoding='utf-8')
shutil_source=(here/'native_cartesian_robot_test.cpp').read_text(encoding='utf-8')
old=function(shutil_source,' bool CanSendMotion(')
shutil_source=shutil_source.replace(old,function(header,'    bool CanSendMotion('),1)
(out/'root_integration.cpp').write_text(shutil_source,encoding='utf-8')
(out/'arm_math.h').write_bytes((here/'fixtures/arm_math_native_shim.h').read_bytes())
vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
found=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
cmd='@echo off\ncall "'+str(Path(found)/'VC/Auxiliary/Build/vcvars64.bat')+'" > nul\nif errorlevel 1 exit /b 1\n'
options='/fp:fast /DCMSIS_INTERPOLATED' if args.fast else '/fp:precise'
cmd+=f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc {options} /I"{repo/"firmware_new"}" /I"{core}" /I"{math}" /I"{out.resolve()}" "{out.resolve()/"root_integration.cpp"}" "{core/"core_stream_engine.cpp"}" "{core/"cartesian_planner.cpp"}" "{core/"motion_values.cpp"}" "{math/"6dof_kinematic.cpp"}" /Fe:root_integration.exe\nif errorlevel 1 exit /b 1\nroot_integration.exe\nexit /b %errorlevel%\n'
(out/'build.cmd').write_text(cmd,encoding='utf-8')
r=subprocess.run(['cmd.exe','/d','/c',str((out/'build.cmd').resolve())],cwd=out,capture_output=True)
(out/'result.log').write_bytes(r.stdout+r.stderr);print((r.stdout+r.stderr).decode('utf-8',errors='replace'))
after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in production}
(out/'identity.json').write_text(json.dumps({'before':before,'after':after,'hardware':False,'production_written':False,'result':r.returncode},indent=2),encoding='utf-8')
if before!=after:print('Concurrent production change detected: rerun frozen source.');raise SystemExit(3)
raise SystemExit(r.returncode)
