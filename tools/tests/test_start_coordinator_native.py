"""Run the real ProtectionCoordinator against deterministic CAN/robot shims.

No board, device, firmware build, or source rewrite is used. Compile both the
frozen baseline and current complete coordinator, then compare their traces.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


HARNESS = r'''
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include "protection_protocol.h"
#include "stop_protocol.h"
static unsigned checks=0,critical=0;
#define CHECK(x) do { ++checks; if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::exit(1);} } while(0)
#define taskENTER_CRITICAL() (++critical)
#define taskEXIT_CRITICAL() (--critical)
static uint32_t nowMs=100;
static uint32_t HAL_GetTick(){return nowMs;}
struct Frame {unsigned at,node,command;uint8_t data[8];};
static std::vector<Frame> frames;
static bool canBusy=false;
struct CtrlStepMotor {
    unsigned node=0;bool inverseDirection=false;float angleLimitMin=-175,angleLimitMax=175,reduction=50,current=1;
    bool TrySendProtection(unsigned command,const uint8_t* data) {
        if(canBusy)return false;Frame f{nowMs,node,command,{}};std::memcpy(f.data,data,8);frames.push_back(f);return true;
    }
    void UpdateAngleCallback(float,bool){}
};
struct Pose {float a[6]{};};
class StreamSink {public:std::vector<std::string> lines;};
static StreamSink output;static StreamSink*usbStreamOutputPtr=&output;
static void Respond(StreamSink&s,const char*fmt,...){char b[256];va_list ap;va_start(ap,fmt);vsnprintf(b,sizeof(b),fmt,ap);va_end(ap);s.lines.emplace_back(b);}
struct CAN_context {
    struct ProbeTxCounters{unsigned accepted=0,txok=0,alst=0,terr=0,aborted=0,lastTxOk=0;}probeTxCounters[9];
    unsigned tx_arbitration_losses=0,tx_transmission_errors=0,RxFifo0FullCallbackCnt=0;
};
static CAN_context*get_can_ctx(void*){return nullptr;}
namespace FWIdentity {static const uint32_t WORDS[ProtectionProtocol::FW_VERSION_FIELD_COUNT]{};}
struct DummyRobot {
    enum StopState{NORMAL,STOPPING,STOPPED,STARTING,DISABLING,DISABLED};StopState stopState=DISABLED;
    bool isEnabled=false,motionActive=false,startPending=false,stopPerNodeTokens=false,stopWithGripper=false,stopNeedsSend=false;
    uint32_t motionEpoch=1;uint16_t stopToken=0,stopSyncedMask=0,stopAckMask=0,stopHoldingMask=0,stopFaultMask=0,stopFaultPrinted=0;
    uint8_t stopAction=0;void*pendingStopNotice=nullptr;void*hcan=nullptr;
    unsigned startCalls=0,startAt=0,disableCalls=0;
    Pose initPose,currentJoints,targetJoints,REST_POSE;float currentRailPos=0,targetRailPos=0;
    uint32_t motorPidLastUpdateMs[9]{},jointAccLastUpdateMs[9]{},jointCurrentLimitLastUpdateMs[9]{};
    struct Handler{void ClearFifo(){}}commandHandler;
    struct Tuning{void SetTuningFlag(int){}}tuningHelper;
    CtrlStepMotor values[8];CtrlStepMotor*motorJ[9]{};CtrlStepMotor*hand=&values[7];
    DummyRobot(){for(unsigned n=1;n<=8;++n){values[n-1].node=n;motorJ[n]=&values[n-1];}}
    void CancelMotion(){}void ResetStopDiagnostics(){}void UpdateJointAnglesCallback(){}
    bool IsRGBSaveBusy(){return false;}
    void RequestStart(){++startCalls;startAt=nowMs;stopState=STARTING;}
    void RequestDisable(){++disableCalls;stopState=DISABLING;}
};
#define private public
#include "protection_coordinator_under_test.h"
#undef private
#include "protection_coordinator_under_test.inc"
using namespace ProtectionProtocol;
static unsigned token(const uint8_t*b){return b[2]|(unsigned(b[3])<<8);}
static void put16(uint8_t*b,unsigned v){b[0]=uint8_t(v);b[1]=uint8_t(v>>8);}
struct Pending {unsigned at,node,command;uint8_t data[8];};
struct Rig {
    DummyRobot robot;ProtectionCoordinator p{robot};std::vector<Pending>pending;
    bool profile[9]{};unsigned version[9]{0,1,1,1,1,1,1,1,1};
    unsigned delay=0,dropCommand=0,dropNode=0;size_t consumed=0;
    Rig(){nowMs=100;frames.clear();canBusy=false;output.lines.clear();}
    void deliver(){for(size_t i=0;i<pending.size();){if(pending[i].at<=nowMs){const auto a=pending[i];pending.erase(pending.begin()+i);p.Receive(uint8_t(a.node),uint8_t(a.command),a.data);}else ++i;}}
    void replies(){while(consumed<frames.size()){
        const Frame f=frames[consumed++];if((!dropNode||dropNode==f.node)&&dropCommand==f.command)continue;
        Pending a{nowMs+delay,f.node,0,{}};
        if(f.command==CONTEXT&&f.data[0]==QUERY){a.command=PROBE_REPLY;a.data[0]=uint8_t(version[f.node]);
            a.data[1]=FEEDBACK_VALID|DISABLED_FLAG|(profile[f.node]?PROFILE_VALID:0);put16(a.data+2,token(f.data));float position=float(f.node)*.1f;std::memcpy(a.data+4,&position,4);
        }else if(f.command==CONFIG){a.command=PARAM_REPLY;put16(a.data+2,token(f.data));a.data[4]=f.data[0];a.data[5]=uint8_t(version[f.node]);
            if(f.data[0]==COMMIT)profile[f.node]=true;
        }else if(f.command==ProtectionProtocol::PARAM_SET){a.command=PARAM_REPLY;a.data[0]=f.data[0];put16(a.data+2,token(f.data));std::memcpy(a.data+4,f.data+4,4);
        }else continue;
        pending.push_back(a);
    }}
    void poll(bool respond=true){deliver();const auto n=frames.size();p.Poll();CHECK(frames.size()-n<=1);CHECK(!critical);if(respond)replies();}
    void step(bool respond=true){poll(respond);nowMs+=5;}
    void begin(){CHECK(p.InterceptStart());CHECK(p.RgbStarting());}
    void run(unsigned limit=3000){for(unsigned elapsed=0;elapsed<limit&&!robot.startCalls&&p.Busy();elapsed+=5)step();}
};
static std::string trace(const std::vector<Frame>& fs){std::string s;char b[100];for(const auto&f:fs){std::snprintf(b,sizeof(b),"%u:%u:%u:%u;",f.at-100,f.node,f.command,unsigned(f.data[0]));s+=b;}return s;}
static void printTrace(const char*key,const std::vector<Frame>&fs){std::printf("\"%s\":\"%s\",",key,trace(fs).c_str());}
int main(){
    unsigned nominal=0,delayed=0,busyTime=0,retryMin=0;
    {Rig r;r.begin();r.run();CHECK(r.robot.startCalls==1);CHECK(r.p.job==ProtectionCoordinator::NONE);nominal=r.robot.startAt-100;
        unsigned configs=0,sets=0,probes=0;for(const auto&f:frames){configs+=f.command==CONFIG;sets+=f.command==ProtectionProtocol::PARAM_SET;probes+=f.command==CONTEXT&&f.data[0]==QUERY;}
        CHECK(configs==14&&sets==21&&probes==16);std::printf("{");printTrace("nominal_trace",frames);}
    {Rig r;r.delay=35;r.begin();r.run();CHECK(r.robot.startCalls==1);delayed=r.robot.startAt-100;CHECK(delayed>nominal);}
    {Rig r;r.begin();canBusy=true;for(unsigned i=0;i<20;++i)r.step();CHECK(frames.empty()&&r.p.received==0&&r.robot.startCalls==0);
        canBusy=false;r.run();CHECK(r.robot.startCalls==1);busyTime=r.robot.startAt-100;CHECK(busyTime>=nominal+100);}
    // A missing BEGIN ACK cannot advance to parameter writes or unlock.
    {Rig r;r.dropCommand=CONFIG;r.dropNode=1;r.begin();r.run(1900);CHECK(!r.robot.startCalls);CHECK(r.p.locked);
        std::vector<unsigned>times;for(const auto&f:frames)if(f.command==CONFIG&&f.node==1){CHECK(f.data[0]==BEGIN);times.push_back(f.at);}
        CHECK(times.size()>2);retryMin=1000;for(size_t i=1;i<times.size();++i)retryMin=std::min(retryMin,times[i]-times[i-1]);CHECK(retryMin>=20);
        for(const auto&f:frames)CHECK(f.command!=ProtectionProtocol::PARAM_SET);}
    {Rig r;r.dropCommand=ProtectionProtocol::PARAM_SET;r.dropNode=1;r.begin();r.run(2000);CHECK(!r.robot.startCalls);CHECK(r.p.locked);
        for(const auto&f:frames)if(f.command==CONFIG)CHECK(f.data[0]!=COMMIT);}
    // Fresh-query ownership, legacy replies and invalid feedback remain gates.
    for(unsigned invalid=0;invalid<4;++invalid){Rig r;r.begin();while(frames.empty())r.step(false);CHECK(frames.size()==1);const auto f=frames[0];uint8_t a[8]{};
        a[0]=1;a[1]=FEEDBACK_VALID|DISABLED_FLAG;put16(a+2,token(f.data));float v=.25f;
        if(invalid==0)put16(a+2,token(f.data)+1);if(invalid==1)a[0]=0;
        if(invalid==2)a[1]&=~FEEDBACK_VALID;if(invalid==3)v=std::numeric_limits<float>::quiet_NaN();std::memcpy(a+4,&v,4);
        r.p.Receive(uint8_t(f.node),PROBE_REPLY,a);CHECK(!(r.p.received&(1U<<f.node)));CHECK(!r.robot.startCalls);
        if(invalid)CHECK(r.p.failed&(1U<<f.node));else CHECK(!r.p.failed);
        a[0]=QUERY_REPLY_FLAG;a[1]=0;r.p.Receive(uint8_t(f.node),CONTEXT_REPLY,a);CHECK(!(r.p.received&(1U<<f.node)));
        if(invalid){r.poll(false);CHECK(r.p.locked&&r.p.job==ProtectionCoordinator::NONE);CHECK(!r.robot.startCalls);}}
    // A COMMIT from the wrong coordinate generation cannot release START.
    {Rig r;r.begin();while(r.p.job!=ProtectionCoordinator::PROFILE)r.step();r.dropCommand=CONFIG;r.dropNode=1;
        while(frames.back().command!=CONFIG)r.step(false);CHECK(frames.back().command==CONFIG);uint8_t a[8]{};put16(a+2,r.p.task+1);a[4]=BEGIN;r.p.Receive(1,PARAM_REPLY,a);CHECK(!r.p.replyReady);
        r.p.stage=4;put16(a+2,r.p.task);a[4]=COMMIT;a[5]=uint8_t(r.p.nodes[1].version+1);r.p.Receive(1,PARAM_REPLY,a);CHECK(r.p.replyStatus==INVALID);
        r.poll(false);CHECK(r.p.locked&&r.p.job==ProtectionCoordinator::NONE);CHECK(!r.robot.startCalls);}
    // No response within 300 ms may masquerade as the final fresh proof.
    {Rig r;r.begin();while(r.p.job!=ProtectionCoordinator::START_VERIFY)r.step();
        const auto probe=r.p.queryProbe;for(unsigned n=1;n<=8;++n){uint8_t a[8]={1,uint8_t(FEEDBACK_VALID|PROFILE_VALID|DISABLED_FLAG),0,0};put16(a+2,probe);r.p.Receive(uint8_t(n),PROBE_REPLY,a);}
        nowMs+=305;r.poll(false);CHECK(!r.robot.startCalls);CHECK(r.p.job==ProtectionCoordinator::START_VERIFY);}
    for(unsigned broken=0;broken<2;++broken){Rig r;r.begin();while(r.p.job!=ProtectionCoordinator::START_VERIFY)r.step();
        for(unsigned n=1;n<=8;++n){uint8_t a[8]={1,uint8_t(FEEDBACK_VALID|PROFILE_VALID|DISABLED_FLAG),0,0};put16(a+2,r.p.queryProbe);
            if(n==3){if(broken==0)a[0]=2;else a[1]&=~PROFILE_VALID;}r.p.Receive(uint8_t(n),PROBE_REPLY,a);}
        r.poll(false);CHECK(!r.robot.startCalls);CHECK(r.p.locked&&r.p.job==ProtectionCoordinator::NONE);}
    // These unchanged branches must keep EXACT before/current transmission traces.
    {Rig r;CHECK(r.p.Command("!CALIBRATION",12,output));for(unsigned i=0;i<40;++i)r.step(false);printTrace("cal_query_trace",frames);}
    {Rig r;r.p.startAfterProfile=false;r.p.active=1;r.p.task=42;for(unsigned n=1;n<=8;++n)r.p.nodes[n].version=1;
        r.p.Enter(ProtectionCoordinator::PROFILE);for(unsigned i=0;i<40;++i)r.step();printTrace("nonstart_profile_trace",frames);}
    {Rig r;r.p.task=42;r.p.Enter(ProtectionCoordinator::CAL_FINISH,0x1FE);for(unsigned i=0;i<40;++i)r.step(false);printTrace("cal_group_trace",frames);}
    {Rig r;r.p.task=42;r.p.active=1;r.p.stalled=2;r.p.nodes[1].task=42;r.p.nodes[1].phase=WAIT_GRANT;r.p.nodes[1].event=55;
        r.p.Enter(ProtectionCoordinator::PROTECT_RUN);for(unsigned i=0;i<40;++i)r.step(false);printTrace("protect_run_trace",frames);}
    // A STOP frame already used this Poll's budget; START must not submit another.
    {Rig r;r.begin();r.p.StopAttempted();r.poll(false);CHECK(frames.empty());CHECK(!r.robot.startCalls);}
    std::printf("\"nominal_ms\":%u,\"delayed35_ms\":%u,\"busy100_ms\":%u,\"unacked_retry_min_ms\":%u,\"checks\":%u,\"hardware\":false}\n",nominal,delayed,busyTime,retryMin,checks);
}
'''


def build_run(source, header, out, repo, vcvars):
    out.mkdir(parents=True, exist_ok=True)
    firmware = repo / 'firmware_new'
    for name in ('protection_protocol.h', 'stop_protocol.h'):
        (out / name).write_bytes((firmware / name).read_bytes())
    (out / 'protection_coordinator_under_test.h').write_text(
        header.read_text(encoding='utf-8-sig').replace('../../../protection_protocol.h', 'protection_protocol.h'), encoding='utf-8')
    production = source.read_text(encoding='utf-8-sig')
    (out / 'protection_coordinator_under_test.inc').write_text('\n'.join(
        line for line in production.splitlines() if not line.lstrip().startswith('#include')), encoding='utf-8')
    (out / 'start_native.cpp').write_text(HARNESS, encoding='utf-8')
    cmd = out / 'build.cmd'
    cmd.write_text(f'@echo off\ncall "{vcvars}" > nul\nif errorlevel 1 exit /b 1\ncl /nologo /utf-8 /std:c++17 /O2 /EHsc /fp:precise start_native.cpp /Fe:start_native.exe\nexit /b %errorlevel%\n')
    built = subprocess.run(['cmd.exe', '/d', '/c', str(cmd)], cwd=out, capture_output=True)
    (out / 'build.log').write_bytes(built.stdout+built.stderr)
    if built.returncode:
        raise RuntimeError(f'Native compile failed: {out / "build.log"}\n{built.stdout.decode(errors="replace")}')
    run = subprocess.run([str(out / 'start_native.exe')], cwd=out, capture_output=True, text=True)
    (out / 'run.log').write_text(run.stdout+run.stderr, encoding='utf-8')
    if run.returncode:
        raise RuntimeError(f'Native test failed: {out / "run.log"}\n{run.stdout}\n{run.stderr}')
    return json.loads(run.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument('--baseline-dir', type=Path, help='Optional frozen coordinator source/header directory for before/current comparison')
    comparison.add_argument('--current-only', action='store_true', help='Run current code without requiring or comparing a baseline')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    qa = repo / 'firmware_artifacts/.qa/start-fast-20261016'
    out = (args.output or qa / ('native-current-only' if args.current_only else 'native')).resolve()
    core = repo / 'firmware_new/dummy-ref-core-fw/Robot/instances'
    before = (args.baseline_dir or qa / 'before-source').resolve()
    source, header = core/'protection_coordinator.cpp', core/'protection_coordinator.h'
    have_baseline = not args.current_only and all((before/name).is_file() for name in (source.name, header.name))
    if args.baseline_dir and not have_baseline:
        parser.error(f'Baseline directory must contain {source.name} and {header.name}: {before}')
    skipped_reason = ('current-only requested' if args.current_only else
                      'default QA baseline is unavailable; current-code checks still run')
    inputs = [source, header]
    if have_baseline:
        inputs.extend((before/source.name, before/header.name))
    hashes = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    frozen = hashes()
    vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    vs = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], text=True).strip()
    vcvars = Path(vs)/'VC/Auxiliary/Build/vcvars64.bat'
    old = build_run(before/source.name, before/header.name, out/'before', repo, vcvars) if have_baseline else None
    new = build_run(source, header, out/'current', repo, vcvars)
    assert new['unacked_retry_min_ms'] == 20
    new_trace = new.pop('nominal_trace')
    traces = {'current': new_trace}
    if old is not None:
        for key in ('cal_query_trace','nonstart_profile_trace','cal_group_trace','protect_run_trace'):
            assert old[key] == new[key], f'Unrelated pacing changed: {key}'
        assert new['nominal_ms'] < old['nominal_ms']/2
        traces['before'] = old.pop('nominal_trace')
    new_frames = [tuple(map(int, frame.split(':'))) for frame in new_trace.strip(';').split(';')]
    for start, end in ((0, 8), (8, 43), (43, 51)):
        times = [frame[0] for frame in new_frames[start:end]]
        assert len(times) == end-start and all(b-a == 5 for a,b in zip(times,times[1:])), 'Successful START stages must advance at 5 ms'
    (out/'nominal-traces.json').write_text(json.dumps(traces,indent=2),encoding='utf-8')
    assert hashes() == frozen, 'Production changed during tests; rerun against final source'
    result = {'before':old,'current':new,'baseline_compared':have_baseline,
              'baseline_comparison':str(before) if have_baseline else skipped_reason,
              'input_sha256':frozen,'hardware':False,
              'scope':'Actual entire coordinator Poll/Receive/Profile; RequestStart is an observed external boundary, not the final motor ACK/PollSoftStop completion.'}
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
