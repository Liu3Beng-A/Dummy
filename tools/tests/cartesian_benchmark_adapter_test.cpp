// Host shim for the UNMODIFIED extracted production adapter. Fake time drives
// safety/control-flow edge cases; no result here measures MCU execution time.
#include "cartesian_benchmark.h"
#include "bench_profile.h"
#include "cartesian_endurance.h"
#include "cartesian_cycle_diagnostic.h"
#include "motion_values.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <new>
#include <string>
#include <vector>

static unsigned assertions=0;
#define CHECK(x) do {++assertions;if(!(x)){std::fprintf(stderr,"FAIL line=%d: %s\n",__LINE__,#x);std::exit(1);}} while(0)
using TickType_t=uint32_t;
using osThreadId_t=void*;
osThreadId_t ControlLoopUpdateHandle=reinterpret_cast<void*>(1);
osThreadId_t controlLoopFixUpdateHandle=reinterpret_cast<void*>(2);
osThreadId_t oledTaskHandle=reinterpret_cast<void*>(3);
osThreadId_t rgbTaskHandle=reinterpret_cast<void*>(4);
osThreadId_t usbServerTaskHandle=reinterpret_cast<void*>(5);
osThreadId_t oledDisplayTaskHandle=reinterpret_cast<void*>(6);
static osThreadId_t currentThread=ControlLoopUpdateHandle;
// HAL TIM6 and RTOS SysTick are independent counters on the target. Keep
// separate fake counters so a test cannot accidentally prove their equality.
static uint32_t fakeTicks=0,fakeRtosTicks=0;
uint32_t SystemCoreClock=168000000;
static uint32_t cycleReadStep=100,criticalDepth=0;
static bool clockFrozen=false,hookEveryRead=false;
static uint64_t clockRemainder=0;
static unsigned cycleReads=0,injectOnRead=0;
static std::function<void()> cycleReadHook;
static std::function<void()> enterCriticalHook;
static std::function<void(uint32_t)> delayHook;
constexpr uint32_t DWT_CTRL_CYCCNTENA_Msk=1,CoreDebug_DEMCR_TRCENA_Msk=1UL<<24;
struct CycleRegister {
    uint32_t value=0;
    unsigned writes=0;
    operator uint32_t();
    CycleRegister& operator=(uint32_t v){value=v;++writes;return *this;}
};
struct FakeDWT { uint32_t CTRL=0;CycleRegister CYCCNT; } fakeDwt;
struct FakeDebug { uint32_t DEMCR=0; } fakeDebug;
FakeDWT* DWT=&fakeDwt;
FakeDebug* CoreDebug=&fakeDebug;
static void AdvanceCycles(uint32_t n){
    DWT->CYCCNT.value+=n;
    clockRemainder+=uint64_t(n)*1000;
    const uint32_t elapsed=uint32_t(clockRemainder/SystemCoreClock);
    fakeTicks+=elapsed;fakeRtosTicks+=elapsed;
    clockRemainder%=SystemCoreClock;
}
CycleRegister::operator uint32_t(){
    ++cycleReads;
    if(cycleReadHook&&(hookEveryRead||cycleReads==injectOnRead))cycleReadHook();
    if(!clockFrozen&&(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)&&(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk))AdvanceCycles(cycleReadStep);
    return value;
}
static void __DSB(){}
static void __ISB(){}
static void __NOP(){if(!clockFrozen&&(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk))AdvanceCycles(4);}
static uint32_t HAL_GetTick(){return fakeTicks;}
static osThreadId_t osThreadGetId(){return currentThread;}
static TickType_t xTaskGetTickCount(){return fakeRtosTicks;}
static void EnterCritical(){if(enterCriticalHook){auto hook=enterCriticalHook;enterCriticalHook={};hook();}++criticalDepth;}
static void ExitCritical(){CHECK(criticalDepth>0);--criticalDepth;}
#define taskENTER_CRITICAL() EnterCritical()
#define taskEXIT_CRITICAL() ExitCritical()
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(x) (x)
static void osDelay(uint32_t n){CHECK(criticalDepth==0);fakeTicks+=n;fakeRtosTicks+=n;if(!clockFrozen)DWT->CYCCNT.value+=uint32_t(uint64_t(SystemCoreClock)*n/1000);if(delayHook)delayHook(n);}
static void vTaskDelayUntil(TickType_t* wake,TickType_t period){const uint32_t target=*wake+period;if(int32_t(target-fakeRtosTicks)>0)osDelay(target-fakeRtosTicks);*wake=target;}
static uint32_t osThreadGetStackSpace(osThreadId_t task){CHECK(criticalDepth==0);return task==ControlLoopUpdateHandle?6400U:task==oledTaskHandle?1300U:1700U;}

struct StreamSink {enum { CHANNEL_TYPE_USB,CHANNEL_TYPE_UART };int channelType=CHANNEL_TYPE_USB;std::vector<std::string> lines;};
static unsigned maxRespondPayload=0;
static void Respond(StreamSink& sink,const char* format,...){
    // Production Respond has 64 bytes including CR/LF, and snprintf receives
    // 62 bytes including its terminator. Assert rather than hide truncation.
    char s[512];va_list args;va_start(args,format);const int n=std::vsnprintf(s,sizeof(s),format,args);va_end(args);
    CHECK(n>=0);CHECK(n<62);maxRespondPayload=std::max(maxRespondPayload,unsigned(n));sink.lines.emplace_back(s);
}
static bool streamActive=false,nativeOwned=false;
static bool CoreStreamActive(){return streamActive;}
static bool CoreNativeOwned(){return nativeOwned;}
struct FakeProtection {
    unsigned calls=0;bool intercept=false;
    std::function<void()> commandHook;
    bool Command(const char*,unsigned,StreamSink&){++calls;if(commandHook)commandHook();return true;}
    bool InterceptStart(){return intercept;}
};
struct FakeRGB {unsigned endBoot=0;void EndBoot(){++endBoot;}} rgb;
struct DummyRobot {
    enum StopState { NORMAL,STOPPING,STOPPED,STARTING,DISABLING,DISABLED };
    struct Queue {bool busy=false;bool IsBusy()const{return busy;}} commandHandler;
    StopState stopState=DISABLED;
    bool isEnabled=false,startPending=false,motionActive=false,rgbSaveBusy=false;
    bool calibration=false,parameter=false,locked=false,tuning=false;
    uint32_t motionEpoch=7;
    bool stopStartAllEight=false,stopWithGripper=false,stopPerNodeTokens=false;
    bool stopDiagPending=false,stopNeedsSend=false;
    uint32_t stopAckMask=0;
    struct SoftStopProtocol {enum {QUERY=3};};int stopAction=0;
    FakeProtection* protection=nullptr;
    uint32_t GetMotionEpoch()const{return motionEpoch;}
    bool IsEnabled()const{return isEnabled;}
    bool IsStopLocked()const{return locked||(stopState!=NORMAL&&stopState!=DISABLED);}
    bool IsMotionBusy()const{return motionActive||streamActive||nativeOwned||tuning;}
    bool IsCalibrationBusy()const{return calibration;}
    bool IsProtectionParameterBusy()const{return parameter;}
    bool IsRGBSaveBusy()const{return rgbSaveBusy;}
    void ResetStopDiagnostics(){}
    bool HandleCartesianCommand(const char*,unsigned,StreamSink&);
    bool RequestStart();
    bool HandleProtectionCommand(const char*,unsigned,StreamSink&);
} dummy;
using SoftStopProtocol=DummyRobot::SoftStopProtocol;

// Fake task-owner bridge: tests adapter lifecycle/admission, never a scheduler.
namespace OledDiagnostic {
static RuntimeSnapshot runtime{};
static bool busy=false, failStart=false, failStop=false;
static unsigned starts=0, stops=0;
static std::function<void()> startHook,stopHook;
bool Start(unsigned mode) {
    CHECK(criticalDepth==0); ++starts; runtime=RuntimeSnapshot{};
    runtime.priority=mode==4?16U:mode==3?24U:0U;runtime.displayStackFree=1800;
    busy=mode>=3;runtime.restored=unsigned(!busy);
    if(startHook)startHook();return !failStart;
}
bool Stop() {
    CHECK(criticalDepth==0);++stops;if(stopHook)stopHook();
    if(failStop){busy=true;runtime.restored=0;return false;}
    busy=false;runtime.restored=1;return true;
}
bool Busy(){return busy;}
RuntimeSnapshot Snapshot(){CHECK(criticalDepth==0);return runtime;}
}

// Generated directly from source frozen by the Python runner.
#include "adapter_fragment.inc"

static StreamSink usb,uart{StreamSink::CHANNEL_TYPE_UART,{}};
static bool Command(const std::string& text,StreamSink& response=usb){response.lines.clear();return dummy.HandleCartesianCommand(text.data(),unsigned(text.size()),response);}
static bool Has(const StreamSink& s,const char* text){for(const auto& line:s.lines)if(line.find(text)!=std::string::npos)return true;return false;}
static void Reset(){
    CHECK(!BenchActive());dummy=DummyRobot{};benchJob=BenchJob{};benchCalibrationEntry=false;plannerLease=false;
    BenchProfile::Reset();BenchProfile::OledSessionReset();BenchProfile::SetTasks(nullptr,nullptr,nullptr,nullptr,nullptr,nullptr);
    new (&benchResults.legacy) LegacyResults{};benchResultsFull=false;
    streamActive=nativeOwned=false;currentThread=ControlLoopUpdateHandle;fakeTicks=fakeRtosTicks=0;cycleReadStep=100;
    clockFrozen=hookEveryRead=false;clockRemainder=0;SystemCoreClock=168000000;fakeDwt=FakeDWT{};fakeDebug=FakeDebug{};
    cycleReads=injectOnRead=0;cycleReadHook={};delayHook={};enterCriticalHook={};
    rgb=FakeRGB{};usb.lines.clear();uart.lines.clear();CHECK(criticalDepth==0);
    OledDiagnostic::runtime=OledDiagnostic::RuntimeSnapshot{};
    OledDiagnostic::busy=OledDiagnostic::failStart=OledDiagnostic::failStop=false;
    OledDiagnostic::starts=OledDiagnostic::stops=0;OledDiagnostic::startHook={};OledDiagnostic::stopHook={};
}
static void Start(const char* args="2,0,1,3,20000"){
    CHECK(Command(std::string("#CBENCH RUN ")+args));CHECK(benchJob.phase==BenchPhase::PENDING);
}
static void Stop(){if(BenchActive()){Command("#CBENCH CANCEL");CartesianBenchmarkPoll();}CHECK(!BenchActive());}
static void Finish(){for(unsigned i=0;i<180&&BenchActive();++i)CartesianBenchmarkPoll();CHECK(!BenchActive());}

static std::string Fields(const char* prefix,std::initializer_list<uint32_t> values){
    std::string line=prefix;bool first=true;
    for(uint32_t value:values){if(!first)line+=',';first=false;line+=std::to_string(value);}
    return line;
}
static void CheckFrame(unsigned index){
    const auto& frame=benchResults.legacy.frames[index];const auto& sample=benchResults.legacy.data[index];
    CHECK(frame.flags==0);CHECK(!benchProfileWindowActive);
    uint32_t sum=0;for(uint32_t value:frame.resident)sum+=value;
    CHECK(sum==uint32_t(frame.endDwt-frame.startDwt));CHECK(sum>=sample.cycles);
    for(unsigned i=0;i<sample.work;++i){CHECK(frame.workCycles[i]>0);if(i)CHECK(frame.workCycles[i]>frame.workCycles[i-1]);}
    for(unsigned i=sample.work;i<BenchProfile::MaxWork;++i)CHECK(frame.workCycles[i]==0);
    if(sample.work)CHECK(frame.workCycles[sample.work-1]<=sample.cycles);
}
static void CheckProfileWire(unsigned first,unsigned amount){
    CHECK(Command("#CBENCH PREAD "+std::to_string(first)+","+std::to_string(amount)));
    unsigned line=0;
    auto expect=[&](const std::string& text){CHECK(line<usb.lines.size());CHECK(usb.lines[line++]==text);};
    for(unsigned index=first;index<first+amount;++index){
        const auto& frame=benchResults.legacy.frames[index];const auto& sample=benchResults.legacy.data[index];const uint32_t id=benchJob.id;
        expect(Fields("CBPROF S ",{id,index,frame.plannedRtos,frame.startRtos,frame.endRtos}));
        expect(Fields("CBPROF H ",{id,index,frame.startHal,frame.endHal,uint32_t(frame.startRtos-frame.plannedRtos)}));
        expect(Fields("CBPROF D ",{id,index,frame.startDwt,frame.endDwt,uint32_t(frame.endDwt-frame.startDwt)}));
        for(unsigned i=0;i<sample.work;++i)expect(Fields("CBPROF W ",{id,index,i,frame.workCycles[i]}));
        for(unsigned i=0;i<BenchProfile::TaskCount;++i)expect(Fields("CBPROF T ",{id,index,i,frame.resident[i]}));
        expect(Fields("CBPROF E ",{id,index,sample.work,frame.flags}));
    }
    CHECK(line==usb.lines.size());for(const auto& text:usb.lines)CHECK(text.size()<62);
}

static void CheckOledFrame(unsigned index){
    CheckFrame(index);const auto& frame=benchResults.legacy.frames[index];uint32_t resident=0,elapsed=0;
    for(unsigned i=0;i<BenchProfile::OledStageCount;++i){
        CHECK(frame.oledResident[i]<=frame.oledElapsed[i]);resident+=frame.oledResident[i];elapsed+=frame.oledElapsed[i];
    }
    CHECK(resident==frame.resident[2]);CHECK(elapsed==uint32_t(frame.endDwt-frame.startDwt));
    uint32_t dr=0,de=0;
    for(unsigned i=0;i<BenchProfile::OledStageCount;++i){
        CHECK(frame.displayResident[i]<=frame.displayElapsed[i]);dr+=frame.displayResident[i];de+=frame.displayElapsed[i];
    }
    CHECK(dr==frame.resident[5]);CHECK(de==uint32_t(frame.endDwt-frame.startDwt));
}
static void CheckOledWire(unsigned first,unsigned amount){
    CHECK(Command("#CBENCH OREAD "+std::to_string(first)+","+std::to_string(amount)));
    unsigned line=0;auto expect=[&](const std::string& text){CHECK(line<usb.lines.size());CHECK(usb.lines[line++]==text);};
    for(unsigned index=first;index<first+amount;++index){
        const auto& frame=benchResults.legacy.frames[index];
        for(unsigned stage=0;stage<BenchProfile::OledStageCount;++stage){
            expect(Fields("CBOLED P ",{benchJob.id,index,stage,frame.oledResident[stage]}));
            expect(Fields("CBOLED W ",{benchJob.id,index,stage,frame.oledElapsed[stage]}));
            expect(Fields("CBOLED Q ",{benchJob.id,index,stage,frame.displayResident[stage]}));
            expect(Fields("CBOLED V ",{benchJob.id,index,stage,frame.displayElapsed[stage]}));
        }
        expect(Fields("CBOLED E ",{benchJob.id,index,frame.flags}));
    }
    CHECK(line==usb.lines.size());for(const auto& text:usb.lines)CHECK(text.size()<62);
}

static void CheckOledStatus(){
    const auto s=BenchProfile::OledSnapshot();
    CHECK(Command("#CBENCH OLED"));CHECK(usb.lines.size()==18);
    CHECK(usb.lines[0]=="CBOLED v=3 id="+std::to_string(benchJob.id)+" enabled="+
        std::to_string(unsigned(benchJob.oledProfile))+" skip="+std::to_string(unsigned(benchJob.oledMode==1))+
        " mode="+std::to_string(unsigned(benchJob.oledMode)));
    CHECK(usb.lines[1]=="CBOLED caps samples=128 stages=5 lanes=2");
    CHECK(usb.lines[2]==std::string("CBOLED rate period_ticks=")+
        (benchJob.oledProfile&&benchJob.oledMode>=2?"100":"0")+" tick_hz=1000");
    CHECK(usb.lines[3]==Fields("CBOLED F ",{benchJob.id,s.begun,s.ended,s.skipped}));
    CHECK(usb.lines[4]==Fields("CBOLED L ",{benchJob.id,s.lateFrames,s.maxLateTicks}));
    CHECK(usb.lines[5]==Fields("CBOLED M ",{benchJob.id,s.overrunFrames,s.maxFrameCycles}));
    CHECK(usb.lines[6]==Fields("CBOLED R ",{benchJob.id,s.renderBegun,s.rateLimited}));
    CHECK(usb.lines[7]==Fields("CBOLED G ",{benchJob.id,s.minRenderGapTicks,s.maxRenderGapTicks}));
    const auto d=BenchProfile::OledDisplaySnapshot();const auto r=OledDiagnostic::Snapshot();
    CHECK(usb.lines[8]==Fields("CBOLED D ",{benchJob.id,d.begun,d.ended,d.skipped}));
    CHECK(usb.lines[9]==Fields("CBOLED J ",{benchJob.id,d.lateFrames,d.maxLateTicks}));
    CHECK(usb.lines[10]==Fields("CBOLED N ",{benchJob.id,d.overrunFrames,d.maxFrameCycles}));
    CHECK(usb.lines[11]==Fields("CBOLED U ",{benchJob.id,d.renderBegun,d.rateLimited}));
    CHECK(usb.lines[12]==Fields("CBOLED H ",{benchJob.id,d.minRenderGapTicks,d.maxRenderGapTicks}));
    CHECK(usb.lines[13]==Fields("CBOLED S ",{benchJob.id,s.maxLoopGapTicks,d.maxLoopGapTicks}));
    CHECK(usb.lines[14]==Fields("CBOLED A ",{benchJob.id,r.maxSnapshotAgeTicks,r.overwritten,r.maxSendGapTicks}));
    CHECK(usb.lines[15]==Fields("CBOLED C ",{benchJob.id,r.flags,r.restored,r.priority}));
    CHECK(usb.lines[16]==Fields("CBOLED K ",{benchJob.id,r.displayStackFree,2048}));
    CHECK(usb.lines[17]=="CBOLED end id="+std::to_string(benchJob.id));
}

static void TestSyntax(){
    Reset();CHECK(Command("#CBENCH"));CHECK(Has(usb,"phase=IDLE"));
    CHECK(Command("#CBENCH RUN 2,0,1,1,20000",uart));CHECK(Has(uart,"usb_only"));CHECK(!BenchActive());
    const char* invalid[]={"#CBENCHX","#CBENCH RUN", "#CBENCH RUN 2,0,1,1", "#CBENCH RUN 2,0,1,1,20000,2",
        "#CBENCH RUN 2,0,1,1,20000,-1", "#CBENCH RUN 2,0,1,1,20000,0.5",
        "#CBENCH RUN 2,0,1,1,20000,0,0", "#CBENCH RUN 2,0,1,1,20000,1,5",
        "#CBENCH RUN 2,0,1,1,20000,1,-1", "#CBENCH RUN 2,0,1,1,20000,1,0.5",
        "#CBENCH RUN 2,0,1,1,20000,0,2", "#CBENCH RUN 2,0,6,1,20000,1,2",
        "#CBENCH RUN 2,0,1,1,20000,1,0,0",
        "#CBENCH RUN 2,0,6,1,20000,1",
        "#CBENCH RUN 2 0 1 1 20000", "#CBENCH RUN 2,0,1,1,20000x", "#CBENCH RUN -1,0,1,1,20000",
        "#CBENCH RUN 6,0,1,1,20000", "#CBENCH RUN 0,3,1,1,20000", "#CBENCH RUN 0,0,0,1,20000",
        "#CBENCH RUN 0,0,193,1,20000", "#CBENCH RUN 0,0,1,0,20000", "#CBENCH RUN 0,0,1,129,20000",
        "#CBENCH RUN 0,0,1,1,99", "#CBENCH RUN 0,0,1,1,20001", "#CBENCH RUN 0,0,1.5,1,20000",
        "#CBENCH RUN nan,0,1,1,20000", "#CBENCH RUN inf,0,1,1,20000", "#CBENCH RUN 1e99,0,1,1,20000",
        "#CBENCH READ 0,0", "#CBENCH READ 0,9", "#CBENCH READ 128,1", "#CBENCH READ 0.5,1"};
    for(const char* s:invalid){CHECK(Command(s));CHECK(Has(usb,"error CBENCH"));CHECK(!BenchActive());CHECK(benchJob.id==0);}
    std::string nul="#CBENCH RUN 2,0,1,1,20000";nul.push_back('\0');nul+="evil";
    CHECK(Command(nul));CHECK(Has(usb,"run_syntax"));CHECK(!BenchActive());
    CHECK(Command("#CBENCH READ 0,1"));CHECK(Has(usb,"read_range_or_busy"));
    CHECK(Command(" \t#CBENCH RUN 2,0,1,1,20000\r\n"));CHECK(BenchActive());Stop();
    Reset();Start("0,0,192,1,20000,0");CHECK(!benchJob.profile);Stop();
    Reset();Start("4,0,5,1,20000,1");CHECK(benchJob.profile);Stop();
    Reset();Start("4,0,5,1,20000,1,2");CHECK(benchJob.profile);CHECK(benchJob.oledProfile);CHECK(benchJob.oledMode==2);Stop();
}

static void TestGates(){
    using Set=std::function<void()>;
    std::vector<Set> blocked={[]{dummy.stopState=DummyRobot::NORMAL;},[]{dummy.stopState=DummyRobot::STARTING;},
        []{dummy.stopState=DummyRobot::DISABLING;},[]{dummy.isEnabled=true;},[]{dummy.startPending=true;},
        []{dummy.locked=true;},[]{dummy.motionActive=true;},[]{streamActive=true;},[]{plannerLease=true;},
        []{dummy.commandHandler.busy=true;},[]{dummy.calibration=true;},[]{dummy.parameter=true;},
        []{dummy.rgbSaveBusy=true;},[]{benchCalibrationEntry=true;},[]{nativeOwned=true;},[]{dummy.tuning=true;}};
    for(const auto& set:blocked)for(const char* args:{"2,0,1,1,20000","2,0,1,1,20000,1,2"}){
        Reset();set();Command(std::string("#CBENCH RUN ")+args);CHECK(Has(usb,"requires_idle_disabled"));CHECK(!BenchActive());
    }
    Reset();Start();const uint32_t id=benchJob.id;
    Command("#CBENCH RUN 2,0,1,1,20000");CHECK(Has(usb,"requires_idle_disabled"));CHECK(benchJob.id==id);
    const char* denied[]={"!START","!CALIBRATION","!HOME","!HAND_C 1","#TOOL_RESET","!RGB_SAVE","#IKCHECK 0,0,0,0,0,0","#CMDMODE 2",">0,0,0,0,0,0","@0,0,0,0,0,0"};
    for(const char* s:denied){CHECK(Command(s));CHECK(Has(usb,"error CBENCH busy"));}
    const char* query[]={"#STACK","#MOTION_STATUS","#GET_CMDMODE","#FW_VERSION 0","#KIN_CONFIG","#TOOL","#TCPPOSE","#CARTESIAN_STATUS"};
    for(const char* s:query){CHECK(!Command(s));CHECK(!Has(usb,"error CBENCH"));}
    CHECK(!dummy.RequestStart());CHECK(dummy.stopState==DummyRobot::DISABLED);CHECK(rgb.endBoot==0);
    CHECK(!CartesianBenchmarkEnterCalibration());Stop();
    CHECK(CartesianBenchmarkEnterCalibration());CHECK(!CartesianBenchmarkEnterCalibration());
    Command("#CBENCH RUN 2,0,1,1,20000");CHECK(Has(usb,"requires_idle_disabled"));
    CartesianBenchmarkLeaveCalibration();Start();Stop();
}

static void TestRunReadAndIds(){
    Reset();Start();const uint32_t id=benchJob.id;
    CHECK(id==1);Command("#CBENCH READ 0,1");CHECK(Has(usb,"read_range_or_busy"));
    currentThread=reinterpret_cast<void*>(2);CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::PENDING);CHECK(benchJob.count==0);
    currentThread=ControlLoopUpdateHandle;DWT->CYCCNT.value=0xffff0000U;
    CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::RUNNING);CHECK(benchJob.count==1);
    CHECK(benchJob.probeTicks>=20);CHECK(benchJob.probeCycles>3000000);CHECK(DWT->CYCCNT.writes==0);
    CHECK(benchResults.legacy.data[0].work==1);CHECK(benchResults.legacy.data[0].ik==1);CHECK(benchResults.legacy.data[0].fk==1);
    Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(benchJob.count==3);CHECK(fakeTicks>=60);
    CHECK(DWT->CYCCNT.writes==0);Command("#CBENCH READ 0,3");CHECK(Has(usb,"CBENCH DATA 1,2,"));CHECK(Has(usb,"CBENCH RESULT 1,2,"));
    CHECK(usb.lines.size()==6);Command("#CBENCH READ 2,2");CHECK(Has(usb,"read_range_or_busy"));
    Start("0,0,1,128,20000");CHECK(benchJob.id==id+1);Finish();CHECK(benchJob.count==128);
    Command("#CBENCH READ 120,8");CHECK(usb.lines.size()==16);CHECK(Has(usb,"CBENCH DATA 2,127,"));
    Command("#CBENCH CANCEL");CHECK(benchJob.phase==BenchPhase::DONE);
    benchJob.id=0xffffffffU;Start("0,0,1,1,20000");CHECK(benchJob.id==1);Stop();
}

static void TestCancelAbortBudget(){
    Reset();Start();Command("#CBENCH CANCEL");CHECK(benchJob.cancel);CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchJob.count==0);CHECK(DWT->CYCCNT.value==0);
    Reset();Start();CartesianBenchmarkPoll();Command("#CBENCH CANCEL");CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchResults.legacy.data[1].work==0);
    for(const char* s:{"!STOP","!DISABLE"}){Reset();Start();CHECK(!Command(s));CHECK(benchJob.cancel);CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CANCELLED);}
    std::vector<std::function<void()>> changes={[]{++dummy.motionEpoch;},[]{dummy.isEnabled=true;},[]{dummy.locked=true;},
        []{dummy.motionActive=true;},[]{streamActive=true;},[]{nativeOwned=true;},[]{dummy.commandHandler.busy=true;},
        []{dummy.calibration=true;},[]{dummy.parameter=true;},[]{dummy.rgbSaveBusy=true;},
        []{DWT->CTRL=0;},[]{CoreDebug->DEMCR=0;},[]{SystemCoreClock=84000000;},[]{fakeTicks+=10001;}};
    for(const auto& change:changes){Reset();Start();CartesianBenchmarkPoll();change();CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::ABORTED);CHECK(benchResults.legacy.data[1].work==0);}
    Reset();Start();clockFrozen=true;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);CHECK(benchJob.count==0);
    Reset();Start();SystemCoreClock=999999;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);
    Reset();Start("2,0,1,3,100");CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::RUNNING);
    cycleReadStep=20000;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::BUDGET);CHECK(benchResults.legacy.data[1].flag==CartBench::BUDGET);
    Reset();Start("2,0,10,3,20000");CartesianBenchmarkPoll();
    injectOnRead=cycleReads+3;cycleReadHook=[]{benchJob.cancel=true;};CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchResults.legacy.data[1].work>=1);CHECK(benchResults.legacy.data[1].work<10);
    Reset();Start("2,0,10,3,20000");CartesianBenchmarkPoll();
    injectOnRead=cycleReads+3;cycleReadHook=[]{++dummy.motionEpoch;};CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::ABORTED);CHECK(benchResults.legacy.data[1].work>=1);CHECK(benchResults.legacy.data[1].work<10);
}

static void TestProfileProtocolAndOffPath(){
    Reset();CHECK(sizeof(BenchFrame)==160);CHECK(sizeof(benchResults.legacy.frames)==128*160);
    CHECK(Command("#CBENCH PROFILE"));CHECK(usb.lines.size()==4);
    CHECK(usb.lines[0]=="CBPROF v=2 id=0 mode=0 n=0");CHECK(usb.lines[2]=="CBPROF mapmask=0 irq_included=1");
    CHECK(usb.lines[3]=="CBPROF end id=0");
    CHECK(Command("#CBENCH PROFILE",uart));CHECK(Has(uart,"usb_only"));
    for(const char* text:{"#CBENCH PREAD 0,0","#CBENCH PREAD 0,5","#CBENCH PREAD 128,1",
        "#CBENCH PREAD -1,1","#CBENCH PREAD 0.5,1","#CBENCH PREAD 0,1,1"}){
        CHECK(Command(text));CHECK(Has(usb,"profile_read_syntax"));
    }
    CHECK(Command("#CBENCH PREAD 0,1"));CHECK(Has(usb,"profile_read_range_or_busy"));
    Start("3,0,5,6,20000,1");const uint32_t id=benchJob.id;
    Command("#CBENCH");CHECK(usb.lines.size()==8);CHECK(Has(usb,"CBENCH v=1"));
    Command("#CBENCH PROFILE");CHECK(usb.lines[0]=="CBPROF v=2 id=1 mode=1 n=0");
    CHECK(usb.lines[3]=="CBPROF end id=1");
    Command("#CBENCH PREAD 0,1");CHECK(Has(usb,"profile_read_range_or_busy"));
    CartesianBenchmarkPoll();CHECK(benchJob.count==1);CHECK(benchJob.phase==BenchPhase::RUNNING);
    Command("#CBENCH PROFILE");CHECK(usb.lines.size()==4);CHECK(Has(usb,"id=1 mode=1 n=1"));
    CHECK(Has(usb,"mapmask=63 irq_included=1"));
    Command("#CBENCH PREAD 0,1");CHECK(Has(usb,"profile_read_range_or_busy"));
    Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(benchJob.id==id);
    for(unsigned i=0;i<benchJob.count;++i)CheckFrame(i);
    CheckProfileWire(0,4);CHECK(usb.lines.size()==4*16);CheckProfileWire(4,2);
    Command("#CBENCH PREAD 5,2");CHECK(Has(usb,"profile_read_range_or_busy"));
    Command("#CBENCH READ 0,1");CHECK(usb.lines.size()==2);CHECK(Has(usb,"CBENCH RESULT 1,0,"));
    // Largest wire values must remain within the existing USB short-line limit.
    benchJob.id=0xffffffffU;auto& maximum=benchResults.legacy.frames[5];
    maximum.plannedRtos=maximum.startRtos=maximum.endRtos=maximum.startHal=maximum.endHal=0xffffffffU;
    maximum.startDwt=maximum.endDwt=maximum.flags=0xffffffffU;
    for(auto& value:maximum.workCycles)value=0xffffffffU;
    for(auto& value:maximum.resident)value=0xffffffffU;
    CheckProfileWire(5,1);
    Reset();Start("0,0,1,128,20000,1");Finish();CHECK(benchJob.count==128);
    CheckProfileWire(124,4);CHECK(Has(usb,"CBPROF E 1,127,1,0"));
    // The legacy and explicit profile=0 paths leave both trace state and fixed
    // profile storage untouched; injecting an independent open backend window
    // detects an accidental Reset/Begin/End on those paths.
    for(const char* args:{"2,0,1,2,20000","2,0,1,2,20000,0"}){
        Reset();BenchProfile::SetTasks(ControlLoopUpdateHandle,controlLoopFixUpdateHandle,oledTaskHandle,rgbTaskHandle,usbServerTaskHandle,oledDisplayTaskHandle);
        CHECK(BenchProfile::Begin(0,reinterpret_cast<void*>(99)));
        for(auto& frame:benchResults.legacy.frames)frame.flags=0x1234;
        Start(args);Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(!benchJob.profile);
        CHECK(benchProfileWindowActive==1);for(const auto& frame:benchResults.legacy.frames)CHECK(frame.flags==0x1234);
        CHECK(BenchProfile::End(DWT->CYCCNT.value).flags==0);
        Command("#CBENCH PROFILE");CHECK(Has(usb,"mode=0 n=2"));
        Command("#CBENCH PREAD 0,1");CHECK(Has(usb,"profile_read_range_or_busy"));
    }
}

static void TestProfileClocksAndTaskResidency(){
    Reset();fakeTicks=1000;fakeRtosTicks=0xfffffff0U;
    Start("3,0,5,2,20000,1");unsigned activeReads=0;bool injected=false;
    hookEveryRead=true;
    cycleReadHook=[&]{
        if(!benchProfileWindowActive||++activeReads!=2)return;
        injected=true;
        void* task[]={controlLoopFixUpdateHandle,oledTaskHandle,rgbTaskHandle,usbServerTaskHandle,oledDisplayTaskHandle,reinterpret_cast<void*>(99)};
        for(unsigned i=0;i<6;++i){BenchProfile::SwitchAt(DWT->CYCCNT.value,task[i]);AdvanceCycles((i+1)*1000);}
        BenchProfile::SwitchAt(DWT->CYCCNT.value,ControlLoopUpdateHandle);
    };
    CartesianBenchmarkPoll();hookEveryRead=false;cycleReadHook={};CHECK(injected);
    CHECK(benchJob.count==1);CheckFrame(0);
    const auto first=benchResults.legacy.frames[0];CHECK(first.plannedRtos==4);CHECK(first.startRtos==4);
    CHECK(first.startHal==1020);CHECK(first.startHal!=first.startRtos);
    for(unsigned i=1;i<7;++i)CHECK(first.resident[i]==i*1000);
    CHECK(first.resident[0]>0);CHECK(first.workCycles[0]>=21000);
    AdvanceCycles(uint32_t(uint64_t(SystemCoreClock)*25/1000));
    DWT->CYCCNT.value=0xffffff00U;clockRemainder=0;
    CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::DONE);CheckFrame(1);
    const auto& late=benchResults.legacy.frames[1];CHECK(late.plannedRtos==24);CHECK(late.startRtos==29);
    CHECK(uint32_t(late.startRtos-late.plannedRtos)==5);CHECK(late.startHal==1045);
    CHECK(late.endDwt<late.startDwt);CHECK(late.resident[0]==uint32_t(late.endDwt-late.startDwt));
    for(unsigned i=1;i<7;++i)CHECK(late.resident[i]==0);
    CheckProfileWire(0,2);
}

static void TestProfileTerminalBoundaries(){
    for(bool cancellation:{true,false}){
        Reset();Start("3,0,5,3,20000,1");CartesianBenchmarkPoll();
        injectOnRead=cycleReads+3;cycleReadHook=[&]{if(cancellation)benchJob.cancel=true;else ++dummy.motionEpoch;};
        CartesianBenchmarkPoll();CHECK(benchJob.phase==(cancellation?BenchPhase::CANCELLED:BenchPhase::ABORTED));
        CHECK(benchJob.count==2);CHECK(benchResults.legacy.data[1].work==1);CheckFrame(1);
        CHECK(benchResults.legacy.frames[1].workCycles[0]==benchResults.legacy.data[1].cycles);CheckProfileWire(1,1);
        CHECK(Has(usb,"CBPROF W 1,1,0,"));CHECK(!Has(usb,"CBPROF W 1,1,1,"));
    }
    Reset();Start("3,0,5,3,100,1");CartesianBenchmarkPoll();cycleReadStep=1800;
    CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::BUDGET);CHECK(benchResults.legacy.data[1].work==5);
    CHECK(benchResults.legacy.frames[1].workCycles[4]==benchResults.legacy.data[1].cycles);CheckFrame(1);CheckProfileWire(1,1);
    CHECK(Has(usb,"CBPROF W 1,1,4,"));CHECK(Has(usb,"CBPROF E 1,1,5,0"));
    Reset();Start("3,0,5,3,100,1");CartesianBenchmarkPoll();cycleReadStep=20000;
    CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::BUDGET);CHECK(benchResults.legacy.data[1].work==0);
    CheckFrame(1);CheckProfileWire(1,1);CHECK(!Has(usb,"CBPROF W "));CHECK(usb.lines.size()==11);
    Reset();Start("3,0,5,3,20000,1");CartesianBenchmarkPoll();Command("#CBENCH CANCEL");CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchResults.legacy.data[1].work==0);
    CheckFrame(1);CheckProfileWire(1,1);CHECK(!Has(usb,"CBPROF W "));
}

static void TestOledProtocolAndAccounting(){
    Reset();CheckOledStatus();
    CHECK(Command("#CBENCH OLED",uart));CHECK(Has(uart,"usb_only"));
    CHECK(Command("#CBENCH OREAD 0,1",uart));CHECK(Has(uart,"usb_only"));
    for(const char* text:{"#CBENCH OREAD 0,0","#CBENCH OREAD 0,5","#CBENCH OREAD 128,1",
        "#CBENCH OREAD -1,1","#CBENCH OREAD 0.5,1","#CBENCH OREAD 0,1,1","#CBENCH OREAD nan,1"}){
        CHECK(Command(text));CHECK(Has(usb,"oled_read_syntax"));CHECK(!BenchActive());
    }
    CHECK(Command("#CBENCH OREAD 0,1"));CHECK(Has(usb,"oled_read_range_or_busy"));
    for(unsigned mode:{0U,1U,2U}){
        const bool skip=mode==1;
        Reset();const std::string args="3,0,5,6,20000,1,"+std::to_string(mode);Start(args.c_str());
        CHECK(benchJob.oledProfile);CHECK(benchJob.oledMode==mode);CHECK(!benchOledSessionActive);
        CheckOledStatus();
        Command("#CBENCH OREAD 0,1");CHECK(Has(usb,"oled_read_range_or_busy"));
        bool injected=false;hookEveryRead=true;
        cycleReadHook=[&]{
            if(injected||!benchProfileWindowActive)return;injected=true;
            BenchProfile::SwitchAt(DWT->CYCCNT.value,oledTaskHandle);
            const auto token=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,fakeRtosTicks,fakeRtosTicks-3);
            CHECK(token.skip==skip);CHECK(token.generation!=0);AdvanceCycles(31);
            BenchProfile::OledStageAt(token,BenchProfile::OLED_SERVICE,DWT->CYCCNT.value);AdvanceCycles(41);
            if(!token.skip){
                BenchProfile::OledStageAt(token,BenchProfile::OLED_DRAW,DWT->CYCCNT.value);AdvanceCycles(53);
                BenchProfile::OledStageAt(token,BenchProfile::OLED_SEND,DWT->CYCCNT.value);AdvanceCycles(67);
            }
            BenchProfile::OledFrameEnd(token,DWT->CYCCNT.value);
            BenchProfile::SwitchAt(DWT->CYCCNT.value,ControlLoopUpdateHandle);
        };
        CartesianBenchmarkPoll();cycleReadHook={};hookEveryRead=false;
        CHECK(injected);CHECK(benchOledSessionActive);CHECK(benchJob.phase==BenchPhase::RUNNING);CheckOledFrame(0);
        CHECK(benchResults.legacy.frames[0].oledResident[BenchProfile::OLED_IMU]==31);
        CHECK(benchResults.legacy.frames[0].oledResident[BenchProfile::OLED_SERVICE]==41);
        CHECK(benchResults.legacy.frames[0].oledResident[BenchProfile::OLED_DRAW]==(skip?0U:53U));
        CHECK(benchResults.legacy.frames[0].oledResident[BenchProfile::OLED_SEND]==(skip?0U:67U));
        const auto active=BenchProfile::OledSnapshot();CHECK(active.begun==1);CHECK(active.ended==1);CHECK(active.skipped==unsigned(skip));
        CHECK(active.lateFrames==1);CHECK(active.maxLateTicks==3);CHECK(active.overrunFrames==0);CHECK(active.maxFrameCycles==(skip?72U:192U));
        CHECK(active.renderBegun==unsigned(!skip));CHECK(active.rateLimited==0);
        CHECK(active.minRenderGapTicks==0);CHECK(active.maxRenderGapTicks==0);
        Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(!benchOledSessionActive);
        for(unsigned i=0;i<benchJob.count;++i)CheckOledFrame(i);
        CheckOledWire(0,4);CHECK(usb.lines.size()==84);CheckOledWire(4,2);
        Command("#CBENCH OREAD 5,2");CHECK(Has(usb,"oled_read_range_or_busy"));
        CheckOledStatus();CHECK(usb.lines[3]==Fields("CBOLED F ",{1,1,1,unsigned(skip)}));
        CHECK(usb.lines[4]=="CBOLED L 1,1,3");CHECK(usb.lines[5]==Fields("CBOLED M ",{1,0,skip?72U:192U}));
        CheckProfileWire(0,4); // The existing v1 profile wire is unchanged.
        // Largest legal wire values still fit the USB's established short line.
        benchJob.id=0xffffffffU;auto& maximum=benchResults.legacy.frames[5];maximum.flags=0xffffffffU;
        for(auto& value:maximum.oledResident)value=0xffffffffU;
        for(auto& value:maximum.oledElapsed)value=0xffffffffU;
        for(auto& value:maximum.displayResident)value=0xffffffffU;
        for(auto& value:maximum.displayElapsed)value=0xffffffffU;
        CheckOledWire(5,1);CheckOledStatus();
    }
    Reset();Start("0,0,1,128,20000,1,0");Finish();CHECK(benchJob.count==128);CheckOledWire(124,4);
    CHECK(Has(usb,"CBOLED E 1,127,0"));CHECK(!benchOledSessionActive);
    // Maximum u32 fields in the widest metadata records fit the existing
    // Respond payload contract without truncation or changing old wire lines.
    CHECK(Fields("CBOLED F ",{0xffffffffU,0xffffffffU,0xffffffffU,0xffffffffU}).size()<62);
    CHECK(Fields("CBOLED G ",{0xffffffffU,0xffffffffU,0xffffffffU}).size()<62);
}

static void TestOledRateAdmission(){
    Reset();Start("3,0,5,6,20000,1,2");CartesianBenchmarkPoll();CHECK(benchOledSessionActive);
    // Drive actual frame-admission ticks independently of the sample schedule.
    // A late opportunity admits only one current frame, without replaying the
    // three missed 100 ms deadlines. The following opportunity is still held.
    for(unsigned tick:{20U,40U,120U,140U,455U,475U}){
        const auto token=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,tick,tick);
        CHECK(token.generation!=0);CHECK(token.skip==(tick==40||tick==140||tick==475));
        AdvanceCycles(71);BenchProfile::OledFrameEnd(token,DWT->CYCCNT.value);
    }
    const auto s=BenchProfile::OledSnapshot();CHECK(s.begun==6);CHECK(s.ended==6);CHECK(s.skipped==3);
    CHECK(s.renderBegun==3);CHECK(s.rateLimited==3);CHECK(s.minRenderGapTicks==100);CHECK(s.maxRenderGapTicks==335);
    CheckOledStatus();CHECK(usb.lines[6]=="CBOLED R 1,3,3");CHECK(usb.lines[7]=="CBOLED G 1,100,335");
    Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(!benchOledSessionActive);CheckOledStatus();
    CHECK(!BenchProfile::OledFrameBegin(DWT->CYCCNT.value,476,476).skip);
    // A new mode=2 job clears previous admission history and admits its first
    // opportunity even when the RTOS counter has wrapped since the last job.
    Start("3,0,5,2,20000,1,2");CartesianBenchmarkPoll();
    const auto first=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,0xfffffff0U,0xfffffff0U);CHECK(!first.skip);
    BenchProfile::OledFrameEnd(first,DWT->CYCCNT.value);
    const auto early=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,4,4);CHECK(early.skip);BenchProfile::OledFrameEnd(early,DWT->CYCCNT.value);
    const auto due=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,84,84);CHECK(!due.skip);BenchProfile::OledFrameEnd(due,DWT->CYCCNT.value);
    CHECK(BenchProfile::OledSnapshot().minRenderGapTicks==100);Stop();CHECK(!benchOledSessionActive);
}

static void TestOledLifecycleAndLegacy(){
    // Five/six fields reset old session results and invalidate a stale token,
    // but do not begin a new OLED session, including explicit profile=1.
    for(const char* args:{"2,0,1,2,20000","2,0,1,2,20000,0","2,0,1,2,20000,1"}){
        Reset();BenchProfile::OledSessionStart(1,SystemCoreClock);
        auto token=BenchProfile::OledFrameBegin(0,0,0);CHECK(token.skip);
        Start(args);CHECK(!benchOledSessionActive);CHECK(!benchJob.oledProfile);CHECK(benchJob.oledMode==0);
        BenchProfile::OledFrameEnd(token,100);CHECK(BenchProfile::OledSnapshot().ended==0);
        Finish();CHECK(!benchOledSessionActive);CHECK(BenchProfile::OledSnapshot().begun==0);
        CheckOledStatus();CHECK(Has(usb,"enabled=0 skip=0 mode=0"));
        Command("#CBENCH OREAD 0,1");CHECK(Has(usb,"oled_read_range_or_busy"));
    }
    for(unsigned mode:{1U,2U}){
    const std::string prefix="3,0,5,2,20000,1,"+std::to_string(mode);
    Reset();Start(prefix.c_str());Command("#CBENCH CANCEL");CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(!benchOledSessionActive);CHECK(BenchProfile::OledSnapshot().begun==0);
    for(bool frozen:{false,true}){
        Reset();Start(prefix.c_str());if(frozen)clockFrozen=true;else SystemCoreClock=999999;
        CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);CHECK(!benchOledSessionActive);CHECK(BenchProfile::OledSnapshot().begun==0);
    }
    // Session must already be active during the existing 20 ms DWT probe.
    // A failing probe and a post-probe safety failure must both release skip.
    for(bool clockFailure:{false,true}){
        Reset();Start(prefix.c_str());bool observed=false;
        delayHook=[&](uint32_t n){
            CHECK(n==20);CHECK(benchOledSessionActive);CHECK(!benchProfileWindowActive);observed=true;
            const auto token=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,fakeRtosTicks,fakeRtosTicks);
            CHECK(token.skip==(mode==1));AdvanceCycles(71);BenchProfile::OledFrameEnd(token,DWT->CYCCNT.value);
            if(clockFailure)fakeTicks+=1000;else ++dummy.motionEpoch;
        };
        CartesianBenchmarkPoll();CHECK(observed);CHECK(benchJob.phase==(clockFailure?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED));
        CHECK(!benchOledSessionActive);const auto s=BenchProfile::OledSnapshot();CHECK(s.begun==1);CHECK(s.ended==1);CHECK(s.skipped==unsigned(mode==1));
        CHECK(!BenchProfile::OledFrameBegin(DWT->CYCCNT.value,fakeRtosTicks,fakeRtosTicks).skip);
    }
    for(unsigned terminal=0;terminal<4;++terminal){
        Reset();const std::string args=std::string(terminal==3?"3,0,5,3,100,1,":"3,0,5,3,20000,1,")+std::to_string(mode);
        Start(args.c_str());CartesianBenchmarkPoll();
        CHECK(benchOledSessionActive);const auto stale=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,fakeRtosTicks,fakeRtosTicks);
        CHECK(stale.skip==(mode==1));AdvanceCycles(61);BenchProfile::OledFrameEnd(stale,DWT->CYCCNT.value);
        if(terminal==0)Command("#CBENCH CANCEL");
        if(terminal==1)++dummy.motionEpoch;
        if(terminal==2)DWT->CTRL=0;
        if(terminal==3)cycleReadStep=20000;
        CartesianBenchmarkPoll();CHECK(!BenchActive());CHECK(!benchOledSessionActive);
        CHECK(benchJob.phase==(terminal==0?BenchPhase::CANCELLED:terminal==3?BenchPhase::BUDGET:BenchPhase::ABORTED));
        const auto s=BenchProfile::OledSnapshot();CHECK(s.begun==1);CHECK(s.ended==1);CHECK(s.skipped==unsigned(mode==1));
        BenchProfile::OledStageAt(stale,BenchProfile::OLED_SEND,DWT->CYCCNT.value);BenchProfile::OledFrameEnd(stale,DWT->CYCCNT.value);
        CHECK(BenchProfile::OledSnapshot().ended==s.ended);CHECK(!BenchProfile::OledFrameBegin(DWT->CYCCNT.value,0,0).skip);
        CheckOledWire(0,2);
    }
    }
}

static void TestSplitDiagnostics(){
    for(unsigned mode:{3U,4U}){
        Reset();const std::string args="3,0,5,3,20000,1,"+std::to_string(mode);
        Start(args.c_str());CHECK(!OledDiagnostic::Busy());
        OledDiagnostic::startHook=[]{CHECK(BenchActive());CHECK(!benchOledSessionActive);};
        OledDiagnostic::stopHook=[]{CHECK(BenchActive());CHECK(!benchOledSessionActive);};
        bool injected=false;hookEveryRead=true;
        cycleReadHook=[&]{
            if(injected||!benchProfileWindowActive)return;injected=true;
            BenchProfile::SwitchAt(DWT->CYCCNT.value,oledTaskHandle);
            const auto w=BenchProfile::OledFrameBegin(DWT->CYCCNT.value,10,10);
            CHECK(w.skip&&!w.rateLimited);AdvanceCycles(31);
            BenchProfile::OledStageAt(w,BenchProfile::OLED_SERVICE,DWT->CYCCNT.value);AdvanceCycles(41);
            BenchProfile::SwitchAt(DWT->CYCCNT.value,oledDisplayTaskHandle);
            const auto d=BenchProfile::OledDisplayFrameBegin(DWT->CYCCNT.value,10,10);CHECK(!d.skip);
            BenchProfile::OledStageAt(d,BenchProfile::OLED_DRAW,DWT->CYCCNT.value);AdvanceCycles(53);
            BenchProfile::OledFrameEnd(w,DWT->CYCCNT.value);
            BenchProfile::OledStageAt(d,BenchProfile::OLED_SEND,DWT->CYCCNT.value);AdvanceCycles(67);
            BenchProfile::OledFrameEnd(d,DWT->CYCCNT.value);
            BenchProfile::SwitchAt(DWT->CYCCNT.value,ControlLoopUpdateHandle);
        };
        CartesianBenchmarkPoll();cycleReadHook={};hookEveryRead=false;
        CHECK(injected);CHECK(OledDiagnostic::Busy());CheckOledFrame(0);
        CHECK(benchResults.legacy.frames[0].oledResident[0]==31&&benchResults.legacy.frames[0].oledResident[1]==41);
        CHECK(benchResults.legacy.frames[0].displayResident[2]==53&&benchResults.legacy.frames[0].displayResident[3]==67);
        CHECK(BenchProfile::OledSnapshot().renderBegun==0&&BenchProfile::OledSnapshot().skipped==1);
        CHECK(BenchProfile::OledDisplaySnapshot().renderBegun==1);
        Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(!OledDiagnostic::Busy());
        CHECK(OledDiagnostic::starts==1&&OledDiagnostic::stops==1);
        CHECK(OledDiagnostic::Snapshot().priority==(mode==4?16U:24U));
        CHECK(OledDiagnostic::Snapshot().restored==1);CheckOledStatus();CheckOledWire(0,3);CheckProfileWire(0,3);
        // All terminal reasons close counters before the potentially yielding
        // ownership hand-back; errors cannot publish DONE before restoration.
        for(unsigned terminal=0;terminal<5;++terminal){
            Reset();Start(args.c_str());
            OledDiagnostic::stopHook=[]{CHECK(!benchOledSessionActive);CHECK(BenchActive());};
            if(terminal==4){clockFrozen=true;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);}
            else {
                CartesianBenchmarkPoll();CHECK(OledDiagnostic::Busy());
                if(terminal==0)Command("#CBENCH CANCEL");
                if(terminal==1)++dummy.motionEpoch;
                if(terminal==2){cycleReadStep=20000;benchJob.config.budgetUs=100;}
                if(terminal==3){benchJob.config.samples=2;}
                CartesianBenchmarkPoll();
                const BenchPhase expected=terminal==0?BenchPhase::CANCELLED:terminal==1?BenchPhase::ABORTED:terminal==2?BenchPhase::BUDGET:BenchPhase::DONE;
                CHECK(benchJob.phase==expected);
            }
            CHECK(!OledDiagnostic::Busy());CHECK(!benchOledSessionActive);CHECK(OledDiagnostic::stops==1);
        }
        Reset();Start(args.c_str());OledDiagnostic::failStart=true;
        CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::ABORTED);CHECK(benchJob.count==0);
        CHECK(!OledDiagnostic::Busy());CHECK(!benchOledSessionActive);
        // A failed hand-back keeps admission closed even after ABORTED. Poll
        // yields and retries; it must not spin or admit a new RUN/START/calibrate.
        Reset();Start(args.c_str());CartesianBenchmarkPoll();
        OledDiagnostic::failStop=true;Command("#CBENCH CANCEL");CartesianBenchmarkPoll();
        CHECK(benchJob.phase==BenchPhase::ABORTED);CHECK(CartesianBenchmarkActive());CHECK(!benchOledSessionActive);
        CHECK(!dummy.RequestStart());CHECK(!CartesianBenchmarkEnterCalibration());
        Command("#CBENCH RUN 2,0,1,1,20000");CHECK(Has(usb,"requires_idle_disabled"));
        Command("!START");CHECK(Has(usb,"error CBENCH busy"));CheckOledStatus();
        CHECK(Has(usb,"CBOLED C 1,0,0,"));const auto before=fakeRtosTicks;
        CartesianBenchmarkPoll();CHECK(fakeRtosTicks>before);CHECK(CartesianBenchmarkActive());
        OledDiagnostic::failStop=false;CartesianBenchmarkPoll();CHECK(!CartesianBenchmarkActive());
        CHECK(OledDiagnostic::Snapshot().restored==1);CHECK(CartesianBenchmarkEnterCalibration());CartesianBenchmarkLeaveCalibration();
    }
    // Largest runtime and dual-lane fields remain within Respond's 64B buffer.
    Reset();OledDiagnostic::runtime={0xffffffffU,0xffffffffU,0xffffffffU,0xffffffffU,0xffffffffU,0xffffffffU,0xffffffffU};
    benchJob.id=0xffffffffU;CheckOledStatus();
}

static void TestRealCalibrationWrapper(){
    Reset();FakeProtection protection;dummy.protection=&protection;
    Start();CHECK(dummy.HandleProtectionCommand("!CALIBRATION",12,usb));CHECK(protection.calls==0);CHECK(Has(usb,"compute_diagnostic_busy"));Stop();
    bool leaseObserved=false;
    protection.commandHook=[&]{leaseObserved=benchCalibrationEntry;Command("#CBENCH RUN 2,0,1,1,20000");CHECK(Has(usb,"requires_idle_disabled"));};
    const std::string cmd=" \t!CALIBRATION\r\n";
    CHECK(dummy.HandleProtectionCommand(cmd.data(),unsigned(cmd.size()),usb));CHECK(protection.calls==1);CHECK(leaseObserved);CHECK(!benchCalibrationEntry);CHECK(!BenchActive());
    protection.commandHook={};CHECK(dummy.RequestStart());CHECK(dummy.stopState==DummyRobot::STARTING);
}

static void TestEnduranceBackend(){
    using namespace CartEndurance;
    State s;const uint32_t hz=168000000,origin=0xfffffff0U;
    s.Reset(hz,600,origin);CHECK(sizeof(s)<=3072);
    CartBench::Sample sample{};sample.cycles=756000;sample.ticks=4;sample.work=5;sample.fk=10;sample.ik=5;
    sample.valid=40;sample.selected=5;sample.checksum=3141804433U;
    BenchProfile::Window w{};w.cycles=756200;w.resident[0]=w.cycles;
    w.oledElapsed[4]=w.displayElapsed[4]=w.cycles;
    for(unsigned i=0;i<30000;++i){
        const uint32_t ms=i*20U,dwt=origin+uint32_t(uint64_t(ms)*hz/1000U);
        CHECK(s.Observe(sample,w,ms,ms,dwt,0)==NONE);
        if(i%500==0)s.Stacks(6400,1300,1700,1000);
        s.Overhead(1000);
    }
    CHECK(s.AdvanceClock(600000,600000,origin+uint32_t(uint64_t(600000)*hz/1000U)));
    CHECK(s.n==30000&&s.elapsedMs==600000);CHECK(s.extendedDwt==uint64_t(600)*hz);
    CHECK(s.sumCycles==uint64_t(756000)*30000U);CHECK(s.sumCycles>0xffffffffU);
    CHECK(s.sumWindow==uint64_t(w.cycles)*30000U);CHECK(s.histogram[4]==30000);
    CHECK(s.trendCount==60&&s.historyCount==8);CHECK(s.maxGap==20);CHECK(s.errorSequence==0xffffffffU);
    for(unsigned i=0;i<60;++i){CHECK(s.trends[i].n==500);CHECK(s.trends[i].sumCycles==uint64_t(756000)*500);CHECK(s.trends[i].commandFree==6400);}
    for(unsigned i=0;i<8;++i)CHECK(s.HistoryAt(i).sequence==29992+i);
    CHECK(s.Observe(sample,w,600000,600000,0,0)==DEADLINE);CHECK(s.n==30000);
    for(unsigned fault=0;fault<8;++fault){
        s.Reset(hz,1,origin);CHECK(s.Observe(sample,w,0,0,origin,0)==NONE);
        auto bad=sample;auto window=w;uint32_t ms=20,hal=20,dwt=origin+3360000U,late=0;Error expected=NUMERIC;
        if(fault==0)bad.failed=1;
        if(fault==1){bad.checksum++;expected=SIGNATURE;}
        if(fault==2){window.resident[0]--;expected=ACCOUNTING;}
        if(fault==3){late=20;expected=DEADLINE;}
        if(fault==4){ms=hal=30000;dwt=origin+uint32_t(uint64_t(ms)*hz/1000U);expected=CLOCK;}
        if(fault==5){hal=40;expected=CLOCK;}
        if(fault==6){bad.flag=CartBench::BUDGET;expected=BUDGET;}
        if(fault==7){bad.flag=CartBench::CANCELLED;expected=CANCELLED;}
        CHECK(s.Observe(bad,window,ms,hal,dwt,late)==expected);CHECK(s.n==2);
        CHECK(s.errorSequence==1);CHECK(s.HistoryAt(1).sequence==1);
        CHECK(s.HistoryAt(1).elapsedMs<=s.elapsedMs);
        if(fault==4){CHECK(s.elapsedMs==30000);CHECK(s.extendedDwt==0);}
        s.Fail(RESTORE,2);CHECK(s.error==expected); // first cause retained
    }
    s.Reset(hz,1,origin);CHECK(!s.AdvanceClock(20,20,origin));CHECK(s.error==CLOCK);CHECK(s.elapsedMs==20);CHECK(s.extendedDwt==0);
    s.Reset(hz,0,origin);CHECK(s.error==SETUP&&s.n==0);
}

static void WriteLongFixture(const char* path){
    FILE* wire=std::fopen(path,"wb");CHECK(wire!=nullptr);
    auto emit=[&]{for(const auto& line:usb.lines)std::fprintf(wire,"%s\r\n",line.c_str());};
    Command("#CBENCH LONG");emit();Command("#CBENCH LREAD");emit();Command("#CBENCH OLED");emit();
    CHECK(std::fclose(wire)==0);
}

static void TestEnduranceAdapter(){
    Reset();CHECK(Command("#CBENCH LONG"));CHECK(usb.lines.size()==10);CHECK(Has(usb,"id=0 phase=IDLE"));
    CHECK(Command("#CBENCH LREAD"));CHECK(Has(usb,"long_read_range_or_busy"));
    for(const char* args:{"2,0,5,1,20000,4","3,3,5,1,20000,4","3,0,4,1,20000,4","3,0,5,0,20000,4",
        "3,0,5,601,20000,4","3,0,5,1,20001,4","3,0,5,1,10000,4","3,0,5,1,20000,3","3,0,5,1,20000,4,0"}){
        CHECK(Command(std::string("#CBENCH LONG ")+args));CHECK(Has(usb,"run_syntax"));CHECK(!BenchActive());
    }
    CHECK(Command("#CBENCH LONG 3,0,5,1,20000,4",uart));CHECK(Has(uart,"usb_only"));
    Reset();dummy.isEnabled=true;Command("#CBENCH LONG 3,0,5,1,20000,4");CHECK(Has(usb,"requires_idle_disabled"));
    Reset();benchJob.id=0xffffffffU;Command("#CBENCH LONG 4,2,5,600,20000,4");CHECK(benchJob.id==1);CHECK(benchJob.endurance);
    Command("#CBENCH LONG 3,0,5,1,20000,2");CHECK(Has(usb,"requires_idle_disabled"));
    CHECK(Command("#CBENCH LREAD"));CHECK(Has(usb,"long_read_range_or_busy"));
    DWT->CYCCNT.value=0xfffff000U;
    for(unsigned i=0;i<30000;++i){CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::RUNNING);CHECK(benchLong.n==i+1);CHECK(benchJob.count==0);}
    CHECK(benchLong.elapsedMs<600000);CHECK(BenchCheckpoint(nullptr)==CartBench::COMPLETE);
    CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(benchLong.n==30000);CHECK(benchLong.elapsedMs>=600000);
    CHECK(benchLong.extendedDwt>uint64_t(20)*0xffffffffU);CHECK(benchLong.error==CartEndurance::NONE);
    CHECK(benchLong.trendCount==60&&benchLong.historyCount==8);CHECK(benchLong.scanCount==62);
    CHECK(!OledDiagnostic::Busy()&&!benchOledSessionActive);CHECK(benchJob.count==0);
    Command("#CBENCH LONG");CHECK(usb.lines.size()==10);CHECK(Has(usb,"n=30000 elapsed_ms=600000"));
    Command("#CBENCH LREAD");CHECK(usb.lines.size()==13+21+7+20+60*3+8*3+1);CHECK(Has(usb,"CBLONG read_end id=1"));
    WriteLongFixture("long-terminal.wire");
    Command("#CBENCH READ 0,1");CHECK(Has(usb,"read_range_or_busy"));
    Start("3,0,5,36,20000,1,4");CHECK(!benchJob.endurance);Finish();CHECK(benchJob.count==36);CheckFrame(35);
    Command("#CBENCH LONG");CHECK(Has(usb,"id=0 phase=IDLE"));
    Start("3,0,5,128,20000,1,2");Finish();CHECK(benchJob.count==128);CheckFrame(127);
    for(unsigned fault=0;fault<7;++fault){
        Reset();Command("#CBENCH LONG 3,0,5,1,20000,4");
        if(fault==0){clockFrozen=true;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);CHECK(benchLong.n==0);}
        else if(fault==1){Command("#CBENCH CANCEL");CHECK(usb.lines.size()==10);CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchLong.n==0);}
        else {
            for(unsigned i=0;i<50;++i)CartesianBenchmarkPoll();CHECK(BenchActive());CHECK(benchLong.n==50);
            if(fault==2)Command("#CBENCH CANCEL");
            if(fault==3)++dummy.motionEpoch;
            if(fault==4)DWT->CTRL=0;
            if(fault==5)fakeTicks+=100;
            if(fault==6)OledDiagnostic::failStop=true;
            CartesianBenchmarkPoll();CHECK(benchJob.phase!=(BenchPhase::DONE));
            CHECK(benchLong.n==50);CHECK(benchLong.error!=CartEndurance::NONE);
        }
        Command("#CBENCH LREAD");CHECK(Has(usb,"CBLONG read_end id=1"));
        if(fault<2)CHECK(Has(usb,"CBLONG minmax 1,0,0"));
        if(fault==6){CHECK(CartesianBenchmarkActive());CHECK(!dummy.RequestStart());OledDiagnostic::failStop=false;CartesianBenchmarkPoll();}
        CHECK(!BenchActive());
    }
    // Restoring OLED ownership may yield. A cancellation, changed epoch or
    // counter failure during that wait must not publish a spurious DONE.
    for(unsigned fault=0;fault<3;++fault){
        Reset();Command("#CBENCH LONG 3,0,5,1,20000,2");
        for(unsigned i=0;i<50;++i)CartesianBenchmarkPoll();
        OledDiagnostic::stopHook=[&]{if(fault==0)benchJob.cancel=true;if(fault==1)++dummy.motionEpoch;if(fault==2)DWT->CTRL=0;};
        CartesianBenchmarkPoll();CHECK(benchJob.phase!=BenchPhase::DONE);CHECK(benchLong.error!=CartEndurance::NONE);
        OledDiagnostic::stopHook={};CHECK(!BenchActive());
    }
    for(unsigned fault=0;fault<2;++fault){
        Reset();Command("#CBENCH LONG 3,0,5,1,20000,4");
        for(unsigned i=0;i<50;++i)CartesianBenchmarkPoll();
        OledDiagnostic::stopHook=[&]{enterCriticalHook=[&]{if(fault==0)benchJob.cancel=true;else ++dummy.motionEpoch;};};
        CartesianBenchmarkPoll();CHECK(benchJob.phase==(fault==0?BenchPhase::CANCELLED:BenchPhase::ABORTED));
        CHECK(benchLong.error==(fault==0?CartEndurance::CANCELLED:CartEndurance::ENVIRONMENT));
        OledDiagnostic::stopHook={};CHECK(!enterCriticalHook);CHECK(!BenchActive());
    }
    for(unsigned fault=0;fault<3;++fault){
        Reset();Command("#CBENCH LONG 3,0,5,1,20000,4");CartesianBenchmarkPoll();
        if(fault==0){fakeTicks+=40;fakeRtosTicks+=40;DWT->CYCCNT.value+=6720000U;}
        if(fault==1){fakeTicks+=30000;fakeRtosTicks+=30000;DWT->CYCCNT.value+=uint32_t(uint64_t(30000)*168000);}
        if(fault==2)cycleReadStep=500000;
        CartesianBenchmarkPoll();CHECK(!BenchActive());
        CHECK(benchLong.error==(fault==0?CartEndurance::DEADLINE:fault==1?CartEndurance::CLOCK:CartEndurance::BUDGET));
        if(fault==0){CHECK(benchLong.n==2);CHECK(benchLong.HistoryAt(1).work==0);}
        if(fault==1){CHECK(benchLong.elapsedMs>=30000);WriteLongFixture("long-clock-error.wire");}
    }
}

static void FullStart(const char* args="0,0,1,16,20000"){
    CHECK(Command(std::string("#CBENCH FULL_RUN ")+args));
    CHECK(benchJob.phase==BenchPhase::PENDING);CHECK(benchJob.full);CHECK(benchJob.oledMode==4);
}

static void CheckFullWire(unsigned first,unsigned amount){
    CHECK(Command("#CBENCH FREAD "+std::to_string(first)+","+std::to_string(amount)));
    CHECK(Has(usb,"CFULL read id="));CHECK(Has(usb,"CFULL read_end id="));
    unsigned ends=0,q=0,v=0;
    for(const auto& row:usb.lines){
        CHECK(row.size()<62);
        if(row.find("CFULL E ")==0)++ends;
        if(row.find("CFULL Q ")==0)++q;
        if(row.find("CFULL V ")==0)++v;
    }
    CHECK(ends==amount);CHECK(q==amount*7);CHECK(v==amount*7);
}

static void WriteFullFixture(const char* path){
    FILE* wire=std::fopen(path,"wb");CHECK(wire!=nullptr);
    auto emit=[&]{for(const auto& line:usb.lines)std::fprintf(wire,"%s\r\n",line.c_str());};
    Command("#CBENCH FULL");emit();
    for(unsigned first=0;first<benchJob.count;first+=4){
        CheckFullWire(first,std::min(4U,benchJob.count-first));emit();
    }
    Command("#CBENCH OLED");emit();CHECK(std::fclose(wire)==0);
}

static void TestFullSyntaxAndGates(){
    Reset();CHECK(sizeof(CartCycle::Frame)==148);CHECK(sizeof(FullResults)<=sizeof(LegacyResults));
    CHECK(Command("#CBENCH FULL"));CHECK(Has(usb,"CFULL"));CHECK(Has(usb,"phase=IDLE"));
    CHECK(Command("#CBENCH FULL_RUN 0,0,1,16,20000",uart));CHECK(Has(uart,"usb_only"));CHECK(!BenchActive());
    for(const char* args:{"", "0,0,1,16", "0,0,1,16,20000,0", "13,0,1,16,20000", "0,6,1,16,20000",
        "0,0,0,16,20000", "0,0,4,16,20000", "0,0,1,0,20000", "0,0,1,161,20000",
        "0,0,1,16,19999", "0,0,1,16,20001", "0,0,1,16.5,20000", "nan,0,1,16,20000",
        "0,-1,1,16,20000", "0,0,1,16,20000x"}){
        CHECK(Command(std::string("#CBENCH FULL_RUN ")+args));CHECK(Has(usb,"error CBENCH"));CHECK(!BenchActive());CHECK(benchJob.id==0);
    }
    for(const char* read:{"#CBENCH FREAD 0,0", "#CBENCH FREAD 0,5", "#CBENCH FREAD 160,1",
        "#CBENCH FREAD -1,1", "#CBENCH FREAD 0.5,1", "#CBENCH FREAD 0,1,0"}){
        CHECK(Command(read));CHECK(Has(usb,"error CBENCH"));
    }
    CHECK(Command("#CBENCH FREAD 0,1"));CHECK(Has(usb,"error CBENCH"));
    std::vector<std::function<void()>> blocked={[]{dummy.stopState=DummyRobot::NORMAL;},[]{dummy.stopState=DummyRobot::STARTING;},
        []{dummy.isEnabled=true;},[]{dummy.startPending=true;},[]{dummy.locked=true;},[]{dummy.motionActive=true;},
        []{streamActive=true;},[]{nativeOwned=true;},[]{plannerLease=true;},[]{dummy.commandHandler.busy=true;},
        []{dummy.calibration=true;},[]{dummy.parameter=true;},[]{dummy.rgbSaveBusy=true;},[]{benchCalibrationEntry=true;},[]{dummy.tuning=true;}};
    for(const auto& set:blocked){Reset();set();Command("#CBENCH FULL_RUN 0,0,1,16,20000");CHECK(Has(usb,"requires_idle_disabled"));CHECK(!BenchActive());}
    Reset();FullStart();const uint32_t id=benchJob.id;
    CHECK(!dummy.RequestStart());CHECK(!CartesianBenchmarkEnterCalibration());
    CHECK(Command("!START"));CHECK(Has(usb,"busy"));
    CHECK(Command("#CBENCH FULL_RUN 0,0,1,16,20000"));CHECK(Has(usb,"requires_idle_disabled"));CHECK(benchJob.id==id);
    CHECK(Command("#CBENCH FREAD 0,1"));CHECK(Has(usb,"error CBENCH"));
    CHECK(Command("#CBENCH FULL_CANCEL"));CartesianBenchmarkPoll();CHECK(!BenchActive());
}

static void TestFullSuccessAndUnion(){
    Reset();FullStart();CartesianBenchmarkPoll();CHECK(benchJob.count==1);CHECK(benchJob.phase==BenchPhase::RUNNING);
    CHECK(OledDiagnostic::Busy());Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(benchJob.count==16);
    CHECK(!CartesianBenchmarkActive());CHECK(!OledDiagnostic::Busy());CHECK(!benchOledSessionActive);
    CHECK(!benchJob.fullReason&&!benchJob.fullWouldStop);
    for(unsigned i=0;i<benchJob.count;++i){
        const auto& frame=benchResults.full.frames[i];CHECK(frame.windowCycles>=frame.cycles);
        CHECK(!(frame.flags&CartCycle::FATAL));CHECK(frame.flags&CartCycle::EMITTED);
        CHECK(frame.reason==CartCycle::OK);CHECK(frame.ik<=5);
        CHECK(frame.startRtos>=frame.plannedRtos);CHECK(frame.endRtos>=frame.startRtos);
    }
    CHECK(Command("#CBENCH FULL"));CHECK(Has(usb,"phase=DONE"));CHECK(Has(usb,"no_motion=1"));
    WriteFullFixture("full-terminal.wire");
    for(const char* old:{"#CBENCH", "#CBENCH READ 0,1", "#CBENCH PREAD 0,1", "#CBENCH OREAD 0,1"}){
        CHECK(Command(old));CHECK(Has(usb,"error CBENCH"));
    }
    CHECK(Command("#CBENCH FREAD 15,2"));CHECK(Has(usb,"error CBENCH"));
    // Large signed motor fields and maximum id must not truncate USB rows.
    benchJob.id=0xffffffffU;auto& frame=benchResults.full.frames[15];
    frame.plannedRtos=frame.startRtos=frame.endRtos=frame.cycles=frame.windowCycles=frame.checksum=0xffffffffU;
    frame.ik=frame.fk=frame.reason=frame.flags=0xffffU;
    for(unsigned j=0;j<7;++j){frame.axes[j].q=INT32_MIN;frame.axes[j].v=INT32_MAX;frame.axes[j].a=INT32_MIN;}
    for(unsigned j=0;j<3;++j){frame.tcp[j]=2147483.f;frame.intent[j]=-2147483.f;}
    frame.railTarget=-2147483.f;frame.margin=1.f;CheckFullWire(15,1);
    // Switching the tagged union in either direction keeps both protocols usable.
    Start("3,0,5,3,20000,1,4");CHECK(!benchJob.full);Finish();CHECK(benchJob.phase==BenchPhase::DONE);
    CheckFrame(2);CheckProfileWire(0,3);CheckOledWire(0,3);
    CHECK(Command("#CBENCH FREAD 0,1"));CHECK(Has(usb,"error CBENCH"));
    FullStart("0,0,1,160,20000");Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(benchJob.count==160);CheckFullWire(156,4);
    benchJob.id=0xffffffffU;FullStart("0,0,1,1,20000");CHECK(benchJob.id==1);Finish();CHECK(benchJob.count==1);
    for(unsigned fixture=0;fixture<6;++fixture)for(unsigned variant=1;variant<=3;++variant){
        const std::string args="0,"+std::to_string(fixture)+","+std::to_string(variant)+",3,20000";
        FullStart(args.c_str());Finish();CHECK(benchJob.phase==BenchPhase::DONE);CHECK(benchJob.count==3);
        CHECK(benchJob.fullConfig.fixture==fixture);CHECK(benchJob.fullConfig.variant==variant);CheckFullWire(0,3);
    }
}

static void TestFullCancelAndFailure(){
    using Source=CartesianBenchmarkCancelSource;
    for(Source source:{Source::USB,Source::SOFT_STOP,Source::DISABLE,Source::MOTION}){
        Reset();FullStart();CartesianBenchmarkCancel(source);CHECK(benchJob.cancel);
        CHECK(benchJob.fullSource==uint8_t(source));CartesianBenchmarkCancel(Source::MOTION);
        CHECK(benchJob.fullSource==uint8_t(source));CartesianBenchmarkPoll();
        CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchJob.count==0);CHECK(!benchJob.fullWouldStop);
        CHECK(benchJob.fullReason==CartCycle::CANCEL);CHECK(!CartesianBenchmarkActive());
        CHECK(!dummy.stopNeedsSend);CHECK(dummy.stopState==DummyRobot::DISABLED);
    }
    for(const char* command:{"#CBENCH FULL_CANCEL", "#CBENCH CANCEL"}){
        Reset();FullStart();CartesianBenchmarkPoll();CHECK(Command(command));CartesianBenchmarkPoll();
        CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchJob.fullSource==uint8_t(Source::USB));
    }
    for(const char* command:{"!STOP", "!DISABLE"}){
        Reset();FullStart();CartesianBenchmarkPoll();CHECK(!Command(command));CartesianBenchmarkPoll();
        CHECK(benchJob.phase==BenchPhase::CANCELLED);
    }
    // Cancellation within an active full-chain window preserves the last
    // checked integer endpoint, including nonzero v/a, without submitting it.
    for(Source source:{Source::USB,Source::SOFT_STOP,Source::DISABLE,Source::MOTION}){
        Reset();FullStart("1,0,1,160,20000");
        for(unsigned i=0;i<8;++i)CartesianBenchmarkPoll();CHECK(BenchActive());
        const unsigned count=benchJob.count;const auto previous=benchResults.full.frames[count-1];
        bool moving=false;for(const auto& a:previous.axes)moving|=a.v!=0||a.a!=0;CHECK(moving);
        injectOnRead=cycleReads+3;cycleReadHook=[&]{CartesianBenchmarkCancel(source);};
        CartesianBenchmarkPoll();cycleReadHook={};CHECK(benchJob.phase==BenchPhase::CANCELLED);
        CHECK(benchJob.count==count+1);CHECK(benchJob.fullSource==uint8_t(source));CHECK(benchJob.fullFirst==count);
        const auto& cancelled=benchResults.full.frames[count];CHECK(cancelled.flags&CartCycle::CANCELLED);
        CHECK(!(cancelled.flags&CartCycle::EMITTED));CHECK(cancelled.checksum==0);
        for(unsigned j=0;j<7;++j){CHECK(cancelled.axes[j].q==previous.axes[j].q);CHECK(cancelled.axes[j].v==previous.axes[j].v);CHECK(cancelled.axes[j].a==previous.axes[j].a);}
        CHECK(!dummy.stopNeedsSend);CHECK(!CartesianBenchmarkActive());
    }
    Reset();FullStart();clockFrozen=true;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);
    CHECK(benchJob.fullWouldStop);CHECK(!dummy.stopNeedsSend);CHECK(!CartesianBenchmarkActive());
    std::vector<std::function<void()>> changes={[]{++dummy.motionEpoch;},[]{dummy.isEnabled=true;},[]{dummy.locked=true;},
        []{dummy.motionActive=true;},[]{streamActive=true;},[]{nativeOwned=true;},[]{dummy.commandHandler.busy=true;},
        []{dummy.calibration=true;},[]{dummy.parameter=true;},[]{dummy.rgbSaveBusy=true;},[]{DWT->CTRL=0;},
        []{CoreDebug->DEMCR=0;},[]{SystemCoreClock=84000000;}};
    for(const auto& change:changes){
        Reset();FullStart();CartesianBenchmarkPoll();change();CartesianBenchmarkPoll();
        CHECK(benchJob.phase!=BenchPhase::DONE);CHECK(!BenchActive());CHECK(benchJob.fullWouldStop);CHECK(!dummy.stopNeedsSend);
    }
    Reset();FullStart();CartesianBenchmarkPoll();bool injectedBudget=false;hookEveryRead=true;
    cycleReadHook=[&]{if(!injectedBudget&&benchProfileWindowActive){injectedBudget=true;AdvanceCycles(4000000);}};
    CartesianBenchmarkPoll();cycleReadHook={};hookEveryRead=false;CHECK(injectedBudget);
    CHECK(benchJob.phase==BenchPhase::BUDGET);CHECK(benchJob.fullWouldStop);CHECK(!dummy.stopNeedsSend);
    Reset();FullStart("5,1,2,160,20000");Finish();CHECK(benchJob.phase!=BenchPhase::DONE);
    CHECK(benchJob.fullReason);CHECK(benchJob.fullWouldStop);CHECK(benchJob.count>0);
    CHECK(benchResults.full.frames[benchJob.count-1].flags&CartCycle::FATAL);
    WriteFullFixture("full-fatal.wire");CHECK(!dummy.stopNeedsSend);CHECK(dummy.stopState==DummyRobot::DISABLED);
}

static void TestFullAtomicFinishAndRestore(){
    using Source=CartesianBenchmarkCancelSource;
    for(unsigned fault=0;fault<3;++fault){
        Reset();FullStart("0,0,1,2,20000");CartesianBenchmarkPoll();
        OledDiagnostic::stopHook=[&]{if(fault==0)CartesianBenchmarkCancel(Source::USB);else if(fault==1)++dummy.motionEpoch;else DWT->CTRL=0;};
        CartesianBenchmarkPoll();CHECK(benchJob.phase!=BenchPhase::DONE);CHECK(benchJob.fullReason);
        CHECK(!CartesianBenchmarkActive());CHECK(!benchOledSessionActive);CHECK(!dummy.stopNeedsSend);
    }
    for(unsigned fault=0;fault<2;++fault){
        Reset();FullStart("0,0,1,2,20000");CartesianBenchmarkPoll();
        OledDiagnostic::stopHook=[&]{enterCriticalHook=[&]{if(fault==0)CartesianBenchmarkCancel(Source::USB);else ++dummy.motionEpoch;};};
        CartesianBenchmarkPoll();CHECK(benchJob.phase==(fault==0?BenchPhase::CANCELLED:BenchPhase::ABORTED));
        CHECK(!enterCriticalHook);CHECK(!CartesianBenchmarkActive());
    }
    Reset();FullStart();CartesianBenchmarkPoll();OledDiagnostic::failStop=true;
    Command("#CBENCH FULL_CANCEL");CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::ABORTED);
    CHECK(CartesianBenchmarkActive());CHECK(!dummy.RequestStart());CHECK(!CartesianBenchmarkEnterCalibration());
    CHECK(Command("#CBENCH FULL_RUN 0,0,1,16,20000"));CHECK(Has(usb,"requires_idle_disabled"));
    const auto ticks=fakeRtosTicks;CartesianBenchmarkPoll();CHECK(fakeRtosTicks>ticks);CHECK(CartesianBenchmarkActive());
    OledDiagnostic::failStop=false;CartesianBenchmarkPoll();CHECK(!CartesianBenchmarkActive());CHECK(!dummy.stopNeedsSend);
}

int main(){
    TestSyntax();TestGates();TestRunReadAndIds();TestCancelAbortBudget();
    TestProfileProtocolAndOffPath();TestProfileClocksAndTaskResidency();TestProfileTerminalBoundaries();
    TestOledProtocolAndAccounting();TestOledRateAdmission();TestOledLifecycleAndLegacy();TestSplitDiagnostics();TestRealCalibrationWrapper();
    TestEnduranceBackend();TestEnduranceAdapter();
    TestFullSyntaxAndGates();TestFullSuccessAndUnion();TestFullCancelAndFailure();TestFullAtomicFinishAndRestore();
    CHECK(criticalDepth==0);
    std::printf("PASS assertions=%u max_respond_payload=%u endurance_state_bytes=%u; production adapter + real OLED/profile backend + RequestStart + calibration wrapper; independent fake HAL/RTOS/DWT only; no hardware\n",assertions,maxRespondPayload,unsigned(sizeof(CartEndurance::State)));
    return 0;
}
