// Host shim for the UNMODIFIED extracted production adapter. Fake time drives
// safety/control-flow edge cases; no result here measures MCU execution time.
#include "cartesian_benchmark.h"
#include "motion_values.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

static unsigned assertions=0;
#define CHECK(x) do {++assertions;if(!(x)){std::fprintf(stderr,"FAIL line=%d: %s\n",__LINE__,#x);std::exit(1);}} while(0)
using TickType_t=uint32_t;
using osThreadId_t=void*;
osThreadId_t ControlLoopUpdateHandle=reinterpret_cast<void*>(1);
static osThreadId_t currentThread=ControlLoopUpdateHandle;
static uint32_t fakeTicks=0;
uint32_t SystemCoreClock=168000000;
static uint32_t cycleReadStep=100,criticalDepth=0;
static bool clockFrozen=false;
static uint64_t clockRemainder=0;
static unsigned cycleReads=0,injectOnRead=0;
static std::function<void()> cycleReadHook;
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
    fakeTicks+=uint32_t(clockRemainder/SystemCoreClock);
    clockRemainder%=SystemCoreClock;
}
CycleRegister::operator uint32_t(){
    if(++cycleReads==injectOnRead&&cycleReadHook)cycleReadHook();
    if(!clockFrozen&&(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)&&(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk))AdvanceCycles(cycleReadStep);
    return value;
}
static void __DSB(){}
static void __ISB(){}
static void __NOP(){if(!clockFrozen&&(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk))AdvanceCycles(4);}
static uint32_t HAL_GetTick(){return fakeTicks;}
static osThreadId_t osThreadGetId(){return currentThread;}
static TickType_t xTaskGetTickCount(){return fakeTicks;}
static void EnterCritical(){++criticalDepth;}
static void ExitCritical(){CHECK(criticalDepth>0);--criticalDepth;}
#define taskENTER_CRITICAL() EnterCritical()
#define taskEXIT_CRITICAL() ExitCritical()
#define pdMS_TO_TICKS(x) (x)
static void osDelay(uint32_t n){CHECK(criticalDepth==0);fakeTicks+=n;if(!clockFrozen)DWT->CYCCNT.value+=uint32_t(uint64_t(SystemCoreClock)*n/1000);}
static void vTaskDelayUntil(TickType_t* wake,TickType_t period){const uint32_t target=*wake+period;if(int32_t(target-fakeTicks)>0)osDelay(target-fakeTicks);*wake=target;}

struct StreamSink {enum { CHANNEL_TYPE_USB,CHANNEL_TYPE_UART };int channelType=CHANNEL_TYPE_USB;std::vector<std::string> lines;};
static void Respond(StreamSink& sink,const char* format,...){char s[512];va_list args;va_start(args,format);std::vsnprintf(s,sizeof(s),format,args);va_end(args);sink.lines.emplace_back(s);}
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

// Generated directly from source frozen by the Python runner.
#include "adapter_fragment.inc"

static StreamSink usb,uart{StreamSink::CHANNEL_TYPE_UART,{}};
static bool Command(const std::string& text,StreamSink& response=usb){response.lines.clear();return dummy.HandleCartesianCommand(text.data(),unsigned(text.size()),response);}
static bool Has(const StreamSink& s,const char* text){for(const auto& line:s.lines)if(line.find(text)!=std::string::npos)return true;return false;}
static void Reset(){
    CHECK(!BenchActive());dummy=DummyRobot{};benchJob=BenchJob{};benchCalibrationEntry=false;plannerLease=false;
    streamActive=nativeOwned=false;currentThread=ControlLoopUpdateHandle;fakeTicks=0;cycleReadStep=100;
    clockFrozen=false;clockRemainder=0;SystemCoreClock=168000000;fakeDwt=FakeDWT{};fakeDebug=FakeDebug{};
    cycleReads=injectOnRead=0;cycleReadHook={};
    rgb=FakeRGB{};usb.lines.clear();uart.lines.clear();CHECK(criticalDepth==0);
}
static void Start(const char* args="2,0,1,3,20000"){
    CHECK(Command(std::string("#CBENCH RUN ")+args));CHECK(benchJob.phase==BenchPhase::PENDING);
}
static void Stop(){if(BenchActive()){Command("#CBENCH CANCEL");CartesianBenchmarkPoll();}CHECK(!BenchActive());}
static void Finish(){for(unsigned i=0;i<140&&BenchActive();++i)CartesianBenchmarkPoll();CHECK(!BenchActive());}

static void TestSyntax(){
    Reset();CHECK(Command("#CBENCH"));CHECK(Has(usb,"phase=IDLE"));
    CHECK(Command("#CBENCH RUN 2,0,1,1,20000",uart));CHECK(Has(uart,"usb_only"));CHECK(!BenchActive());
    const char* invalid[]={"#CBENCHX","#CBENCH RUN", "#CBENCH RUN 2,0,1,1", "#CBENCH RUN 2,0,1,1,20000,1",
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
}

static void TestGates(){
    using Set=std::function<void()>;
    std::vector<Set> blocked={[]{dummy.stopState=DummyRobot::NORMAL;},[]{dummy.stopState=DummyRobot::STARTING;},
        []{dummy.stopState=DummyRobot::DISABLING;},[]{dummy.isEnabled=true;},[]{dummy.startPending=true;},
        []{dummy.locked=true;},[]{dummy.motionActive=true;},[]{streamActive=true;},[]{plannerLease=true;},
        []{dummy.commandHandler.busy=true;},[]{dummy.calibration=true;},[]{dummy.parameter=true;},
        []{dummy.rgbSaveBusy=true;},[]{benchCalibrationEntry=true;},[]{nativeOwned=true;},[]{dummy.tuning=true;}};
    for(const auto& set:blocked){Reset();set();Command("#CBENCH RUN 2,0,1,1,20000");CHECK(Has(usb,"requires_idle_disabled"));CHECK(!BenchActive());}
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
    CHECK(benchJob.data[0].work==1);CHECK(benchJob.data[0].ik==1);CHECK(benchJob.data[0].fk==1);
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
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchJob.data[1].work==0);
    for(const char* s:{"!STOP","!DISABLE"}){Reset();Start();CHECK(!Command(s));CHECK(benchJob.cancel);CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CANCELLED);}
    std::vector<std::function<void()>> changes={[]{++dummy.motionEpoch;},[]{dummy.isEnabled=true;},[]{dummy.locked=true;},
        []{dummy.motionActive=true;},[]{streamActive=true;},[]{nativeOwned=true;},[]{dummy.commandHandler.busy=true;},
        []{dummy.calibration=true;},[]{dummy.parameter=true;},[]{dummy.rgbSaveBusy=true;},
        []{DWT->CTRL=0;},[]{CoreDebug->DEMCR=0;},[]{SystemCoreClock=84000000;},[]{fakeTicks+=10001;}};
    for(const auto& change:changes){Reset();Start();CartesianBenchmarkPoll();change();CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::ABORTED);CHECK(benchJob.data[1].work==0);}
    Reset();Start();clockFrozen=true;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);CHECK(benchJob.count==0);
    Reset();Start();SystemCoreClock=999999;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::CLOCK_ERROR);
    Reset();Start("2,0,1,3,100");CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::RUNNING);
    cycleReadStep=20000;CartesianBenchmarkPoll();CHECK(benchJob.phase==BenchPhase::BUDGET);CHECK(benchJob.data[1].flag==CartBench::BUDGET);
    Reset();Start("2,0,10,3,20000");CartesianBenchmarkPoll();
    injectOnRead=cycleReads+3;cycleReadHook=[]{benchJob.cancel=true;};CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::CANCELLED);CHECK(benchJob.data[1].work>=1);CHECK(benchJob.data[1].work<10);
    Reset();Start("2,0,10,3,20000");CartesianBenchmarkPoll();
    injectOnRead=cycleReads+3;cycleReadHook=[]{++dummy.motionEpoch;};CartesianBenchmarkPoll();
    CHECK(benchJob.phase==BenchPhase::ABORTED);CHECK(benchJob.data[1].work>=1);CHECK(benchJob.data[1].work<10);
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

int main(){
    TestSyntax();TestGates();TestRunReadAndIds();TestCancelAbortBudget();TestRealCalibrationWrapper();
    CHECK(criticalDepth==0);
    std::printf("PASS assertions=%u; production adapter + RequestStart + calibration wrapper; fake RTOS/DWT only; no hardware\n",assertions);
    return 0;
}
