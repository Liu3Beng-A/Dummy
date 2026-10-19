#include "common_inc.h"
#include "cartesian_planner.h"
#include "tcp_tool.h"
#include "motion_values.h"
#include "cartesian_benchmark.h"
#include "bench_profile.h"
#include "cartesian_endurance.h"
#include "cartesian_cycle_diagnostic.h"
#include <cstring>
#include <cmath>
extern DummyRobot dummy;
extern osThreadId_t ControlLoopUpdateHandle;
extern osThreadId_t controlLoopFixUpdateHandle,oledTaskHandle,rgbTaskHandle,usbServerTaskHandle,oledDisplayTaskHandle;
namespace {
NativeCartesian::Planner planner;
volatile bool plannerLease=false;
uint32_t maxPlanningUs=0,lastPlanningUs=0,lastWork=0;
uint32_t maxIKCheckUs=0;
float lastBound=0,lastRotation=0;
const char* lastPlan="NONE";
class RobotProgress final:public NativeCartesian::Progress {
public:
    explicit RobotProgress(uint32_t e):epoch(e),began(HAL_GetTick()){}
    bool budgetExpired=false;
    bool Checkpoint() override {
        if(dummy.GetMotionEpoch()!=epoch||dummy.IsStopLocked()||!dummy.IsEnabled())return false;
        if(HAL_GetTick()-began>30000){budgetExpired=true;return false;}
        if((++calls&31)==0){CoreNativeHeartbeat(epoch);osDelay(1);}
        return true;
    }
private:uint32_t epoch,began,calls=0;
};
bool Equal(const char* cmd,unsigned n,const char* literal){return std::strlen(literal)==n&&!std::memcmp(cmd,literal,n);}
bool Prefix(const char* cmd,unsigned n,const char* literal){unsigned k=std::strlen(literal);return n>=k&&!std::memcmp(cmd,literal,k);}
bool ExactMotorR(int32_t q,float& value){
    value=float(q)/51200.f;
    for(unsigned retry=0;retry<4;++retry){volatile float scaled=value*51200.f;
        if(scaled>=2147483648.f||scaled<-2147483648.f)return false;
        if(int32_t(scaled)==q)return true;
        value=std::nextafterf(value,q>=0?1.e30f:-1.e30f);
    }
    return false;
}
void PrintTool(StreamSink& response){auto p=TcpTool::Get();auto s=TcpTool::GetStatus();
    Respond(response,"TOOL v=1 rev=%lu dirty=%u storage=%u",(unsigned long)s.revision,unsigned(s.dirty),unsigned(s.storage));
    Respond(response,"TOOL XYZ %.3f %.3f %.3f",p.X*1000,p.Y*1000,p.Z*1000);
    Respond(response,"TOOL ABC %.3f %.3f %.3f",p.A,p.B,p.C);}

enum class BenchPhase : uint8_t { IDLE, PENDING, RUNNING, DONE, CANCELLED, ABORTED, BUDGET, CLOCK_ERROR };
// Fixed SRAM result storage; runtime task hand-over is owned by main.cpp.
// DWT window surrounds Measure, so it includes small adapter/callback costs.
// Task residency includes IRQ time and is not exclusive algorithm CPU time.
struct BenchFrame {
    uint32_t plannedRtos=0,startRtos=0,endRtos=0,startHal=0,endHal=0,startDwt=0,endDwt=0;
    uint32_t workCycles[BenchProfile::MaxWork]{},resident[BenchProfile::TaskCount]{};
    uint32_t oledResident[BenchProfile::OledStageCount]{},oledElapsed[BenchProfile::OledStageCount]{};
    uint32_t displayResident[BenchProfile::OledStageCount]{},displayElapsed[BenchProfile::OledStageCount]{};
    uint32_t flags=0;
};
static_assert(sizeof(BenchFrame)==160,"profile storage budget changed");
struct LegacyResults { BenchFrame frames[CartBench::MaxSamples]{}; CartBench::Sample data[CartBench::MaxSamples]{}; };
struct FullResults { CartCycle::Frame frames[CartCycle::MaxSamples]{}; };
union BenchResults {
    LegacyResults legacy;
    FullResults full;
    BenchResults():legacy{}{}
};
static_assert(sizeof(FullResults)<=sizeof(LegacyResults),"full-cycle results must reuse the existing buffer");
BenchResults benchResults;
bool benchResultsFull=false;
struct BenchJob {
    CartBench::Config config{};
    CartCycle::Config fullConfig{};
    CartCycle::Summary fullSummary{};
    bool full=false,fullWouldStop=false;
    uint16_t fullReason=0;
    uint8_t fullSource=0;
    uint32_t fullFirst=0;
    volatile BenchPhase phase=BenchPhase::IDLE;
    volatile bool cancel=false;
    uint32_t id=0,epoch=0,clock=0,ctrl=0,demcr=0,began=0;
    uint32_t probeCycles=0,probeTicks=0,setupCycles=0;
    unsigned count=0;
    TickType_t wake=0;
    bool profile=false;
    bool oledProfile=false;
    uint8_t oledMode=0;
    bool endurance=false;
    uint16_t seconds=0;
    uint32_t longStartRtos=0,longStartHal=0,longCount=0,longElapsed=0,longError=0;
};
BenchJob benchJob;
CartEndurance::State benchLong;
CartBench::Compute benchCompute;
CartCycle::Compute benchCycle;
bool benchCalibrationEntry=false;
const char* BenchName(BenchPhase phase) {
    static const char* const names[]={"IDLE","PENDING","RUNNING","DONE","CANCELLED","ABORTED","BUDGET","CLOCK_ERROR"};
    return names[unsigned(phase)];
}
bool BenchActive() { return benchJob.phase==BenchPhase::PENDING||benchJob.phase==BenchPhase::RUNNING; }
uint8_t BenchCheckpoint(void*);
void BenchFullFail(uint16_t reason,bool wouldStop,unsigned sequence) {
    taskENTER_CRITICAL();
    if(!benchJob.fullReason){benchJob.fullReason=reason;benchJob.fullWouldStop=wouldStop;benchJob.fullFirst=sequence;}
    taskEXIT_CRITICAL();
}
void BenchLongPublish(){
    taskENTER_CRITICAL();benchJob.longCount=benchLong.n;benchJob.longElapsed=benchLong.elapsedMs;
    benchJob.longError=benchLong.error;taskEXIT_CRITICAL();
}
void BenchLongStacks(){
    const uint32_t at=DWT->CYCCNT;
    // CMSIS osThreadGetStackSpace already multiplies the FreeRTOS word count
    // by sizeof(StackType_t); these three values are bytes.
    const uint32_t c=osThreadGetStackSpace(ControlLoopUpdateHandle),w=osThreadGetStackSpace(oledTaskHandle),d=osThreadGetStackSpace(oledDisplayTaskHandle);
    benchLong.Stacks(c,w,d,uint32_t(DWT->CYCCNT-at));
}
void BenchFinish(BenchPhase phase) {
    if(benchJob.endurance){
        if(phase!=BenchPhase::DONE)benchLong.Fail(phase==BenchPhase::CANCELLED?CartEndurance::CANCELLED:
            phase==BenchPhase::BUDGET?CartEndurance::BUDGET:phase==BenchPhase::CLOCK_ERROR?CartEndurance::CLOCK:CartEndurance::ENVIRONMENT,benchLong.n);
        if(benchJob.phase==BenchPhase::RUNNING&&
           !benchLong.AdvanceClock(xTaskGetTickCount()-benchJob.longStartRtos,HAL_GetTick()-benchJob.longStartHal,DWT->CYCCNT)&&phase==BenchPhase::DONE)
            phase=BenchPhase::CLOCK_ERROR;
        BenchLongStacks();
    }
    // Seal counters atomically; restoration may yield and must not hold a lock.
    taskENTER_CRITICAL();BenchProfile::OledSessionStop();taskEXIT_CRITICAL();
    const bool restored=OledDiagnostic::Stop();
    if(benchJob.full){
        // Restoration yields. Guard and terminal publication share the lease
        // lock so a STOP/cancel arriving during restoration cannot become DONE.
        taskENTER_CRITICAL();
        if(!restored)BenchFullFail(34,true,benchJob.count);
        if(phase==BenchPhase::DONE){
            const auto guard=BenchCheckpoint(nullptr);
            const bool clockBad=!(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock;
            if(guard!=CartBench::COMPLETE){
                BenchFullFail(guard==CartBench::CANCELLED?CartCycle::CANCEL:clockBad?32:37,guard!=CartBench::CANCELLED,benchJob.count);
                phase=guard==CartBench::CANCELLED?BenchPhase::CANCELLED:clockBad?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED;
            }
        }
        if(phase!=BenchPhase::DONE&&!benchJob.fullReason)
            BenchFullFail(phase==BenchPhase::CANCELLED?CartCycle::CANCEL:phase==BenchPhase::CLOCK_ERROR?32:37,phase!=BenchPhase::CANCELLED,benchJob.count);
        benchJob.phase=restored?phase:BenchPhase::ABORTED;
        taskEXIT_CRITICAL();return;
    }
    if(benchJob.endurance){
        if(!restored)benchLong.Fail(CartEndurance::RESTORE,benchLong.n);
        if(phase==BenchPhase::DONE){
            const auto guard=BenchCheckpoint(nullptr);
            const bool clockBad=!(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock;
            if(guard!=CartBench::COMPLETE)benchLong.Fail(guard==CartBench::CANCELLED?CartEndurance::CANCELLED:clockBad?CartEndurance::CLOCK:CartEndurance::ENVIRONMENT,benchLong.n);
            benchLong.AdvanceClock(xTaskGetTickCount()-benchJob.longStartRtos,HAL_GetTick()-benchJob.longStartHal,DWT->CYCCNT);
            if(benchLong.error)phase=benchLong.error==CartEndurance::CANCELLED?BenchPhase::CANCELLED:
                benchLong.error==CartEndurance::CLOCK?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED;
        }
        // Last admission/environment check and publication share one short
        // critical section. USB CANCEL/STOP cannot slip between them and DONE.
        taskENTER_CRITICAL();
        if(phase==BenchPhase::DONE){
            const auto guard=BenchCheckpoint(nullptr);
            const bool clockBad=!(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock;
            if(guard!=CartBench::COMPLETE)benchLong.Fail(guard==CartBench::CANCELLED?CartEndurance::CANCELLED:clockBad?CartEndurance::CLOCK:CartEndurance::ENVIRONMENT,benchLong.n);
            if(benchLong.error)phase=benchLong.error==CartEndurance::CANCELLED?BenchPhase::CANCELLED:
                benchLong.error==CartEndurance::CLOCK?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED;
        }
        benchJob.longCount=benchLong.n;benchJob.longElapsed=benchLong.elapsedMs;benchJob.longError=benchLong.error;
        benchJob.phase=restored?phase:BenchPhase::ABORTED;taskEXIT_CRITICAL();return;
    }
    taskENTER_CRITICAL();benchJob.phase=restored?phase:BenchPhase::ABORTED;taskEXIT_CRITICAL();
}
void BenchLongStatus(StreamSink& response){
    uint32_t id=0,hz=0,count=0,elapsed=0,error=0;unsigned op=0,fixture=0,mode=0,seconds=0;BenchPhase phase=BenchPhase::IDLE;
    taskENTER_CRITICAL();
    if(benchJob.endurance){id=benchJob.id;hz=benchJob.clock;count=benchJob.longCount;elapsed=benchJob.longElapsed;error=benchJob.longError;
        op=benchJob.config.operation;fixture=benchJob.config.fixture;mode=benchJob.oledMode;seconds=benchJob.seconds;phase=benchJob.phase;}
    taskEXIT_CRITICAL();
    Respond(response,"CBLONG v=1 id=%lu phase=%s",(unsigned long)id,BenchName(phase));
    Respond(response,"CBLONG caps seconds=600 period_ms=20 work=5");
    Respond(response,"CBLONG caps bins=21 trends=60 history=8");
    Respond(response,"CBLONG cfg op=%u fixture=%u mode=%u seconds=%u",op,fixture,mode,seconds);
    Respond(response,"CBLONG cfg work=5 budget_us=20000 planned=%u",seconds*50U);
    Respond(response,"CBLONG clock hz=%lu tick_hz=%lu",(unsigned long)hz,(unsigned long)configTICK_RATE_HZ);
    Respond(response,"CBLONG run n=%lu elapsed_ms=%lu",(unsigned long)count,(unsigned long)elapsed);
    Respond(response,"CBLONG stop reason=%s",CartEndurance::Name(CartEndurance::Error(error)));
    Respond(response,"CBLONG no_motion=1 scope=components");
    Respond(response,"CBLONG end id=%lu",(unsigned long)id);
}
void BenchLongRead(StreamSink& response){
    taskENTER_CRITICAL();const bool allowed=benchJob.endurance&&!BenchActive();const uint32_t id=benchJob.id;taskEXIT_CRITICAL();
    if(!allowed){Respond(response,"error CBENCH long_read_range_or_busy");return;}
    // Only USB admits jobs. It cannot admit another while this same USB handler
    // prints terminal immutable aggregates. Never copy/print bulk in a lock.
    const auto& s=benchLong;
    Respond(response,"CBLONG read v=1 id=%lu",(unsigned long)id);
    Respond(response,"CBLONG total %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.n,(unsigned long)s.elapsedMs);
    Respond(response,"CBLONG sum %lu,%lu,%lu",(unsigned long)id,(unsigned long)(s.sumCycles>>32),(unsigned long)uint32_t(s.sumCycles));
    Respond(response,"CBLONG window %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)(s.sumWindow>>32),(unsigned long)uint32_t(s.sumWindow),(unsigned long)s.maxWindow);
    Respond(response,"CBLONG dwt %lu,%lu,%lu",(unsigned long)id,(unsigned long)(s.extendedDwt>>32),(unsigned long)uint32_t(s.extendedDwt));
    Respond(response,"CBLONG minmax %lu,%lu,%lu",(unsigned long)id,(unsigned long)(s.n?s.minCycles:0),(unsigned long)s.maxCycles);
    Respond(response,"CBLONG late %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)s.lateCount,(unsigned long)s.maxLate,(unsigned long)s.maxGap);
    Respond(response,"CBLONG error %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.error,(unsigned long)s.errorSequence);
    Respond(response,"CBLONG sig %lu,%u,%u,%u,%u,%u",(unsigned long)id,unsigned(s.signature.fk),unsigned(s.signature.ik),unsigned(s.signature.valid),unsigned(s.signature.selected),unsigned(s.signature.failed));
    Respond(response,"CBLONG hash %lu,%lu",(unsigned long)id,(unsigned long)s.signature.checksum);
    Respond(response,"CBLONG counts %lu,21,%lu,%lu",(unsigned long)id,(unsigned long)s.trendCount,(unsigned long)s.historyCount);
    Respond(response,"CBLONG overhead %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)(s.sumOverhead>>32),(unsigned long)uint32_t(s.sumOverhead),(unsigned long)s.maxOverhead);
    Respond(response,"CBLONG scan %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.scanCount,(unsigned long)s.maxScan);
    for(unsigned i=0;i<CartEndurance::Bins;++i)Respond(response,"CBLONG H %lu,%u,%lu",(unsigned long)id,i,(unsigned long)s.histogram[i]);
    for(unsigned i=0;i<BenchProfile::TaskCount;++i)Respond(response,"CBLONG P %lu,%u,%lu,%lu",(unsigned long)id,i,(unsigned long)(s.resident[i]>>32),(unsigned long)uint32_t(s.resident[i]));
    for(unsigned l=0;l<4;++l)for(unsigned k=0;k<BenchProfile::OledStageCount;++k)
        Respond(response,"CBLONG O %lu,%u,%u,%lu,%lu",(unsigned long)id,l,k,(unsigned long)(s.lane[l][k]>>32),(unsigned long)uint32_t(s.lane[l][k]));
    for(unsigned i=0;i<s.trendCount;++i){const auto& t=s.trends[i];
        Respond(response,"CBLONG T %lu,%u,%u,%lu,%lu",(unsigned long)id,i,unsigned(t.n),(unsigned long)(t.sumCycles>>32),(unsigned long)uint32_t(t.sumCycles));
        Respond(response,"CBLONG U %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)t.maxCycles,(unsigned long)t.maxLate,(unsigned long)t.maxGap);
        Respond(response,"CBLONG K %lu,%u,%u,%u,%u",(unsigned long)id,i,unsigned(t.commandFree),unsigned(t.workerFree),unsigned(t.displayFree));
    }
    for(unsigned i=0;i<s.historyCount;++i){const auto& h=s.HistoryAt(i);
        Respond(response,"CBLONG F %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)h.sequence,(unsigned long)h.elapsedMs,(unsigned long)h.windowCycles);
        Respond(response,"CBLONG G %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)h.lateTicks,(unsigned long)h.sampleCycles,(unsigned long)h.flags);
        Respond(response,"CBLONG J %lu,%u,%lu,%u",(unsigned long)id,i,(unsigned long)h.checksum,unsigned(h.failed));
    }
    Respond(response,"CBLONG read_end id=%lu",(unsigned long)id);
}
void BenchOledStatus(StreamSink& response) {
    uint32_t id;bool enabled;unsigned mode;BenchProfile::OledSummary s,d;
    taskENTER_CRITICAL();id=benchJob.id;enabled=benchJob.oledProfile;mode=benchJob.oledMode;
    s=BenchProfile::OledSnapshot();d=BenchProfile::OledDisplaySnapshot();taskEXIT_CRITICAL();
    const auto r=OledDiagnostic::Snapshot();
    Respond(response,"CBOLED v=3 id=%lu enabled=%u skip=%u mode=%u",(unsigned long)id,unsigned(enabled),unsigned(mode==1),mode);
    Respond(response,"CBOLED caps samples=128 stages=5 lanes=2");
    Respond(response,"CBOLED rate period_ticks=%lu tick_hz=%lu",(unsigned long)(enabled&&mode>=2?pdMS_TO_TICKS(100):0),(unsigned long)configTICK_RATE_HZ);
    Respond(response,"CBOLED F %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)s.begun,(unsigned long)s.ended,(unsigned long)s.skipped);
    Respond(response,"CBOLED L %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.lateFrames,(unsigned long)s.maxLateTicks);
    Respond(response,"CBOLED M %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.overrunFrames,(unsigned long)s.maxFrameCycles);
    Respond(response,"CBOLED R %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.renderBegun,(unsigned long)s.rateLimited);
    Respond(response,"CBOLED G %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.minRenderGapTicks,(unsigned long)s.maxRenderGapTicks);
    Respond(response,"CBOLED D %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)d.begun,(unsigned long)d.ended,(unsigned long)d.skipped);
    Respond(response,"CBOLED J %lu,%lu,%lu",(unsigned long)id,(unsigned long)d.lateFrames,(unsigned long)d.maxLateTicks);
    Respond(response,"CBOLED N %lu,%lu,%lu",(unsigned long)id,(unsigned long)d.overrunFrames,(unsigned long)d.maxFrameCycles);
    Respond(response,"CBOLED U %lu,%lu,%lu",(unsigned long)id,(unsigned long)d.renderBegun,(unsigned long)d.rateLimited);
    Respond(response,"CBOLED H %lu,%lu,%lu",(unsigned long)id,(unsigned long)d.minRenderGapTicks,(unsigned long)d.maxRenderGapTicks);
    Respond(response,"CBOLED S %lu,%lu,%lu",(unsigned long)id,(unsigned long)s.maxLoopGapTicks,(unsigned long)d.maxLoopGapTicks);
    Respond(response,"CBOLED A %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)r.maxSnapshotAgeTicks,(unsigned long)r.overwritten,(unsigned long)r.maxSendGapTicks);
    Respond(response,"CBOLED C %lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)r.flags,(unsigned long)r.restored,(unsigned long)r.priority);
    Respond(response,"CBOLED K %lu,%lu,2048",(unsigned long)id,(unsigned long)r.displayStackFree);
    Respond(response,"CBOLED end id=%lu",(unsigned long)id);
}
void BenchProfileStatus(StreamSink& response) {
    uint32_t id,mask;unsigned count,mode;
    taskENTER_CRITICAL();id=benchJob.id;count=benchJob.count;mode=benchJob.profile?1U:0U;mask=BenchProfile::MapMask();taskEXIT_CRITICAL();
    Respond(response,"CBPROF v=2 id=%lu mode=%u n=%u",(unsigned long)id,mode,count);
    Respond(response,"CBPROF caps samples=128 work=5 tasks=7");
    Respond(response,"CBPROF mapmask=%lu irq_included=1",(unsigned long)mask);
    Respond(response,"CBPROF end id=%lu",(unsigned long)id);
}
void BenchFullStatus(StreamSink& response) {
    CartCycle::Config c;CartCycle::Summary s{};uint32_t id=0,hz=0,count=0,first=0,setup=0;
    unsigned reason=0,source=0,wouldStop=0;BenchPhase phase=BenchPhase::IDLE;
    taskENTER_CRITICAL();
    if(benchJob.full){c=benchJob.fullConfig;s=benchJob.fullSummary;id=benchJob.id;hz=benchJob.clock;count=benchJob.count;
        first=benchJob.fullFirst;setup=benchJob.setupCycles;reason=benchJob.fullReason;source=benchJob.fullSource;wouldStop=benchJob.fullWouldStop;phase=benchJob.phase;}
    taskEXIT_CRITICAL();
    Respond(response,"CFULL v=1 id=%lu phase=%s",(unsigned long)id,BenchName(phase));
    Respond(response,"CFULL caps samples=160 scenarios=13 variants=3");
    Respond(response,"CFULL cfg case=%u fixture=%u variant=%u samples=%u",unsigned(c.scenario),unsigned(c.fixture),unsigned(c.variant),unsigned(c.samples));
    Respond(response,"CFULL clock hz=%lu tick_hz=%lu",(unsigned long)hz,(unsigned long)configTICK_RATE_HZ);
    Respond(response,"CFULL run n=%lu planned=%u budget=20000",(unsigned long)count,unsigned(c.samples));
    Respond(response,"CFULL fault reason=%u source=%u would_stop=%u",reason,source,wouldStop);
    Respond(response,"CFULL first seq=%lu",(unsigned long)first);
    Respond(response,"CFULL result emitted=%lu accepted=%lu",(unsigned long)s.emitted,(unsigned long)s.accepted);
    Respond(response,"CFULL work ik=%lu fk=%lu peak=%u",(unsigned long)s.ik,(unsigned long)s.fk,unsigned(s.ikPeak));
    Respond(response,"CFULL setup cycles=%lu",(unsigned long)setup);
    Respond(response,"CFULL no_motion=1 scope=full_cycle");
    Respond(response,"CFULL end id=%lu",(unsigned long)id);
}
int32_t BenchScaled(float value,float scale) {
    // Release is -Ofast: bitwise finite check must precede float-to-int casts.
    uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));
    if((bits&0x7f800000U)==0x7f800000U)return 0;
    const float scaled=value*scale;
    if(scaled>=2147483648.f)return INT32_MAX;
    if(scaled<-2147483648.f)return INT32_MIN;
    return int32_t(std::round(scaled));
}
void BenchFullRead(StreamSink& response,unsigned first,unsigned amount) {
    taskENTER_CRITICAL();const bool allowed=benchJob.full&&!BenchActive()&&first+amount<=benchJob.count;
    const uint32_t id=benchJob.id;taskEXIT_CRITICAL();
    if(!allowed){Respond(response,"error CBENCH full_read_range_or_busy");return;}
    // USB is the sole job admission owner. Terminal frames remain immutable
    // throughout this USB response; no copies/printing in a critical section.
    Respond(response,"CFULL read id=%lu first=%u n=%u",(unsigned long)id,first,amount);
    for(unsigned i=first;i<first+amount;++i){const auto& f=benchResults.full.frames[i];
        Respond(response,"CFULL T %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)f.plannedRtos,(unsigned long)f.startRtos,(unsigned long)f.endRtos);
        Respond(response,"CFULL C %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)f.cycles,(unsigned long)f.windowCycles,(unsigned long)f.checksum);
        Respond(response,"CFULL S %lu,%u,%u,%u,%u,%u",(unsigned long)id,i,unsigned(f.ik),unsigned(f.fk),unsigned(f.reason),unsigned(f.flags));
        Respond(response,"CFULL P %lu,%u,%ld,%ld,%ld",(unsigned long)id,i,(long)BenchScaled(f.tcp[0],1000),(long)BenchScaled(f.tcp[1],1000),(long)BenchScaled(f.tcp[2],1000));
        Respond(response,"CFULL I %lu,%u,%ld,%ld,%ld",(unsigned long)id,i,(long)BenchScaled(f.intent[0],1000),(long)BenchScaled(f.intent[1],1000),(long)BenchScaled(f.intent[2],1000));
        Respond(response,"CFULL R %lu,%u,%ld,%ld",(unsigned long)id,i,(long)BenchScaled(f.railTarget,1000),(long)BenchScaled(f.margin,1000000));
        for(unsigned a=0;a<7;++a){
            Respond(response,"CFULL Q %lu,%u,%u,%ld",(unsigned long)id,i,a,(long)f.axes[a].q);
            Respond(response,"CFULL V %lu,%u,%u,%ld,%ld",(unsigned long)id,i,a,(long)f.axes[a].v,(long)f.axes[a].a);
        }
        Respond(response,"CFULL E %lu,%u",(unsigned long)id,i);
    }
    Respond(response,"CFULL read_end id=%lu first=%u n=%u",(unsigned long)id,first,amount);
}
void BenchWorkBoundary(void*,unsigned completed,uint32_t cumulative) {
    BenchFrame& frame=benchResults.legacy.frames[benchJob.count];
    if(completed<1||completed>BenchProfile::MaxWork){frame.flags|=1U<<16;return;}
    frame.workCycles[completed-1]=cumulative;
}
void BenchStatus(StreamSink& response) {
    CartBench::Config c;uint32_t id,hz,ctrl,demcr,pc,pt,sc;unsigned count;BenchPhase phase;
    taskENTER_CRITICAL();c=benchJob.config;id=benchJob.id;hz=benchJob.clock;ctrl=benchJob.ctrl;
    demcr=benchJob.demcr;count=benchJob.count;phase=benchJob.phase;
    pc=benchJob.probeCycles;pt=benchJob.probeTicks;sc=benchJob.setupCycles;taskEXIT_CRITICAL();
    Respond(response,"CBENCH v=1 id=%lu phase=%s",(unsigned long)id,BenchName(phase));
    Respond(response,"CBENCH config op=%u fixture=%u work=%u samples=%u",unsigned(c.operation),unsigned(c.fixture),unsigned(c.work),unsigned(c.samples));
    Respond(response,"CBENCH clock_hz=%lu budget_us=%lu n=%u",(unsigned long)hz,(unsigned long)c.budgetUs,count);
    Respond(response,"CBENCH period_ms=20");
    Respond(response,"CBENCH dwt_ctrl=%lu demcr=%lu",(unsigned long)ctrl,(unsigned long)demcr);
    Respond(response,"CBENCH probe_cycles=%lu probe_ticks=%lu",(unsigned long)pc,(unsigned long)pt);
    Respond(response,"CBENCH setup_cycles=%lu",(unsigned long)sc);
    Respond(response,"CBENCH no_motion=1 scope=components");
}
bool BenchUnsigned(const float* values,const uint32_t* limits,unsigned count) {
    for(unsigned i=0;i<count;++i)if(values[i]<0||values[i]>limits[i]||std::floor(values[i])!=values[i])return false;
    return true;
}
uint32_t BenchCycles(void*) { return DWT->CYCCNT; }
uint32_t BenchTicks(void*) { return HAL_GetTick(); }
uint8_t BenchCheckpoint(void*) {
    if(benchJob.cancel)return CartBench::CANCELLED;
    if(HAL_GetTick()-benchJob.began>(benchJob.endurance?uint32_t(benchJob.seconds)*1000U+2000U:10000U)||dummy.GetMotionEpoch()!=benchJob.epoch||
       dummy.IsEnabled()||dummy.IsStopLocked()||dummy.IsMotionBusy()||dummy.commandHandler.IsBusy()||
       dummy.IsCalibrationBusy()||dummy.IsProtectionParameterBusy()||dummy.IsRGBSaveBusy()||
       !(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||
       SystemCoreClock!=benchJob.clock)return CartBench::ABORTED;
    return CartBench::COMPLETE;
}
bool BenchClockStart() {
    benchJob.clock=SystemCoreClock;
    if(benchJob.clock<1000000||benchJob.clock>300000000)return false;
    // Do not reset the shared counter or replace global micros().
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;__DSB();__ISB();
    benchJob.ctrl=DWT->CTRL;benchJob.demcr=CoreDebug->DEMCR;
    uint32_t at=DWT->CYCCNT;for(unsigned i=0;i<32;++i)__NOP();
    if(DWT->CYCCNT==at)return false;
    if(benchJob.oledProfile) {
        taskENTER_CRITICAL();BenchProfile::OledSessionStart(benchJob.oledMode,benchJob.clock,pdMS_TO_TICKS(100));taskEXIT_CRITICAL();
    }
    const uint32_t tick=HAL_GetTick();at=DWT->CYCCNT;
    osDelay(20);
    benchJob.probeCycles=DWT->CYCCNT-at;benchJob.probeTicks=HAL_GetTick()-tick;
    const uint64_t milliseconds=uint64_t(benchJob.probeCycles)*1000/benchJob.clock;
    return benchJob.probeTicks>=10&&benchJob.probeTicks<1000&&
        milliseconds+2>=benchJob.probeTicks&&milliseconds<=benchJob.probeTicks+2;
}
void BenchFullPoll() {
    const auto guard=BenchCheckpoint(nullptr);
    if(guard!=CartBench::COMPLETE){
        const bool clockBad=!(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock;
        BenchFullFail(guard==CartBench::CANCELLED?CartCycle::CANCEL:clockBad?32:37,guard!=CartBench::CANCELLED,benchJob.count);
        BenchFinish(guard==CartBench::CANCELLED?BenchPhase::CANCELLED:clockBad?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED);return;
    }
    const uint32_t planned=benchJob.wake,start=xTaskGetTickCount(),hal=HAL_GetTick();
    if(uint32_t(start-planned)>=pdMS_TO_TICKS(20)){BenchFullFail(33,true,benchJob.count);BenchFinish(BenchPhase::ABORTED);return;}
    uint32_t at;
    taskENTER_CRITICAL();at=DWT->CYCCNT;const bool begun=BenchProfile::Begin(at,ControlLoopUpdateHandle);taskEXIT_CRITICAL();
    if(!begun){taskENTER_CRITICAL();BenchProfile::Reset();taskEXIT_CRITICAL();BenchFullFail(35,true,benchJob.count);BenchFinish(BenchPhase::ABORTED);return;}
    const CartBench::Hooks hooks{nullptr,BenchCycles,BenchTicks,BenchCheckpoint,benchJob.clock,nullptr};
    auto& f=benchResults.full.frames[benchJob.count];f=benchCycle.Measure(hooks);
    taskENTER_CRITICAL();const uint32_t end=DWT->CYCCNT;const auto w=BenchProfile::End(end);
    f.plannedRtos=planned;f.startRtos=start;f.endRtos=xTaskGetTickCount();f.windowCycles=end-at;
    const uint32_t elapsed=HAL_GetTick()-hal;taskEXIT_CRITICAL();
    uint32_t sum=0;for(auto v:w.resident)sum+=v;
    const uint64_t ms=uint64_t(f.windowCycles)*1000U/benchJob.clock;
    BenchPhase phase=BenchPhase::RUNNING;
    if(elapsed>=1000||ms+2<elapsed||ms>uint64_t(elapsed)+2U||
       !(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock){
        BenchFullFail(32,true,benchJob.count);phase=BenchPhase::CLOCK_ERROR;
    }else if(w.flags||sum!=w.cycles||f.windowCycles!=w.cycles||f.windowCycles<f.cycles){
        BenchFullFail(35,true,benchJob.count);phase=BenchPhase::ABORTED;
    }else if(uint64_t(f.windowCycles)*1000000U>uint64_t(benchJob.clock)*20000U||(f.flags&CartCycle::BUDGET)){
        BenchFullFail(CartCycle::TIME_BUDGET,true,benchJob.count);phase=BenchPhase::BUDGET;
    }else if(f.flags&CartCycle::CANCELLED){BenchFullFail(CartCycle::CANCEL,false,benchJob.count);phase=BenchPhase::CANCELLED;
    }else if(f.flags&(CartCycle::FATAL|CartCycle::ABORTED)){
        BenchFullFail(f.reason?f.reason:uint16_t(37),true,benchJob.count);phase=BenchPhase::ABORTED;
    }
    taskENTER_CRITICAL();benchJob.fullSummary=benchCycle.Totals();++benchJob.count;taskEXIT_CRITICAL();
    if(phase!=BenchPhase::RUNNING)BenchFinish(phase);
    else if(benchJob.count==benchJob.fullConfig.samples)BenchFinish(BenchPhase::DONE);
}
}

bool CartesianBenchmarkActive() { return BenchActive()||OledDiagnostic::Busy(); }
void CartesianBenchmarkCancel(CartesianBenchmarkCancelSource source) {
    taskENTER_CRITICAL();
    if(BenchActive()){
        benchJob.cancel=true;
        if(benchJob.full&&!benchJob.fullReason){benchJob.fullSource=uint8_t(source);BenchFullFail(CartCycle::CANCEL,false,benchJob.count);}
    }
    taskEXIT_CRITICAL();
}
bool CartesianBenchmarkEnterCalibration() {
    taskENTER_CRITICAL();
    const bool allowed=!BenchActive()&&!OledDiagnostic::Busy()&&!benchCalibrationEntry;
    if(allowed)benchCalibrationEntry=true;
    taskEXIT_CRITICAL();return allowed;
}
void CartesianBenchmarkLeaveCalibration() { taskENTER_CRITICAL();benchCalibrationEntry=false;taskEXIT_CRITICAL(); }
void CartesianBenchmarkPoll() {
    // Only the normal-priority command task may execute a computation sample.
    if(osThreadGetId()!=ControlLoopUpdateHandle)return;
    if(!BenchActive()) {
        if(OledDiagnostic::Busy()){osDelay(1);OledDiagnostic::Stop();}
        return;
    }
    if(benchJob.phase==BenchPhase::PENDING) {
        benchJob.began=HAL_GetTick();
        if(benchJob.endurance)benchLong.Reset(SystemCoreClock,benchJob.seconds,DWT->CYCCNT);
        // Reservation is still active during clock/setup validation.
        if(benchJob.cancel){BenchFinish(BenchPhase::CANCELLED);return;}
        if(!OledDiagnostic::Start(benchJob.oledMode)){BenchFinish(BenchPhase::ABORTED);return;}
        if(!BenchClockStart()){BenchFinish(BenchPhase::CLOCK_ERROR);return;}
        if(BenchCheckpoint(nullptr)!=CartBench::COMPLETE){BenchFinish(benchJob.cancel?BenchPhase::CANCELLED:BenchPhase::ABORTED);return;}
        const uint32_t at=DWT->CYCCNT;
        // Explicitly begin the union member lifetime outside the lease lock.
        // Placement new uses the existing static storage and allocates no heap.
        if(benchJob.full){new (&benchResults.full) FullResults{};benchResultsFull=true;}
        else if(benchResultsFull){new (&benchResults.legacy) LegacyResults{};benchResultsFull=false;}
        const bool prepared=benchJob.full?benchCycle.Prepare(benchJob.fullConfig):benchCompute.Prepare(benchJob.config);
        benchJob.setupCycles=DWT->CYCCNT-at;
        if(!prepared){if(benchJob.full)BenchFullFail(36,true,0);BenchFinish(BenchPhase::ABORTED);return;}
        if(benchJob.profile) {
            taskENTER_CRITICAL();
            BenchProfile::Reset();
            BenchProfile::SetTasks(ControlLoopUpdateHandle,controlLoopFixUpdateHandle,oledTaskHandle,rgbTaskHandle,usbServerTaskHandle,oledDisplayTaskHandle);
            taskEXIT_CRITICAL();
        }
        benchJob.wake=xTaskGetTickCount();
        if(benchJob.endurance){benchJob.longStartRtos=benchJob.wake;benchJob.longStartHal=benchJob.began=HAL_GetTick();
            benchLong.Reset(benchJob.clock,benchJob.seconds,DWT->CYCCNT);}
        benchJob.phase=BenchPhase::RUNNING;
    } else vTaskDelayUntil(&benchJob.wake,pdMS_TO_TICKS(20));
    if(benchJob.full){BenchFullPoll();return;}
    if(benchJob.endurance){
        const uint32_t elapsed=xTaskGetTickCount()-benchJob.longStartRtos;
        if(elapsed-benchLong.previousElapsed>1000U){benchLong.Fail(CartEndurance::CLOCK,benchLong.n);BenchFinish(BenchPhase::CLOCK_ERROR);return;}
        const auto guard=BenchCheckpoint(nullptr);
        if(guard!=CartBench::COMPLETE){
            const bool clockBad=!(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock;
            benchLong.Fail(guard==CartBench::CANCELLED?CartEndurance::CANCELLED:clockBad?CartEndurance::CLOCK:CartEndurance::ENVIRONMENT,benchLong.n);
            BenchFinish(guard==CartBench::CANCELLED?BenchPhase::CANCELLED:clockBad?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED);return;
        }
        if(benchLong.n==uint32_t(benchJob.seconds)*50U){
            if(elapsed>=uint32_t(benchJob.seconds)*1000U){
                benchLong.AdvanceClock(elapsed,HAL_GetTick()-benchJob.longStartHal,DWT->CYCCNT);
                BenchFinish(benchLong.error?BenchPhase::CLOCK_ERROR:BenchPhase::DONE);
            }
            return;
        }
        if(elapsed>=uint32_t(benchJob.seconds)*1000U){benchLong.Fail(CartEndurance::DEADLINE,benchLong.n);BenchFinish(BenchPhase::ABORTED);return;}
    }
    const CartBench::Hooks hooks{nullptr,BenchCycles,BenchTicks,BenchCheckpoint,benchJob.clock,
        benchJob.profile?BenchWorkBoundary:nullptr};
    CartBench::Sample sample{};
    if(!benchJob.profile)sample=benchCompute.Measure(benchJob.config,hooks);
    else {
        BenchFrame& frame=benchResults.legacy.frames[benchJob.count];frame=BenchFrame{};
        taskENTER_CRITICAL();
        frame.plannedRtos=benchJob.wake;frame.startRtos=xTaskGetTickCount();frame.startHal=HAL_GetTick();
        frame.startDwt=DWT->CYCCNT;
        const bool begun=BenchProfile::Begin(frame.startDwt,ControlLoopUpdateHandle);
        taskEXIT_CRITICAL();
        const bool longLate=benchJob.endurance&&uint32_t(frame.startRtos-frame.plannedRtos)>=pdMS_TO_TICKS(20);
        if(longLate)benchLong.Fail(frame.startRtos-benchJob.longStartRtos-benchLong.previousElapsed>1000U?CartEndurance::CLOCK:CartEndurance::DEADLINE,benchLong.n);
        if(begun&&!longLate)sample=benchCompute.Measure(benchJob.config,hooks);
        else if(longLate)sample.flag=CartBench::ABORTED;
        else {sample.flag=CartBench::ABORTED;frame.flags|=1U<<16;}
        taskENTER_CRITICAL();
        frame.endDwt=DWT->CYCCNT;
        if(begun) {
            const auto window=BenchProfile::End(frame.endDwt);frame.flags|=window.flags;
            std::memcpy(frame.resident,window.resident,sizeof(frame.resident));
            if(benchJob.oledProfile) {
                std::memcpy(frame.oledResident,window.oledResident,sizeof(frame.oledResident));
                std::memcpy(frame.oledElapsed,window.oledElapsed,sizeof(frame.oledElapsed));
                std::memcpy(frame.displayResident,window.displayResident,sizeof(frame.displayResident));
                std::memcpy(frame.displayElapsed,window.displayElapsed,sizeof(frame.displayElapsed));
            }
            if(window.cycles<sample.cycles)frame.flags|=1U<<17;
        } else BenchProfile::Reset();
        frame.endRtos=xTaskGetTickCount();frame.endHal=HAL_GetTick();
        taskEXIT_CRITICAL();
        if(benchJob.endurance){
            const uint32_t postStart=DWT->CYCCNT;
            BenchProfile::Window w{};w.cycles=frame.endDwt-frame.startDwt;w.flags=frame.flags;
            std::memcpy(w.resident,frame.resident,sizeof(w.resident));
            std::memcpy(w.oledResident,frame.oledResident,sizeof(w.oledResident));std::memcpy(w.oledElapsed,frame.oledElapsed,sizeof(w.oledElapsed));
            std::memcpy(w.displayResident,frame.displayResident,sizeof(w.displayResident));std::memcpy(w.displayElapsed,frame.displayElapsed,sizeof(w.displayElapsed));
            if(!(DWT->CTRL&DWT_CTRL_CYCCNTENA_Msk)||!(CoreDebug->DEMCR&CoreDebug_DEMCR_TRCENA_Msk)||SystemCoreClock!=benchJob.clock)
                benchLong.Fail(CartEndurance::CLOCK,benchLong.n);
            const auto error=benchLong.Observe(sample,w,frame.startRtos-benchJob.longStartRtos,frame.startHal-benchJob.longStartHal,
                frame.startDwt,frame.startRtos-frame.plannedRtos);
            if(benchLong.n==1||benchLong.n%500U==0)BenchLongStacks();
            BenchLongPublish();benchLong.Overhead(uint32_t(DWT->CYCCNT-postStart));
            if(error!=CartEndurance::NONE)BenchFinish(error==CartEndurance::BUDGET?BenchPhase::BUDGET:
                error==CartEndurance::CANCELLED?BenchPhase::CANCELLED:error==CartEndurance::CLOCK?BenchPhase::CLOCK_ERROR:BenchPhase::ABORTED);
            return;
        }
    }
    taskENTER_CRITICAL();
    benchResults.legacy.data[benchJob.count++]=sample;
    taskEXIT_CRITICAL();
    if(sample.flag==CartBench::BUDGET)BenchFinish(BenchPhase::BUDGET);
    else if(sample.flag==CartBench::CANCELLED)BenchFinish(BenchPhase::CANCELLED);
    else if(sample.flag!=CartBench::COMPLETE)BenchFinish(BenchPhase::ABORTED);
    else if(benchJob.count==benchJob.config.samples)BenchFinish(BenchPhase::DONE);
}
bool DummyRobot::HandleCartesianCommand(const char* cmd,unsigned n,StreamSink& response){
    while(n&&(*cmd==' '||*cmd=='\t'||*cmd=='\r'||*cmd=='\n')){++cmd;--n;}
    while(n&&(cmd[n-1]==' '||cmd[n-1]=='\t'||cmd[n-1]=='\r'||cmd[n-1]=='\n'))--n;
    if(Prefix(cmd,n,"#CBENCH")) {
        if(response.channelType!=StreamSink::CHANNEL_TYPE_USB){Respond(response,"error CBENCH usb_only");return true;}
        if(Equal(cmd,n,"#CBENCH FULL")){BenchFullStatus(response);return true;}
        if(Equal(cmd,n,"#CBENCH FULL_CANCEL")){
            if(!benchJob.full){Respond(response,"error CBENCH full_no_job");return true;}
            CartesianBenchmarkCancel(CartesianBenchmarkCancelSource::USB);BenchFullStatus(response);return true;
        }
        if(Prefix(cmd,n,"#CBENCH FREAD ")){
            float values[2];const uint32_t limits[2]={CartCycle::MaxSamples-1,4};
            if(!MotionValues::ParseCartesian(cmd+14,n-14,values,2,2)||!BenchUnsigned(values,limits,2)||values[1]<1){Respond(response,"error CBENCH full_read_syntax");return true;}
            BenchFullRead(response,unsigned(values[0]),unsigned(values[1]));return true;
        }
        if(Prefix(cmd,n,"#CBENCH FULL_RUN ")){
            float values[5];const uint32_t limits[5]={CartCycle::SCENARIO_COUNT-1,5,3,CartCycle::MaxSamples,20000};
            if(!MotionValues::ParseCartesian(cmd+17,n-17,values,5,5)||!BenchUnsigned(values,limits,5)||values[2]<1||values[3]<1||values[4]!=20000){Respond(response,"error CBENCH full_run_syntax");return true;}
            const CartCycle::Config config{uint8_t(values[0]),uint8_t(values[1]),uint8_t(values[2]),uint16_t(values[3]),uint32_t(values[4])};
            taskENTER_CRITICAL();
            const bool allowed=!BenchActive()&&!OledDiagnostic::Busy()&&!benchCalibrationEntry&&stopState==DISABLED&&!isEnabled&&!startPending&&!IsStopLocked()&&
                !IsMotionBusy()&&!CoreStreamActive()&&!CoreNativeOwned()&&!plannerLease&&!commandHandler.IsBusy()&&!IsCalibrationBusy()&&!IsProtectionParameterBusy()&&!rgbSaveBusy;
            if(allowed){benchJob.full=true;benchJob.fullConfig=config;benchJob.fullSummary=CartCycle::Summary{};
                benchJob.fullReason=benchJob.fullSource=0;benchJob.fullWouldStop=false;benchJob.fullFirst=0;
                benchJob.cancel=false;benchJob.count=0;benchJob.epoch=motionEpoch;benchJob.profile=benchJob.oledProfile=true;benchJob.oledMode=4;
                benchJob.endurance=false;benchJob.seconds=0;BenchProfile::OledSessionReset();
                benchJob.clock=benchJob.ctrl=benchJob.demcr=benchJob.probeCycles=benchJob.probeTicks=benchJob.setupCycles=0;
                if(!++benchJob.id)++benchJob.id;benchJob.phase=BenchPhase::PENDING;}
            taskEXIT_CRITICAL();
            if(allowed)BenchFullStatus(response);else Respond(response,"error CBENCH requires_idle_disabled");return true;
        }
        if(benchJob.full&&(Equal(cmd,n,"#CBENCH")||Equal(cmd,n,"#CBENCH PROFILE")||Prefix(cmd,n,"#CBENCH READ ")||Prefix(cmd,n,"#CBENCH PREAD ")||Prefix(cmd,n,"#CBENCH OREAD "))){
            Respond(response,"error CBENCH full_status_required");return true;
        }
        if(Equal(cmd,n,"#CBENCH")){BenchStatus(response);return true;}
        if(Equal(cmd,n,"#CBENCH PROFILE")){BenchProfileStatus(response);return true;}
        if(Equal(cmd,n,"#CBENCH OLED")){BenchOledStatus(response);return true;}
        if(Equal(cmd,n,"#CBENCH LONG")){BenchLongStatus(response);return true;}
        if(Equal(cmd,n,"#CBENCH LREAD")){BenchLongRead(response);return true;}
        if(Equal(cmd,n,"#CBENCH CANCEL")){CartesianBenchmarkCancel(CartesianBenchmarkCancelSource::USB);if(benchJob.full)BenchFullStatus(response);else if(benchJob.endurance)BenchLongStatus(response);else BenchStatus(response);return true;}
        if(Prefix(cmd,n,"#CBENCH PREAD ")) {
            float values[2];const uint32_t limits[2]={CartBench::MaxSamples-1,4};
            if(!MotionValues::ParseCartesian(cmd+14,n-14,values,2,2)||!BenchUnsigned(values,limits,2)||values[1]<1){Respond(response,"error CBENCH profile_read_syntax");return true;}
            taskENTER_CRITICAL();const bool allowed=!BenchActive()&&benchJob.profile;const unsigned count=benchJob.count;const uint32_t id=benchJob.id;taskEXIT_CRITICAL();
            const unsigned first=unsigned(values[0]),amount=unsigned(values[1]);
            if(!allowed||first+amount>count){Respond(response,"error CBENCH profile_read_range_or_busy");return true;}
            for(unsigned i=first;i<first+amount;++i) {
                BenchFrame f;unsigned work;
                taskENTER_CRITICAL();f=benchResults.legacy.frames[i];work=benchResults.legacy.data[i].work;taskEXIT_CRITICAL();
                Respond(response,"CBPROF S %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)f.plannedRtos,(unsigned long)f.startRtos,(unsigned long)f.endRtos);
                Respond(response,"CBPROF H %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)f.startHal,(unsigned long)f.endHal,(unsigned long)(f.startRtos-f.plannedRtos));
                Respond(response,"CBPROF D %lu,%u,%lu,%lu,%lu",(unsigned long)id,i,(unsigned long)f.startDwt,(unsigned long)f.endDwt,(unsigned long)(f.endDwt-f.startDwt));
                for(unsigned w=0;w<work&&w<BenchProfile::MaxWork;++w)
                    Respond(response,"CBPROF W %lu,%u,%u,%lu",(unsigned long)id,i,w,(unsigned long)f.workCycles[w]);
                for(unsigned t=0;t<BenchProfile::TaskCount;++t)
                    Respond(response,"CBPROF T %lu,%u,%u,%lu",(unsigned long)id,i,t,(unsigned long)f.resident[t]);
                Respond(response,"CBPROF E %lu,%u,%u,%lu",(unsigned long)id,i,work,(unsigned long)f.flags);
            }
            return true;
        }
        if(Prefix(cmd,n,"#CBENCH OREAD ")) {
            float values[2];const uint32_t limits[2]={CartBench::MaxSamples-1,4};
            if(!MotionValues::ParseCartesian(cmd+14,n-14,values,2,2)||!BenchUnsigned(values,limits,2)||values[1]<1){Respond(response,"error CBENCH oled_read_syntax");return true;}
            taskENTER_CRITICAL();const bool allowed=!BenchActive()&&benchJob.oledProfile;const unsigned count=benchJob.count;const uint32_t id=benchJob.id;taskEXIT_CRITICAL();
            const unsigned first=unsigned(values[0]),amount=unsigned(values[1]);
            if(!allowed||first+amount>count){Respond(response,"error CBENCH oled_read_range_or_busy");return true;}
            for(unsigned i=first;i<first+amount;++i) {
                BenchFrame f;taskENTER_CRITICAL();f=benchResults.legacy.frames[i];taskEXIT_CRITICAL();
                for(unsigned stage=0;stage<BenchProfile::OledStageCount;++stage) {
                    Respond(response,"CBOLED P %lu,%u,%u,%lu",(unsigned long)id,i,stage,(unsigned long)f.oledResident[stage]);
                    Respond(response,"CBOLED W %lu,%u,%u,%lu",(unsigned long)id,i,stage,(unsigned long)f.oledElapsed[stage]);
                    Respond(response,"CBOLED Q %lu,%u,%u,%lu",(unsigned long)id,i,stage,(unsigned long)f.displayResident[stage]);
                    Respond(response,"CBOLED V %lu,%u,%u,%lu",(unsigned long)id,i,stage,(unsigned long)f.displayElapsed[stage]);
                }
                Respond(response,"CBOLED E %lu,%u,%lu",(unsigned long)id,i,(unsigned long)f.flags);
            }
            return true;
        }
        if(Prefix(cmd,n,"#CBENCH READ ")) {
            float values[2];const uint32_t limits[2]={CartBench::MaxSamples-1,8};
            if(!MotionValues::ParseCartesian(cmd+13,n-13,values,2,2)||!BenchUnsigned(values,limits,2)||values[1]<1){Respond(response,"error CBENCH read_syntax");return true;}
            taskENTER_CRITICAL();const bool active=BenchActive();const unsigned count=benchJob.count;const uint32_t id=benchJob.id;taskEXIT_CRITICAL();
            const unsigned first=unsigned(values[0]),amount=unsigned(values[1]);
            if(active||first+amount>count){Respond(response,"error CBENCH read_range_or_busy");return true;}
            for(unsigned i=first;i<first+amount;++i) {
                CartBench::Sample s;taskENTER_CRITICAL();s=benchResults.legacy.data[i];taskEXIT_CRITICAL();
                Respond(response,"CBENCH DATA %lu,%u,%lu,%lu,%u,%u",(unsigned long)id,i,(unsigned long)s.cycles,(unsigned long)s.ticks,unsigned(s.work),unsigned(s.flag));
                Respond(response,"CBENCH RESULT %lu,%u,%u,%u,%u,%u,%u,%lu",(unsigned long)id,i,unsigned(s.fk),unsigned(s.ik),unsigned(s.valid),unsigned(s.selected),unsigned(s.failed),(unsigned long)s.checksum);
            }
            return true;
        }
        if(Prefix(cmd,n,"#CBENCH RUN ")||Prefix(cmd,n,"#CBENCH LONG ")) {
            const bool endurance=Prefix(cmd,n,"#CBENCH LONG ");
            float values[7]{};const uint32_t limits[7]={5,2,CartBench::MaxWork,CartBench::MaxSamples,20000,1,4};
            const unsigned fields=MotionValues::ParseCartesian(cmd+(endurance?13:12),n-(endurance?13:12),values,endurance?6:5,endurance?6:7);
            const uint32_t longLimits[6]={4,2,5,600,20000,4};
            const bool longValid=fields==6&&BenchUnsigned(values,longLimits,6)&&(values[0]==3||values[0]==4)&&values[2]==5&&values[3]>=1&&values[4]==20000&&(values[5]==2||values[5]==4);
            if((endurance&&!longValid)||(!endurance&&(!fields||!BenchUnsigned(values,limits,fields)||values[2]<1||values[3]<1||values[4]<100||
               (fields>=6&&values[5]!=0&&values[2]>BenchProfile::MaxWork)||
               (fields==7&&values[5]!=1)))){Respond(response,"error CBENCH run_syntax");return true;}
            const CartBench::Config config{uint8_t(values[0]),uint8_t(values[1]),uint16_t(values[2]),uint16_t(endurance?128:values[3]),uint32_t(values[4])};
            taskENTER_CRITICAL();
            const bool allowed=!BenchActive()&&!OledDiagnostic::Busy()&&!benchCalibrationEntry&&stopState==DISABLED&&!isEnabled&&!startPending&&!IsStopLocked()&&
                !IsMotionBusy()&&!CoreStreamActive()&&!CoreNativeOwned()&&!plannerLease&&!commandHandler.IsBusy()&&!IsCalibrationBusy()&&!IsProtectionParameterBusy()&&!rgbSaveBusy;
            if(allowed){benchJob.full=false;benchJob.config=config;benchJob.cancel=false;benchJob.count=0;benchJob.epoch=motionEpoch;benchJob.profile=endurance||(fields>=6&&values[5]!=0);
                benchJob.endurance=endurance;benchJob.seconds=endurance?uint16_t(values[3]):0;
                benchJob.longCount=benchJob.longElapsed=benchJob.longError=0;
                benchJob.oledProfile=endurance||fields==7;benchJob.oledMode=endurance?uint8_t(values[5]):fields==7?uint8_t(values[6]):0;BenchProfile::OledSessionReset();
                benchJob.clock=benchJob.ctrl=benchJob.demcr=benchJob.probeCycles=benchJob.probeTicks=benchJob.setupCycles=0;
                if(!++benchJob.id)++benchJob.id;benchJob.phase=BenchPhase::PENDING;}
            taskEXIT_CRITICAL();
            if(!allowed)Respond(response,"error CBENCH requires_idle_disabled");else if(endurance)BenchLongStatus(response);else BenchStatus(response);
            return true;
        }
        Respond(response,"error CBENCH syntax");return true;
    }
    if(CartesianBenchmarkActive()) {
        if(Equal(cmd,n,"!STOP")||Equal(cmd,n,"!DISABLE"))CartesianBenchmarkCancel(Equal(cmd,n,"!STOP")?CartesianBenchmarkCancelSource::SOFT_STOP:CartesianBenchmarkCancelSource::DISABLE);
        else if(!Equal(cmd,n,"#STACK")&&!Equal(cmd,n,"#MOTION_STATUS")&&!Equal(cmd,n,"#GET_CMDMODE")&&
                !Equal(cmd,n,"#FW_VERSION 0")&&!Equal(cmd,n,"#KIN_CONFIG")&&!Equal(cmd,n,"#TOOL")&&
                !Equal(cmd,n,"#TCPPOSE")&&!Equal(cmd,n,"#CARTESIAN_STATUS")) {
            Respond(response,"error CBENCH busy");return true;
        }
    }
    if(Equal(cmd,n,"#KIN_CONFIG")){
        Respond(response,"KIN v=2 xyz=mm abc=deg joints=deg rail=mm");
        Respond(response,"KIN base=207.3 db=0 arm=170 forearm=117");
        Respond(response,"KIN elbow=69.5 wrist=113 tool_default=114");
        Respond(response,"KIN rail_axis=+Y rail_min=-250 rail_max=250");
        Respond(response,"KIN TCP=@T x,y,z,a,b,c,speed,rail_auto");return true;}
    if(Equal(cmd,n,"#CARTESIAN_STATUS")){
        auto s=CoreNativeSnapshotRead();
        Respond(response,"CARTESIAN id=%lu owner=%u state=%u code=%u",(unsigned long)s.id,unsigned(s.owned),unsigned(s.state),unsigned(s.reason));
        Respond(response,"CARTESIAN n=%u c=%u e=%u free=%u",unsigned(s.next),unsigned(s.committed),unsigned(s.executed),unsigned(s.free));
        Respond(response,"CARTESIAN fault src=%u st=%u node=%u n=%u",unsigned(s.failure.source),unsigned(s.failure.state),unsigned(s.failure.node),unsigned(s.failure.sequence));
        Respond(response,"CARTESIAN fault age=%lu cmd=%u gap=%lu",(unsigned long)s.failure.age,unsigned(s.failure.command),(unsigned long)s.failure.gap);
        Respond(response,"CARTESIAN plan=%s work=%lu",lastPlan,(unsigned long)lastWork);
        Respond(response,"CARTESIAN us=%lu max_us=%lu",(unsigned long)lastPlanningUs,(unsigned long)maxPlanningUs);
        Respond(response,"CARTESIAN bound_mm=%.4f bound_deg=%.4f",lastBound*1000,lastRotation*57.2957795f);return true;}
    if(Equal(cmd,n,"#TOOL")){PrintTool(response);return true;}
    if(Equal(cmd,n,"#TCPPOSE")){
        DOF6Kinematic::Joint6D_t joint{};float rail;
        taskENTER_CRITICAL();joint=currentJoints;rail=currentRailPos;taskEXIT_CRITICAL();
        DOF6Kinematic::Pose6D_t flange{};if(!dof6Solver->SolveFK(joint,flange)){Respond(response,"error TCPPOSE FK_INVALID");return true;}
        auto tcp=NativeCartesian::Compose(flange,TcpTool::Get());tcp.Y+=rail*.001f;float p[6];NativeCartesian::ToXYZABC(tcp,p);
        Respond(response,"TCPPOSE world XYZ %.3f %.3f %.3f",p[0],p[1],p[2]);Respond(response,"TCPPOSE ABC %.3f %.3f %.3f",p[3],p[4],p[5]);return true;}
    if(Prefix(cmd,n,"#IKCHECK ")){
        if(CoreNativeOwned()){Respond(response,"error IKCHECK native_busy");return true;}
        float q[6];if(!MotionValues::ParseCartesian(cmd+9,n-9,q,6,6)){Respond(response,"error IKCHECK syntax");return true;}
        DOF6Kinematic::Joint6D_t joint{};std::memcpy(joint.a,q,sizeof(q));DOF6Kinematic::Pose6D_t p{};DOF6Kinematic::IKSolves_t solutions{};uint32_t began=micros();
        if(!dof6Solver->SolveFK(joint,p)||!dof6Solver->SolveIKMeters(p,joint,solutions)){Respond(response,"error IKCHECK NO_IK");return true;}
        unsigned valid=0;for(unsigned i=0;i<8;++i)valid+=solutions.solFlag[i][0]&&solutions.solFlag[i][1]&&solutions.solFlag[i][2];
        uint32_t elapsed=micros()-began;if(elapsed>maxIKCheckUs)maxIKCheckUs=elapsed;
        Respond(response,"IKCHECK wall_us=%lu max=%lu",(unsigned long)elapsed,(unsigned long)maxIKCheckUs);
        Respond(response,"IKCHECK valid=%u no_motion=1",valid);Respond(response,"IKCHECK flange XYZ %.3f %.3f %.3f",p.X*1000,p.Y*1000,p.Z*1000);return true;}
    const bool set=Prefix(cmd,n,"#TOOL_SET "),reset=Equal(cmd,n,"#TOOL_RESET"),save=Equal(cmd,n,"#TOOL_SAVE");
    if(!set&&!reset&&!save){if(Prefix(cmd,n,"#TOOL")){Respond(response,"error TOOL syntax");return true;}return false;}
    float values[6]{};if(set&&!MotionValues::ParseCartesian(cmd+10,n-10,values,6,6)){Respond(response,"error TOOL syntax");return true;}
    if(set&&(std::fabs(values[0])>500||std::fabs(values[1])>500||std::fabs(values[2])>500)){Respond(response,"error TOOL XYZ range=+-500mm");return true;}
    taskENTER_CRITICAL();
    bool allowed=!rgbSaveBusy&&!motionActive&&!CoreStreamActive()&&!plannerLease&&!commandHandler.IsBusy()&&!IsCalibrationBusy()&&!IsProtectionParameterBusy()&&!startPending;
    if(save)allowed=allowed&&stopState==DISABLED&&!isEnabled&&!stopFaultMask;
    else allowed=allowed&&(stopState==NORMAL||stopState==DISABLED);
    bool ok=false;if(allowed){if(save)rgbSaveBusy=true;else if(set)ok=TcpTool::Set(values[0],values[1],values[2],values[3],values[4],values[5]);else{TcpTool::Reset();ok=true;}}
    taskEXIT_CRITICAL();
    if(!allowed){Respond(response,save?"error TOOL save_requires_idle_disabled":"error TOOL idle_required");return true;}
    if(save){ok=TcpTool::Save();taskENTER_CRITICAL();rgbSaveBusy=false;taskEXIT_CRITICAL();}
    Respond(response,ok?(save?"ok TOOL saved":"ok TOOL applied"):"error TOOL invalid_or_flash_failure");PrintTool(response);return true;
}
bool DummyRobot::RunCartesian(const DOF6Kinematic::Pose6D_t& goal,const DOF6Kinematic::Pose6D_t& tool,bool automaticRail,uint32_t epoch,uint32_t id){
    // The serialized command task is the sole producer; direct fibre callers
    // must not block the USB task that is also responsible for emergency STOP.
    if(osThreadGetId()!=ControlLoopUpdateHandle)return false;
    taskENTER_CRITICAL();
    bool allowed=!plannerLease&&!motionActive&&!CoreStreamActive()&&!IsStopLocked()&&isEnabled&&epoch==motionEpoch&&commandMode<=COMMAND_CONTINUES_TRAJECTORY&&!rgbSaveBusy;
    if(allowed)plannerLease=true;taskEXIT_CRITICAL();if(!allowed)return false;
    uint32_t ownId=id?id:AllocateMotionTaskId();uint8_t code=CoreNativeBegin(epoch,ownId);bool success=false,started=false,settlingFailure=false;
    const char* reason="EXECUTOR_BEGIN";uint32_t began=HAL_GetTick();
    auto valid=[&](){return epoch==motionEpoch&&!IsStopLocked()&&isEnabled;};
    auto wait=[&](){CoreNativeHeartbeat(epoch);osDelay(1);return valid();};
    auto stage=[&](const char* name){if(usbStreamOutputPtr)Respond(*usbStreamOutputPtr,"CARTESIAN id=%lu stage=%s",(unsigned long)ownId,name);};
    NativeCartesian::Request request{};NativeCartesian::Limits limits{};StreamProtocol::Record record{};uint32_t trajectoryStartedAt=0;
    if(code==StreamProtocol::OK){
        stage("OPEN");
        while(valid()&&HAL_GetTick()-began<4000){auto s=CoreNativeSnapshotRead();if(s.reason){code=s.reason;break;}if(s.state==CoreStreamEngine::OPEN)break;wait();}
        auto s=CoreNativeSnapshotRead();if(s.state!=CoreStreamEngine::OPEN||!valid())reason="EXECUTOR_OPEN";
        else{
            bool held=true;for(uint8_t node=1;node<=7&&valid();++node){code=CoreNativeReadHeld(node,epoch);uint32_t at=HAL_GetTick();
                while(valid()&&HAL_GetTick()-at<1000&&!CoreNativeHeld(node,request.seed[node-1],epoch))wait();
                if(code!=StreamProtocol::OK||!CoreNativeHeld(node,request.seed[node-1],epoch)){held=false;break;}}
            if(!held||!valid())reason="EXECUTOR_HELD";
            else{
                request.goal=goal;request.tool=tool;request.automaticRail=automaticRail;request.exactSeed=true;request.speed=jointSpeed;
                for(unsigned j=0;j<7;++j){const auto* m=motorJ[j+1];float direction=m->inverseDirection?-1.f:1.f;
                    limits.origin[j]=j==6?0:initPose.a[j];limits.stepsPerUnit[j]=direction*(j==6?51200.f/RAIL_MM_PER_CIRCLE:51200.f*m->reduction/360.f);
                    request.start[j]=limits.origin[j]+request.seed[j]/limits.stepsPerUnit[j];limits.lower[j]=m->angleLimitMin;limits.upper[j]=m->angleLimitMax;
                    limits.velocity[j]=j==6?20.f:40.f;float physical=jointAccRuntime[j+1]*(j==6?RAIL_MM_PER_CIRCLE:360.f/m->reduction);
                    limits.acceleration[j]=std::fmin(j==6?40.f:80.f,physical>0?physical:40.f);limits.jerk[j]=j==6?3000.f:20000.f;}
                stage("PRECHECK");RobotProgress progress(epoch);uint32_t at=micros();auto result=planner.Build(*dof6Solver,request,limits,progress);
                if(progress.budgetExpired&&result==NativeCartesian::CANCELLED)result=NativeCartesian::BUDGET;
                lastPlanningUs=micros()-at;if(lastPlanningUs>maxPlanningUs)maxPlanningUs=lastPlanningUs;lastWork=planner.Work();lastPlan=NativeCartesian::Name(result);
                lastBound=planner.PositionBound();lastRotation=planner.RotationBound();reason=lastPlan;
                if(result==NativeCartesian::OK&&valid()){
                    taskENTER_CRITICAL();bool publishing=valid()&&CoreNativeOwned()&&CoreNativeSnapshotRead().id==ownId;
                    if(publishing){++motionCompletionGeneration;motionTaskId=ownId;motionTaskEpoch=epoch;motionMode=commandMode;motionKind="MOVEL";motionPhase=MOTION_DISPATCHING;
                    motionFailureReason=nullptr;nativePhase=NATIVE_EXECUTING;motionActive=true;motionDispatchPending=false;motionStartedAt=trajectoryStartedAt=HAL_GetTick();motionCompletedAt=0;motionProgressMask=jointsStateFlag=0;
                    targetRailPos=planner.RailTarget();std::memcpy(targetJoints.a,planner.GoalJoints(),sizeof(targetJoints.a));}taskEXIT_CRITICAL();
                    if(publishing){
                    if(usbStreamOutputPtr)Respond(*usbStreamOutputPtr,"CARTESIAN id=%lu n=%u rail=%.2f",(unsigned long)ownId,planner.Segments(),planner.RailTarget());
                    unsigned sequence=0;bool runningPublished=false;stage("BUFFER");uint32_t executionAt=HAL_GetTick();reason="EXECUTOR_FAULT";
                    while(valid()&&HAL_GetTick()-executionAt<uint32_t(planner.Duration()*1000)+10000){s=CoreNativeSnapshotRead();if(s.reason){code=s.reason;break;}
                        if(s.free&&sequence<=planner.Segments()){if(!planner.Record(sequence,s.session,record)){reason="RECORD_INVALID";break;}
                            code=CoreNativeSubmit(record,epoch);if(code==StreamProtocol::OK)++sequence;else if(code!=StreamProtocol::BUSY)break;}
                        if(!started&&s.state==CoreStreamEngine::ARMED){code=CoreNativeStart(epoch);if(code!=StreamProtocol::OK)break;started=true;stage("START_PENDING");}
                        if(started&&!runningPublished&&(s.state==CoreStreamEngine::RUNNING||s.state==CoreStreamEngine::DRAINING)){taskENTER_CRITICAL();runningPublished=valid()&&motionTaskId==ownId;if(runningPublished)motionPhase=MOTION_RUNNING;taskEXIT_CRITICAL();if(runningPublished)stage("RUNNING");}
                        if(started&&s.state==CoreStreamEngine::IDLE){success=true;break;}wait();
                    }
                    }else reason="CANCELLED";
                }
            }
        }
    }
    auto primary=CoreNativeSnapshotRead();
    if(primary.id==ownId&&primary.epoch==epoch&&primary.reason&&primary.reason!=StreamProtocol::ABORTED){code=primary.reason;reason="EXECUTOR_FAULT";}
    else if(!valid())reason="CANCELLED";
    // Retire the exact session before ordinary completion queries or another
    // command can take ownership. STOP changes epoch but cleanup uses its lease.
    if(CoreNativeOwned()){
        if(!success&&started&&valid())RequestSoftStop();
        if(!success&&started)CoreStreamCancel();
        CoreNativeClose(epoch);uint32_t closeAt=HAL_GetTick();
        while(CoreStreamActive()&&HAL_GetTick()-closeAt<1800){CoreNativeClose(epoch);osDelay(1);}
        if(CoreStreamActive()){success=false;reason="EXECUTOR_CLOSE";if(valid())RequestSoftStop();}
        CoreNativeRelease(epoch);
    }
    auto retired=CoreNativeSnapshotRead();
    if(retired.id==ownId&&retired.epoch==epoch&&retired.reason&&retired.reason!=StreamProtocol::ABORTED){code=retired.reason;if(retired.failure.source==CoreStreamEngine::SOURCE_CLOSE)reason="EXECUTOR_CLOSE";}
    else if(!valid()){reason="CANCELLED";if(!code)code=StreamProtocol::ABORTED;}
    if(success&&valid()){
        // END/COMPLETED proves reference execution. Existing tagged completion
        // rounds still prove physical settling without resending ordinary 0x07.
        float expected[7];for(unsigned j=0;j<7;++j)if(!ExactMotorR(record.axes[j].q,expected[j])){success=false;reason="ENDPOINT_FLOAT";break;}
        taskENTER_CRITICAL();success=success&&valid()&&motionTaskId==ownId&&!CoreNativeOwned();if(success){motionTxOkMask=motionTxHistoryMask=0xFE;motionTxPendingMask=0;motionTargetReadyMask=0;motionConfirmedMask=0;motionFailureReason=nullptr;
        for(unsigned j=0;j<7;++j){motionMotorTargets[j+1]=expected[j];motionTxAttempts[j+1]=0;motionFirstReadyMs[j+1]=MOTION_TIME_MISSING;motionReplyFlags[j+1]=0;motionRetryAt[j+1]=HAL_GetTick()+10000;}
        motionStartedAt=HAL_GetTick();nativePhase=NATIVE_SETTLING;BeginMotionRound();}taskEXIT_CRITICAL();if(success)stage("SETTLING");uint32_t at=HAL_GetTick();
        while(success&&valid()&&motionActive&&HAL_GetTick()-at<3000)osDelay(2);
        taskENTER_CRITICAL();success=success&&valid()&&motionTaskId==ownId&&motionPhase==MOTION_COMPLETED;
        if(success)motionStartedAt=trajectoryStartedAt;
        else if(motionTaskId==ownId&&motionPhase==MOTION_FAILED&&motionFailureReason){
            settlingFailure=true;reason=motionFailureReason;code=StreamProtocol::DEADLINE;
        }
        taskEXIT_CRITICAL();
        if(!success&&!settlingFailure){
            if(!valid()){reason="CANCELLED";if(!code)code=StreamProtocol::ABORTED;}
            else{reason="TCP_SETTLE_TIMEOUT";RequestSoftStop();}
        }
    }
    taskENTER_CRITICAL();plannerLease=false;success=success&&valid()&&motionTaskId==ownId&&motionPhase==MOTION_COMPLETED;
    if(!settlingFailure&&!valid()&&(code==StreamProtocol::OK||code==StreamProtocol::ABORTED)&&
       std::strcmp(reason,"TCP_SETTLE_TIMEOUT")&&std::strcmp(reason,"EXECUTOR_CLOSE")){
        reason="CANCELLED";code=StreamProtocol::ABORTED;
    }
    if(motionTaskId==ownId)nativePhase=NATIVE_NONE;
    if(!success&&epoch==motionEpoch){motionActive=motionDispatchPending=false;motionTaskId=ownId;motionTaskEpoch=epoch;motionKind="MOVEL";motionMode=commandMode;motionPhase=MOTION_FAILED;motionCompletedAt=HAL_GetTick();motionFailureReason=reason;}
    taskEXIT_CRITICAL();
    if(!success&&usbStreamOutputPtr){Respond(*usbStreamOutputPtr,"CARTESIAN id=%lu reason=%s code=%u",(unsigned long)ownId,reason,unsigned(code));}
    if(success)stage("DONE");return success;
}
