"""Compile and execute the real OLED thread and diagnostic wrappers with stubs.

Only host C++ is compiled. No serial, ST-Link, target build, or motor API exists
in this harness. Synthetic ticks/cycles verify ordering and skip restoration;
they do not estimate F405 timing or instrumentation overhead. Every invocation
freezes inputs, extraction, commands, failure logs, and hashes in artifacts.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
INST = Path("firmware_new/dummy-ref-core-fw/Robot/instances")
MAIN = Path("firmware_new/dummy-ref-core-fw/UserApp/main.cpp")
INPUTS = [MAIN, INST / "bench_profile.h", INST / "bench_profile.cpp",
          Path("tools/tests/test_oled_benchmark_thread.py")]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def function(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


HARNESS = r'''
#include "bench_profile.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);std::exit(1);}} while(0)
using TickType_t=uint32_t;
using osThreadId_t=void*;
using TaskHandle_t=void*;
using StackType_t=uint32_t;
struct StaticTask_t {uint32_t opaque=0;};
enum osPriority_t {osPriorityError=-1,osPriorityBelowNormal=16,osPriorityNormal=24};
enum osStatus_t {osOK=0,osError=-1};
constexpr unsigned pdTRUE=1,portMAX_DELAY=0xffffffffU;
static uint32_t ticks=0,critical=0,entries=0,frames=0,frameLimit=0;
struct FakeDwt {uint32_t CYCCNT=0;} fakeDwt;
FakeDwt* DWT=&fakeDwt;
uint32_t SystemCoreClock=168000000;
static std::vector<std::string> events;
static std::vector<std::string> texts;
static std::vector<uint32_t> actualTicks;
static std::function<void(const char*)> serviceHook;
static bool injectPreemption=false;
static void* const command=reinterpret_cast<void*>(1);
static void* const realtime=reinterpret_cast<void*>(2);
static void* const oledTask=reinterpret_cast<void*>(3);
static void* const displayTask=reinterpret_cast<void*>(6);
static void* currentTask=command;
static bool workerOnStack=false,displayOnStack=false,autoWorker=false,autoDisplay=false,inSend=false;
static bool failNormalPriority=false,failBelowPriority=false,wrongReadback=false;
static osPriority_t displayPriority=osPriorityNormal;
static unsigned notifications=0,notifyTakes=0,notifyLimit=0,prioritySets=0;
static TickType_t lastAutoWorker=0;
static void PumpWorker();
static void PumpDisplay();
static void Advance(uint32_t cycles){DWT->CYCCNT+=cycles;}
static void AdvanceTicks(uint32_t count){ticks+=count;Advance(count*(SystemCoreClock/1000));}
static void Event(const char* name,uint32_t cycles=11){
    CHECK(critical==0);events.emplace_back(name);Advance(cycles);
    if(injectPreemption&&benchProfileWindowActive){
        BenchProfile::SwitchAt(DWT->CYCCNT,realtime);Advance(17);
        BenchProfile::SwitchAt(DWT->CYCCNT,currentTask);
    }
    if(serviceHook)serviceHook(name);
}
static void EnterCritical(){CHECK(critical<8);++critical;++entries;}
static void ExitCritical(){CHECK(critical>0);--critical;}
#define taskENTER_CRITICAL() EnterCritical()
#define taskEXIT_CRITICAL() ExitCritical()
#define pdMS_TO_TICKS(x) (x)
static TickType_t xTaskGetTickCount(){return ticks;}
static osStatus_t osThreadSetPriority(osThreadId_t task,osPriority_t priority){
    CHECK(critical==0);CHECK(task==displayTask);++prioritySets;
    if((priority==osPriorityNormal&&failNormalPriority)||(priority==osPriorityBelowNormal&&failBelowPriority))return osError;
    displayPriority=priority;return osOK;
}
static osPriority_t osThreadGetPriority(osThreadId_t task){
    CHECK(critical==0);CHECK(task==displayTask);return wrongReadback?osPriorityNormal:displayPriority;
}
static uint32_t osThreadGetStackSpace(osThreadId_t task){CHECK(critical==0);CHECK(task==displayTask);return 1536;}
static void xTaskNotifyGive(TaskHandle_t task){CHECK(critical==0);CHECK(task==displayTask);++notifications;}
struct ThreadDone {};
static uint32_t ulTaskNotifyTake(unsigned clear,uint32_t wait){
    CHECK(critical==0);CHECK(clear==pdTRUE);CHECK(wait==portMAX_DELAY);
    if(notifyTakes++==notifyLimit||!notifications)throw ThreadDone{};
    const unsigned n=notifications;notifications=0;return n;
}
static void osDelay(uint32_t delay){
    CHECK(critical==0);CHECK(delay>0);
    for(unsigned i=0;i<delay;++i){
        AdvanceTicks(1);
        if(autoWorker&&!workerOnStack&&TickType_t(ticks-lastAutoWorker)>=20){lastAutoWorker=ticks;PumpWorker();}
        if(autoDisplay&&!displayOnStack&&notifications){notifications=0;PumpDisplay();}
    }
}
static void vTaskDelayUntil(TickType_t* wake,TickType_t period){
    CHECK(critical==0);CHECK(period==20);
    const unsigned frame=frames++;
    if(frame==frameLimit)throw ThreadDone{};
    const uint32_t previous=ticks;
    *wake+=period;ticks=frame<actualTicks.size()?actualTicks[frame]:*wake;
    Advance((ticks-previous)*(SystemCoreClock/1000));
    events.emplace_back("delay");
}
static uint32_t micros(){return 1+DWT->CYCCNT/(SystemCoreClock/1000000);}
struct FakeImu {struct {float ax=1,ay=2;}data;void Update(bool enabled){CHECK(enabled);Event("imu",101);}}mpu6050;
struct FakeRobot {
    struct {float a[6]{};}currentJoints;
    struct {float X=0,Y=0,Z=0,A=0,B=0,C=0;}currentPose6D;
    float currentRailPos=0;unsigned jointsStateFlag=0;int commandMode=2;
    bool enabled=false;
    struct Handler {void PollCancelled(){Event("cancel",23);}} commandHandler;
    void PollPidPrint(){Event("pid",13);}
    void PollStopPrint(){Event("stop",19);}
    void PollProtectionPrint(){Event("protection",29);}
    void PollMotorAccelerationSync(){Event("acceleration",31);}
    bool IsEnabled(){return enabled;}
}dummy;
static void CoreStreamPrint(){Event("stream",37);}
static const void* u8g2_font_5x8_tr=nullptr;
static const void* u8g2_font_10x20_tr=nullptr;
static const void* u8g2_font_6x10_tr=nullptr;
struct FakeOled {
    void clearBuffer(){Event("clear");}
    void setFont(const void*){Event("font");}
    void setCursor(int,int){Event("cursor");}
    template<class... A>void printf(const char* format,A... args){
        char text[256];std::snprintf(text,sizeof(text),format,args...);texts.emplace_back(text);Event("printf");
    }
    void drawBox(int,int,int,int){Event("box");}
    void setDrawColor(int){Event("color");}
    void sendBuffer(){CHECK(!inSend);inSend=true;Event("send",1009);inSend=false;}
}oled;
#include "thread_fragment.inc"

static void SetCurrent(void* task){if(benchProfileWindowActive)BenchProfile::SwitchAt(DWT->CYCCNT,task);currentTask=task;}
static void PumpWorker(){
    CHECK(!workerOnStack);workerOnStack=true;void* prior=currentTask;SetCurrent(oledTask);
    OledWorkerIteration(ticks);SetCurrent(prior);workerOnStack=false;
}
static void PumpDisplay(){
    CHECK(!displayOnStack);displayOnStack=true;void* prior=currentTask;SetCurrent(displayTask);
    OledDisplayIteration();SetCurrent(prior);displayOnStack=false;
}

static unsigned Count(const char* name){unsigned n=0;for(const auto& e:events)n+=e==name;return n;}
static void CheckServiceOrder(unsigned frame,bool draw){
    unsigned index=0;for(unsigned seen=0;index<events.size();++index)
        if(events[index]=="delay"&&seen++==frame)break;
    CHECK(index<events.size());++index;
    for(const char* name:{"imu","pid","stop","cancel","protection","acceleration","stream"}){
        CHECK(index<events.size());CHECK(events[index++]==name);
    }
    if(draw){CHECK(index<events.size());CHECK(events[index]=="clear");}
    else CHECK(index==events.size()||events[index]=="delay");
}
static void Reset(){
    BenchProfile::Reset();BenchProfile::OledSessionReset();
    BenchProfile::SetTasks(command,realtime,oledTask,reinterpret_cast<void*>(4),reinterpret_cast<void*>(5),displayTask);
    ticks=critical=entries=frames=frameLimit=0;fakeDwt={};events.clear();texts.clear();actualTicks.clear();serviceHook={};injectPreemption=false;dummy={};
    currentTask=command;workerOnStack=displayOnStack=autoWorker=autoDisplay=inSend=false;
    failNormalPriority=failBelowPriority=wrongReadback=false;displayPriority=osPriorityNormal;
    notifications=notifyTakes=notifyLimit=prioritySets=0;lastAutoWorker=0;
    oledTaskHandle=oledTask;oledDisplayTaskHandle=displayTask;oledMailbox={};oledRuntime={};
    oledRequestedMode=oledActiveMode=0;oledDisplayBusy=oledMailboxPending=false;oledRestoreAllowed=true;
    oledLastRenderMicros=0;oledSendMonitoring=false;oledLastCompletedSendTick=0;
}
static void Run(unsigned count){
    frameLimit=count;workerOnStack=true;void* prior=currentTask;SetCurrent(oledTask);
    try{ThreadOledUpdate(nullptr);CHECK(false);}catch(const ThreadDone&){}
    SetCurrent(prior);workerOnStack=false;CHECK(critical==0);
}
static BenchProfile::Window WindowRun(bool skip,bool preempt=false){
    BenchProfile::OledSessionStart(skip,SystemCoreClock);
    CHECK(BenchProfile::Begin(DWT->CYCCNT,oledTask));injectPreemption=preempt;
    Run(2);return BenchProfile::End(DWT->CYCCNT);
}
static void CheckWindow(const BenchProfile::Window& w){
    CHECK(w.flags==0);uint32_t resident=0,elapsed=0;
    for(unsigned i=0;i<BenchProfile::OledStageCount;++i){
        CHECK(w.oledResident[i]<=w.oledElapsed[i]);resident+=w.oledResident[i];elapsed+=w.oledElapsed[i];
    }
    CHECK(resident==w.resident[2]);CHECK(elapsed==w.cycles);
    resident=elapsed=0;
    for(unsigned i=0;i<BenchProfile::OledStageCount;++i){
        CHECK(w.displayResident[i]<=w.displayElapsed[i]);resident+=w.displayResident[i];elapsed+=w.displayElapsed[i];
    }
    CHECK(resident==w.resident[5]);CHECK(elapsed==w.cycles);
    uint32_t total=0;for(auto value:w.resident)total+=value;CHECK(total==w.cycles);
}
static void StartSplit(unsigned mode){
    CHECK(mode==3||mode==4);autoWorker=true;lastAutoWorker=ticks;
    CHECK(OledDiagnostic::Start(mode));CHECK(oledRequestedMode==mode);CHECK(oledActiveMode==mode);
    CHECK(OledDiagnostic::Busy());CHECK(oledRuntime.restored==0);
    CHECK(displayPriority==(mode==4?osPriorityBelowNormal:osPriorityNormal));
    CHECK(oledRuntime.priority==unsigned(displayPriority));CHECK(critical==0);
    BenchProfile::OledSessionStart(mode,SystemCoreClock,100);
}
static bool HasText(const char* text){for(const auto& value:texts)if(value==text)return true;return false;}
int main(){
    Reset();Run(2);CHECK(entries>0);CHECK(Count("imu")==2);CHECK(Count("send")==2);
    CheckServiceOrder(0,true);CheckServiceOrder(1,true);CHECK(BenchProfile::OledSnapshot().begun==0);
    Reset();dummy.enabled=true;Run(1);CHECK(entries>0);CHECK(Count("send")==1);CheckServiceOrder(0,true);
    Reset();auto normal=WindowRun(false,true);CheckWindow(normal);
    CheckServiceOrder(0,true);CheckServiceOrder(1,true);CHECK(Count("send")==2);
    auto s=BenchProfile::OledSnapshot();CHECK(s.begun==2);CHECK(s.ended==2);CHECK(s.skipped==0);
    CHECK(normal.oledResident[BenchProfile::OLED_IMU]==202);
    CHECK(normal.oledResident[BenchProfile::OLED_SERVICE]==2*(13+19+23+29+31+37));
    CHECK(normal.oledResident[BenchProfile::OLED_DRAW]>0);CHECK(normal.oledResident[BenchProfile::OLED_SEND]==2018);
    CHECK(normal.resident[1]>0);CHECK(normal.oledElapsed[BenchProfile::OLED_IMU]>normal.oledResident[BenchProfile::OLED_IMU]);
    Reset();dummy.enabled=true;auto skipped=WindowRun(true,true);CheckWindow(skipped);
    CheckServiceOrder(0,false);CheckServiceOrder(1,false);CHECK(Count("send")==0);CHECK(Count("clear")==0);
    s=BenchProfile::OledSnapshot();CHECK(s.begun==2);CHECK(s.ended==2);CHECK(s.skipped==2);
    CHECK(skipped.oledResident[BenchProfile::OLED_IMU]==normal.oledResident[BenchProfile::OLED_IMU]);
    CHECK(skipped.oledResident[BenchProfile::OLED_SERVICE]==normal.oledResident[BenchProfile::OLED_SERVICE]);
    for(unsigned i:{BenchProfile::OLED_DRAW,BenchProfile::OLED_SEND})CHECK(skipped.oledResident[i]==0&&skipped.oledElapsed[i]==0);
    // A token admitted as skip remains skip for that frame. Session shutdown
    // invalidates accounting immediately and cannot keep later frames hidden.
    Reset();BenchProfile::OledSessionStart(true,SystemCoreClock);bool changed=false;
    serviceHook=[&](const char* name){if(!changed&&std::string(name)=="stop"){changed=true;BenchProfile::OledSessionStop();}};
    Run(2);CHECK(changed);CheckServiceOrder(0,false);CheckServiceOrder(1,true);CHECK(Count("send")==1);CHECK(!benchOledSessionActive);
    s=BenchProfile::OledSnapshot();CHECK(s.begun==1);CHECK(s.ended==0);CHECK(s.skipped==0);
    // A new normal session cannot be mutated by the old skip token's later
    // stage/end calls; the next frame is admitted against the new session.
    Reset();BenchProfile::OledSessionStart(true,SystemCoreClock);changed=false;
    serviceHook=[&](const char* name){if(!changed&&std::string(name)=="pid"){changed=true;BenchProfile::OledSessionStop();BenchProfile::OledSessionStart(false,SystemCoreClock);}};
    Run(2);CheckServiceOrder(0,false);CheckServiceOrder(1,true);CHECK(Count("send")==1);
    s=BenchProfile::OledSnapshot();CHECK(s.begun==1);CHECK(s.ended==1);CHECK(s.skipped==0);
    // Beginning a diagnostic halfway through an ordinary frame does not skip
    // or attribute that frame as a newly begun diagnostic frame.
    Reset();changed=false;
    serviceHook=[&](const char* name){if(!changed&&std::string(name)=="stream"){changed=true;BenchProfile::OledSessionStart(true,SystemCoreClock);}};
    Run(2);CheckServiceOrder(0,true);CheckServiceOrder(1,false);CHECK(Count("send")==1);
    s=BenchProfile::OledSnapshot();CHECK(s.begun==1);CHECK(s.ended==1);CHECK(s.skipped==1);
    // Rate limiting changes only draw/send eligibility. The real extracted
    // loop still runs every 20 ms and executes every IMU/service in order.
    Reset();BenchProfile::OledSessionStart(2,SystemCoreClock,100);
    CHECK(BenchProfile::Begin(DWT->CYCCNT,oledTask));injectPreemption=true;
    Run(11);auto limited=BenchProfile::End(DWT->CYCCNT);CheckWindow(limited);
    CHECK(Count("imu")==11);CHECK(Count("acceleration")==11);CHECK(Count("stream")==11);
    CHECK(Count("clear")==3);CHECK(Count("send")==3);
    for(unsigned i=0;i<11;++i)CheckServiceOrder(i,i%5==0);
    s=BenchProfile::OledSnapshot();CHECK(s.begun==11);CHECK(s.ended==11);
    CHECK(s.renderBegun==3);CHECK(s.skipped==8);CHECK(s.rateLimited==8);
    CHECK(s.minRenderGapTicks==100);CHECK(s.maxRenderGapTicks==100);
    CHECK(limited.oledResident[BenchProfile::OLED_IMU]==11*101);
    CHECK(limited.oledResident[BenchProfile::OLED_SERVICE]==11*(13+19+23+29+31+37));
    CHECK(limited.oledResident[BenchProfile::OLED_SEND]==3*1009);
    // Actual late opportunities, rather than the old planned wake times,
    // determine admission: a long stall permits one frame, never a burst.
    Reset();actualTicks={20,390,391,410,490};BenchProfile::OledSessionStart(2,SystemCoreClock,100);
    Run(5);for(unsigned i=0;i<5;++i)CheckServiceOrder(i,i==0||i==1||i==4);
    CHECK(Count("send")==3);CHECK(Count("imu")==5);
    s=BenchProfile::OledSnapshot();CHECK(s.renderBegun==3);CHECK(s.rateLimited==2);
    CHECK(s.minRenderGapTicks==100);CHECK(s.maxRenderGapTicks==370);
    // Tick wrap does not reset the cooldown or produce duplicate draws.
    Reset();ticks=0xfffffff0U;BenchProfile::OledSessionStart(2,SystemCoreClock,100);
    Run(11);CHECK(Count("send")==3);CHECK(Count("imu")==11);
    s=BenchProfile::OledSnapshot();CHECK(s.minRenderGapTicks==100);CHECK(s.maxRenderGapTicks==100);
    // Termination during a rate-suppressed frame preserves that token's
    // decision; the next ordinary frame immediately returns to old rendering.
    Reset();BenchProfile::OledSessionStart(2,SystemCoreClock,100);unsigned polls=0;
    serviceHook=[&](const char* name){if(std::string(name)=="pid"&&++polls==3)BenchProfile::OledSessionStop();};
    Run(4);CHECK(!benchOledSessionActive);CHECK(Count("imu")==4);CHECK(Count("send")==2);
    CheckServiceOrder(0,true);CheckServiceOrder(1,false);CheckServiceOrder(2,false);CheckServiceOrder(3,true);
    s=BenchProfile::OledSnapshot();CHECK(s.begun==3);CHECK(s.ended==2);
    CHECK(s.renderBegun==1);CHECK(s.skipped==1);CHECK(s.rateLimited==1);
    CHECK(critical==0);
    // Inactive display never draws, including the real notification thread.
    Reset();PumpDisplay();CHECK(Count("clear")==0);CHECK(Count("send")==0);
    notifications=9;notifyLimit=1;displayOnStack=true;SetCurrent(displayTask);
    try{ThreadOledDisplay(nullptr);CHECK(false);}catch(const ThreadDone&){}
    displayOnStack=false;SetCurrent(command);CHECK(notifications==0);CHECK(Count("send")==0);
    // Both diagnostic priorities preserve service cadence and keep render out
    // of the worker. Every lane and task bucket balances independently.
    for(unsigned mode:{3U,4U}){
        Reset();StartSplit(mode);CHECK(Count("imu")==1);CHECK(Count("send")==0);
        CHECK(OledDiagnostic::Snapshot().displayStackFree==1536);
        CHECK(BenchProfile::Begin(DWT->CYCCNT,command));injectPreemption=true;
        PumpDisplay();CHECK(Count("send")==1);
        for(unsigned i=0;i<10;++i){AdvanceTicks(20);PumpWorker();PumpDisplay();}
        const auto splitWindow=BenchProfile::End(DWT->CYCCNT);CheckWindow(splitWindow);
        CHECK(splitWindow.oledResident[BenchProfile::OLED_DRAW]==0);
        CHECK(splitWindow.oledResident[BenchProfile::OLED_SEND]==0);
        CHECK(splitWindow.displayResident[BenchProfile::OLED_IMU]==0);
        CHECK(splitWindow.displayResident[BenchProfile::OLED_SERVICE]==0);
        CHECK(splitWindow.displayResident[BenchProfile::OLED_DRAW]>0);
        CHECK(splitWindow.displayResident[BenchProfile::OLED_SEND]==3*1009);
        auto workerStats=BenchProfile::OledSnapshot();auto displayStats=BenchProfile::OledDisplaySnapshot();
        CHECK(workerStats.begun==10);CHECK(workerStats.ended==10);CHECK(workerStats.skipped==10);
        CHECK(workerStats.renderBegun==0);CHECK(workerStats.maxLoopGapTicks==20);
        CHECK(displayStats.begun==11);CHECK(displayStats.ended==11);CHECK(displayStats.renderBegun==3);
        CHECK(displayStats.skipped==8);CHECK(displayStats.rateLimited==8);
        CHECK(displayStats.minRenderGapTicks==100);CHECK(displayStats.maxRenderGapTicks==100);
        CHECK(Count("imu")==11);CHECK(Count("stream")==11);CHECK(Count("send")==3);
        BenchProfile::OledSessionStop();CHECK(OledDiagnostic::Stop());CHECK(!OledDiagnostic::Busy());
        CHECK(displayPriority==osPriorityNormal);CHECK(oledRuntime.restored==1);
        const unsigned sends=Count("send");AdvanceTicks(20);PumpWorker();CHECK(Count("send")==sends+1);
    }
    // A busy/missed display consumes a single newest mailbox value. A worker
    // publishing while draw is in flight cannot change that local frame.
    Reset();StartSplit(4);dummy.currentRailPos=1;AdvanceTicks(20);PumpWorker();
    dummy.currentRailPos=2;AdvanceTicks(20);PumpWorker();CHECK(oledRuntime.overwritten==2);
    AdvanceTicks(30);bool publishedDuringDraw=false;
    serviceHook=[&](const char* name){
        if(!publishedDuringDraw&&std::string(name)=="printf"){
            publishedDuringDraw=true;CHECK(oledDisplayBusy);dummy.currentRailPos=9;
            AdvanceTicks(20);PumpWorker();CHECK(oledMailboxPending);
        }
    };
    PumpDisplay();CHECK(publishedDuringDraw);CHECK(HasText("rail:2.00mm"));CHECK(!HasText("rail:9.00mm"));
    CHECK(oledRuntime.maxSnapshotAgeTicks==30);CHECK(Count("send")==1);CHECK(oledMailboxPending);
    PumpDisplay();CHECK(!oledMailboxPending);CHECK(Count("send")==1);
    serviceHook={};dummy.currentRailPos=7;AdvanceTicks(100);PumpWorker();PumpDisplay();
    CHECK(Count("send")==2);CHECK(HasText("rail:7.00mm"));
    CHECK(BenchProfile::OledDisplaySnapshot().minRenderGapTicks>=100);
    BenchProfile::OledSessionStop();CHECK(OledDiagnostic::Stop());CHECK(!OledDiagnostic::Busy());
    // The real display thread clears all accumulated notifications and renders
    // at most the one latest snapshot, rather than replaying old notifications.
    Reset();StartSplit(3);dummy.currentRailPos=11;AdvanceTicks(20);PumpWorker();
    dummy.currentRailPos=12;AdvanceTicks(20);PumpWorker();CHECK(notifications==3);
    notifyLimit=1;displayOnStack=true;SetCurrent(displayTask);
    try{ThreadOledDisplay(nullptr);CHECK(false);}catch(const ThreadDone&){}
    displayOnStack=false;SetCurrent(command);CHECK(notifications==0);CHECK(Count("send")==1);
    CHECK(HasText("rail:12.00mm"));CHECK(!HasText("rail:11.00mm"));
    BenchProfile::OledSessionStop();CHECK(OledDiagnostic::Stop());
    // No acknowledgement times out closed and restores before allowing a new
    // request. A legacy send cannot recursively become the display owner.
    Reset();CHECK(!OledDiagnostic::Start(4));CHECK(ticks>=500);CHECK(oledRuntime.flags&OledStartTimeout);
    CHECK(!OledDiagnostic::Busy());CHECK(oledRuntime.restored==1);CHECK(displayPriority==osPriorityNormal);
    Reset();bool startInSend=false;serviceHook=[&](const char* name){
        if(!startInSend&&std::string(name)=="send"){
            startInSend=true;CHECK(workerOnStack);autoWorker=true;CHECK(!OledDiagnostic::Start(4));
            CHECK(!oledDisplayBusy);CHECK(oledActiveMode==0);CHECK(!OledDiagnostic::Busy());
        }
    };
    AdvanceTicks(20);PumpWorker();CHECK(startInSend);CHECK(Count("send")==1);CHECK(oledRuntime.flags&OledStartTimeout);
    serviceHook={};AdvanceTicks(20);PumpWorker();CHECK(Count("send")==2);
    // Priority assignment, readback, and restoration errors are real failures.
    Reset();autoWorker=true;failBelowPriority=true;CHECK(!OledDiagnostic::Start(4));
    CHECK(oledRuntime.flags&OledPriorityError);CHECK(oledRuntime.restored==1);CHECK(!OledDiagnostic::Busy());
    Reset();autoWorker=true;wrongReadback=true;CHECK(!OledDiagnostic::Start(4));
    CHECK(oledRuntime.flags&OledPriorityError);CHECK(displayPriority==osPriorityNormal);CHECK(!OledDiagnostic::Busy());
    Reset();StartSplit(4);BenchProfile::OledSessionStop();failNormalPriority=true;
    CHECK(!OledDiagnostic::Stop());CHECK(oledRuntime.flags&OledPriorityError);
    CHECK(oledRuntime.restored==0);CHECK(OledDiagnostic::Busy());CHECK(oledActiveMode==4);
    const unsigned stoppedSends=Count("send");AdvanceTicks(20);PumpWorker();PumpDisplay();
    CHECK(Count("send")==stoppedSends);CHECK(!OledDiagnostic::Start(3));
    failNormalPriority=false;CHECK(OledDiagnostic::Stop());CHECK(!OledDiagnostic::Busy());
    CHECK(oledRuntime.restored==1);CHECK(displayPriority==osPriorityNormal);
    // A command preempting an in-flight display send may wait 500 ms and fail
    // closed. Worker services continue; ownership cannot return prematurely.
    Reset();StartSplit(4);bool stopInSend=false;
    serviceHook=[&](const char* name){
        if(!stopInSend&&std::string(name)=="send"){
            stopInSend=true;CHECK(displayOnStack);CHECK(oledDisplayBusy);
            BenchProfile::OledSessionStop();CHECK(!OledDiagnostic::Stop());
            CHECK(oledRuntime.flags&OledStopTimeout);CHECK(oledRuntime.restored==0);
            CHECK(OledDiagnostic::Busy());CHECK(oledActiveMode==4);
            CHECK(Count("send")==1);CHECK(Count("imu")>=25);
        }
    };
    PumpDisplay();CHECK(stopInSend);CHECK(!oledDisplayBusy);CHECK(OledDiagnostic::Busy());
    CHECK(BenchProfile::OledDisplaySnapshot().begun==1);CHECK(BenchProfile::OledDisplaySnapshot().ended==0);
    serviceHook={};CHECK(OledDiagnostic::Stop());CHECK(!OledDiagnostic::Busy());
    CHECK(displayPriority==osPriorityNormal);CHECK(oledRuntime.restored==1);
    const unsigned restoredSends=Count("send");AdvanceTicks(20);PumpWorker();CHECK(Count("send")==restoredSends+1);
    // Completed-send gap includes the leading/trailing interval, then freezes
    // before restore. A later ordinary update cannot change latched results.
    Reset();CHECK(OledDiagnostic::Start(0));AdvanceTicks(30);ObserveOledSend(false);
    CHECK(oledRuntime.maxSendGapTicks==30);PumpWorker();AdvanceTicks(45);ObserveOledSend(false);
    CHECK(oledRuntime.maxSendGapTicks==45);AdvanceTicks(5);CHECK(OledDiagnostic::Stop());
    CHECK(oledRuntime.maxSendGapTicks==50);AdvanceTicks(200);PumpWorker();CHECK(oledRuntime.maxSendGapTicks==50);
    Reset();StartSplit(4);AdvanceTicks(123);BenchProfile::OledSessionStop();CHECK(OledDiagnostic::Stop());
    CHECK(oledRuntime.maxSendGapTicks==143);const auto frozen=OledDiagnostic::Snapshot();
    AdvanceTicks(500);PumpWorker();CHECK(OledDiagnostic::Snapshot().maxSendGapTicks==frozen.maxSendGapTicks);
    CHECK(OledDiagnostic::Snapshot().displayStackFree==1536);CHECK(critical==0);
    // Argument/handle failures cannot render or start a split session.
    Reset();CHECK(!OledDiagnostic::Start(5));CHECK(!OledDiagnostic::Busy());
    oledDisplayTaskHandle=nullptr;CHECK(!OledDiagnostic::Start(3));CHECK(oledRuntime.flags&OledPriorityError);
    CHECK(Count("send")==0);CHECK(critical==0);
    std::printf("PASS checks=%u; real extracted worker/display threads, lifecycle bridge, mailbox and backend; original scenarios plus split modes 3/4; synthetic clocks only; no hardware\n",checks);
}
'''


def compile_run(out: Path, snap: Path) -> dict:
    exe = out / ("oled_thread_test.exe" if os.name == "nt" else "oled_thread_test")
    src = out / "oled_thread_test.cpp"
    src.write_text(HARNESS, encoding="utf-8")
    inputs = [src, snap / INST / "bench_profile.cpp"]
    includes = [out, snap / INST]
    if os.name == "nt":
        vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        installation = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
            "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"]).decode("utf-8-sig").strip()
        if not installation:
            raise RuntimeError("Visual Studio x64 C++ tools missing")
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        command = ["cl", "/nologo", "/utf-8", "/std:c++17", "/O2", "/W4", "/EHsc"]
        command += ["/I" + str(path) for path in includes] + list(map(str, inputs)) + ["/Fe:" + str(exe)]
        script = out / "build.cmd"
        script.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                          + subprocess.list2cmdline(command) + '\nexit /b %errorlevel%\n', encoding="utf-8")
        run = subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=out, capture_output=True)
    else:
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise RuntimeError("C++17 compiler missing")
        command = [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra"]
        command += ["-I" + str(path) for path in includes] + list(map(str, inputs)) + ["-o", str(exe)]
        run = subprocess.run(command, cwd=out, capture_output=True)
    (out / "compile.log").write_bytes(run.stdout + run.stderr)
    (out / "command.json").write_text(json.dumps(command, indent=2), encoding="utf-8")
    if run.returncode:
        raise RuntimeError("Compile failed: " + str(out / "compile.log"))
    tested = subprocess.run([str(exe)], cwd=out, capture_output=True)
    (out / "run.log").write_bytes(tested.stdout + tested.stderr)
    if tested.returncode:
        raise RuntimeError("Tests failed: " + str(out / "run.log"))
    return {"passed": True, "exit_code": tested.returncode, "executable_sha256": sha(exe),
            "stdout": tested.stdout.decode("utf-8", errors="replace")}


def main() -> int:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out = ROOT / "firmware_artifacts" / ("oled-thread-host-" + stamp)
    snap = out / "source"
    snap.mkdir(parents=True)
    before = {str(path): sha(ROOT / path) for path in INPUTS}
    result, error = {}, None
    try:
        for path in INPUTS:
            target = snap / path
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / path, target)
        source = (snap / MAIN).read_text(encoding="utf-8-sig")
        start = source.index("osThreadId_t oledTaskHandle;")
        end = source.index("/* Timer Callbacks", start)
        fragment = source[start:end] + "\n"
        (out / "thread_fragment.inc").write_text(fragment, encoding="utf-8")
        result = compile_run(out, snap)
    except Exception as failure:
        error = repr(failure)
        (out / "failure.txt").write_text(error + "\n", encoding="utf-8")
    after = {str(path): sha(ROOT / path) for path in INPUTS}
    (out / "source_identity.json").write_text(json.dumps({"before": before, "after": after,
        "unchanged": before == after}, indent=2), encoding="utf-8")
    report = {"passed": error is None and before == after, "result": result, "error": error,
              "platform": platform.platform(), "python": sys.version, "command": sys.argv,
              "utc": stamp, "devices_opened": False, "firmware_built": False, "mcu_timing": False,
              "real_thread_and_wrappers_extracted": True, "real_backend_compiled": True}
    (out / "results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    files = {p.relative_to(out).as_posix(): sha(p) for p in out.rglob("*") if p.is_file()}
    (out / "evidence_index.json").write_text(json.dumps(files, indent=2), encoding="utf-8")
    print(str(out))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
