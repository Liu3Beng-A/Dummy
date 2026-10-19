#include "cartesian_endurance.h"
#include <cstring>

namespace CartEndurance {
namespace {
uint32_t Max(uint32_t a,uint32_t b){return a>b?a:b;}
bool Same(const CartBench::Sample& a,const CartBench::Sample& b){
    return a.work==b.work&&a.fk==b.fk&&a.ik==b.ik&&a.valid==b.valid&&
        a.selected==b.selected&&a.failed==b.failed&&a.checksum==b.checksum;
}
uint16_t Free(uint32_t value){return uint16_t(value>65535?65535:value);}
}
const char* Name(Error error){
    static const char* const names[]={"NONE","NUMERIC","SIGNATURE","ACCOUNTING","DEADLINE","CLOCK","BUDGET","CANCELLED","ENVIRONMENT","RESTORE","SETUP"};
    return unsigned(error)<=SETUP?names[unsigned(error)]:"ENVIRONMENT";
}
void State::Reset(uint32_t clockHz,uint32_t durationSeconds,uint32_t startDwt){
    // No multi-kilobyte temporary on a firmware task stack.
    std::memset(this,0,sizeof(*this));hz=clockHz;seconds=durationSeconds;
    previousDwt=startDwt;minCycles=0xFFFFFFFFU;errorSequence=0xFFFFFFFFU;
    if(!hz||durationSeconds<1||durationSeconds>MaxSeconds)Fail(SETUP,0);
}
void State::Fail(Error why,uint32_t sequence){if(error==NONE){error=why;errorSequence=sequence;}}
bool State::AdvanceClock(uint32_t rtos,uint32_t hal,uint32_t dwt){
    // Every start (and the final duration boundary) must be observed. A long
    // unobserved gap cannot reconstruct any number of whole DWT wraps.
    const uint32_t gap=rtos-previousElapsed;
    const uint32_t skew=rtos>hal?rtos-hal:hal-rtos;
    // Retain the raw observed RTOS extent even when clock validity fails. The
    // DWT extension must not invent elapsed cycles across an ambiguous gap.
    elapsedMs=rtos;
    if(gap>1000U||skew>3U||uint64_t(gap)*hz>=uint64_t(0xFFFFFFFFU)*1000U){Fail(CLOCK,n);return false;}
    const uint64_t candidate=extendedDwt+uint32_t(dwt-previousDwt);
    const uint64_t measured=hz?candidate*1000U/hz:0;
    if(!hz||measured+3U<hal||measured>uint64_t(hal)+3U){Fail(CLOCK,n);return false;}
    extendedDwt=candidate;previousDwt=dwt;previousElapsed=rtos;
    return true;
}
Error State::Observe(const CartBench::Sample& s,const BenchProfile::Window& w,
                     uint32_t rtos,uint32_t hal,uint32_t dwt,uint32_t late){
    if(n>=seconds*50U||n>=MaxSeconds*50U){Fail(DEADLINE,n);return Error(error);}
    const uint32_t gap=n?rtos-previousElapsed:0;
    AdvanceClock(rtos,hal,dwt);
    if(s.flag==CartBench::BUDGET)Fail(BUDGET,n);
    else if(s.flag==CartBench::CANCELLED)Fail(CANCELLED,n);
    else if(s.flag!=CartBench::COMPLETE)Fail(ENVIRONMENT,n);
    if(late>=20U)Fail(DEADLINE,n);
    if(s.flag==CartBench::COMPLETE){
        if(s.work!=5||s.fk!=10||s.ik!=5||s.valid<5||s.valid>40||s.selected!=5||s.failed)Fail(NUMERIC,n);
        if(n&&!Same(signature,s))Fail(SIGNATURE,n);
        if(!n)signature=s;
    }
    uint64_t total=0;for(unsigned t=0;t<BenchProfile::TaskCount;++t){total+=w.resident[t];resident[t]+=w.resident[t];}
    if(w.flags||total!=w.cycles||w.cycles<s.cycles)Fail(ACCOUNTING,n);
    const uint32_t* lanes[4]={w.oledResident,w.oledElapsed,w.displayResident,w.displayElapsed};
    for(unsigned l=0;l<4;++l){
        uint64_t sum=0;
        for(unsigned k=0;k<BenchProfile::OledStageCount;++k){sum+=lanes[l][k];lane[l][k]+=lanes[l][k];}
        if(sum!=(l==0?w.resident[2]:l==2?w.resident[5]:w.cycles))Fail(ACCOUNTING,n);
    }
    for(unsigned k=0;k<BenchProfile::OledStageCount;++k)
        if(w.oledResident[k]>w.oledElapsed[k]||w.displayResident[k]>w.displayElapsed[k])Fail(ACCOUNTING,n);
    if(uint64_t(w.cycles)*1000000U>=uint64_t(hz)*20000U)Fail(BUDGET,n);
    const unsigned bin=hz?unsigned(uint64_t(s.cycles)*1000U/hz):Bins-1;
    ++histogram[bin<Bins?bin:Bins-1];sumCycles+=s.cycles;sumWindow+=w.cycles;
    if(s.cycles<minCycles)minCycles=s.cycles;
    maxCycles=Max(maxCycles,s.cycles);maxWindow=Max(maxWindow,w.cycles);
    lateCount+=late?1U:0U;maxLate=Max(maxLate,late);maxGap=Max(maxGap,gap);
    const unsigned slot=n/500U;Trend& t=trends[slot];
    ++t.n;t.late+=late?1U:0U;t.sumCycles+=s.cycles;t.maxCycles=Max(t.maxCycles,s.cycles);
    t.maxLate=Max(t.maxLate,late);t.maxGap=Max(t.maxGap,gap);trendCount=slot+1;
    History& h=history[historyHead];h.sequence=n;h.elapsedMs=rtos;h.windowCycles=w.cycles;
    h.lateTicks=late;h.sampleCycles=s.cycles;h.flags=w.flags|(error<<24);
    h.checksum=s.checksum;h.failed=s.failed;h.work=s.work;
    historyHead=(historyHead+1)%HistorySize;if(historyCount<HistorySize)++historyCount;
    ++n;return Error(error);
}
void State::Stacks(uint32_t command,uint32_t worker,uint32_t display,uint32_t cycles){
    ++scanCount;maxScan=Max(maxScan,cycles);
    if(!trendCount)return;
    Trend& t=trends[trendCount-1];
    const uint16_t c=Free(command),w=Free(worker),d=Free(display);
    if(!t.reserved||c<t.commandFree)t.commandFree=c;
    if(!t.reserved||w<t.workerFree)t.workerFree=w;
    if(!t.reserved||d<t.displayFree)t.displayFree=d;
    t.reserved=1;
}
void State::Overhead(uint32_t cycles){sumOverhead+=cycles;maxOverhead=Max(maxOverhead,cycles);}
const History& State::HistoryAt(unsigned index)const{return history[(historyHead+HistorySize-historyCount+index)%HistorySize];}
}
