#pragma once
#include "cartesian_benchmark.h"
#include "bench_profile.h"
#include <cstdint>

// Fixed-memory observation only. No RTOS, actuator, CAN, Flash or allocation.
namespace CartEndurance {
constexpr unsigned Bins=21, Trends=60, HistorySize=8, MaxSeconds=600;
enum Error:uint32_t { NONE,NUMERIC,SIGNATURE,ACCOUNTING,DEADLINE,CLOCK,BUDGET,CANCELLED,ENVIRONMENT,RESTORE,SETUP };
const char* Name(Error error);
struct Trend {
    uint16_t n=0,late=0;
    uint32_t maxCycles=0;
    uint64_t sumCycles=0;
    uint32_t maxLate=0,maxGap=0;
    uint16_t commandFree=0,workerFree=0,displayFree=0,reserved=0;
};
struct History {
    uint32_t sequence=0,elapsedMs=0,windowCycles=0,lateTicks=0;
    uint32_t sampleCycles=0,flags=0,checksum=0;
    uint16_t failed=0,work=0;
};
struct State {
    uint32_t hz=0,seconds=0,n=0,elapsedMs=0,minCycles=0,maxCycles=0;
    uint32_t lateCount=0,maxLate=0,maxGap=0,maxWindow=0;
    uint32_t error=NONE,errorSequence=0xFFFFFFFFU;
    uint64_t sumCycles=0,sumWindow=0,extendedDwt=0;
    uint64_t resident[BenchProfile::TaskCount]{};
    uint64_t lane[4][BenchProfile::OledStageCount]{};
    uint32_t histogram[Bins]{};
    Trend trends[Trends]{};
    History history[HistorySize]{};
    CartBench::Sample signature{};
    uint32_t trendCount=0,historyCount=0,historyHead=0;
    uint32_t previousDwt=0,previousElapsed=0;
    uint32_t maxOverhead=0,scanCount=0,maxScan=0;
    uint64_t sumOverhead=0;
    void Reset(uint32_t clockHz,uint32_t durationSeconds,uint32_t startDwt);
    void Fail(Error why,uint32_t sequence);
    bool AdvanceClock(uint32_t elapsedRtosMs,uint32_t elapsedHalMs,uint32_t dwt);
    Error Observe(const CartBench::Sample& sample,const BenchProfile::Window& window,
                  uint32_t elapsedRtosMs,uint32_t elapsedHalMs,uint32_t startDwt,uint32_t lateTicks);
    void Stacks(uint32_t command,uint32_t worker,uint32_t display,uint32_t cycles);
    void Overhead(uint32_t cycles);
    const History& HistoryAt(unsigned index) const;
};
static_assert(sizeof(Trend)==32,"ten-second trend budget changed");
static_assert(sizeof(History)==32,"history budget changed");
static_assert(sizeof(State)<=3072,"endurance storage exceeds 3KiB");
}
