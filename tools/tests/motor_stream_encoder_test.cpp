#include "mt6816_base.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
static unsigned checks;
#define CHECK(x) do {++checks;if(!(x)){fprintf(stderr,"FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
struct Read {uint16_t word;bool valid;};
uint16_t table[16384];
struct Encoder:MT6816Base {
 std::vector<Read> reads;unsigned calls=0;bool valid=true;
 Encoder():MT6816Base(table){}
 void SpiInit() override{}
 uint16_t SpiTransmitAndRead16Bits(uint16_t)override {CHECK(calls<reads.size());valid=reads[calls].valid;return reads[calls++].word;}
 bool SpiTransactionValid()const override{return valid;}
 void Set(std::vector<Read> value){reads=value;calls=0;}
};
// State-only wrapper around source extracted by motor_stream_test.py.
struct FeedbackState {
 int32_t realLapPosition=300,realLapPositionLast=300,realPosition=300,realPositionLast=300;
 uint32_t transportMissingTicks=0,invalidFeedbackTicks=0;
 bool isStalled=false,softDisable=false;
 int32_t focPosition=12,focCurrent=34;unsigned clears=0;
 void ClearIntegral(){++clears;}
 void InvalidatePositionEntry(){}
};
struct DriverProbe {unsigned sleeps=0;void Sleep(){++sleeps;}};
struct {int32_t velocityLimit=1536000;} boardConfig;
struct FeedbackProbe {
 static constexpr int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS=51200;
 struct {const int32_t CONTROL_FREQUENCY=20000;} motionPlanner;
 Encoder* encoder;FeedbackState* controller;DriverProbe* driver;
 #include "motor_feedback_under_test.h"
};
int main(){
 for(unsigned i=0;i<16384;++i)table[i]=uint16_t(i*3);
 Encoder e;
 // 0x0191 has even parity and rawAngle100. 0x0190 has odd parity.
 e.Set({{1,true},{0x90,true},{1,true},{0x91,true}});CHECK(e.UpdateAngle()==300);CHECK(e.calls==4 && e.IsSampleValid());
 e.Set({{1,true},{0x90,true},{1,true},{0x91,true}});CHECK(e.UpdateAngle(1)==300);CHECK(e.calls==2 && !e.IsSampleValid());
 e.Set({{1,true},{0x91,true}});CHECK(e.UpdateAngle(1)==300 && e.calls==2 && e.IsSampleValid());
 e.Set({{0,false},{0,false},{0,false}});e.UpdateAngle();CHECK(e.calls==3 && !e.IsSampleValid());
 e.Set({{0,false}});e.UpdateAngle(1);CHECK(e.calls==1 && !e.IsSampleValid());
 e.Set({{1,true},{0x92,true}});e.UpdateAngle(1);CHECK(!e.IsSampleValid()); // no magnet, valid parity
 e.Set({{0,false}});e.UpdateAngle(0);CHECK(e.calls==1 && !e.IsSampleValid());
 e.Set({{0,false},{0,false},{0,false}});e.UpdateAngle(255);CHECK(e.calls==3 && !e.IsSampleValid());
 for(unsigned raw=0;raw<65536;++raw) {
  e.Set({{uint16_t(raw>>8),true},{uint16_t(raw&255),true}});e.UpdateAngle(1);
  unsigned ones=0;for(unsigned bit=0;bit<16;++bit)ones+=(raw>>bit)&1;
  CHECK(e.IsSampleValid()==(!(ones&1) && !(raw&2)));
 }
 // Invalid samples never overwrite the last valid rectified position.
 e.Set({{1,true},{0x91,true}});CHECK(e.UpdateAngle(1)==300);
 for(unsigned tick=0;tick<100;++tick) {e.Set({{0,false}});CHECK(e.UpdateAngle(1)==300 && !e.IsSampleValid());}
 e.Set({{1,true},{0x91,true}});CHECK(e.UpdateAngle(1)==300 && e.IsSampleValid());
 // The actual controller guard sleeps at 100 invalid samples (5 ms), not 99.
 // Encoder Init confirms calibration-table validity before this initialized-state case.
 e.Set({{1,true},{0x91,true}});CHECK(e.Init());
 FeedbackState state;DriverProbe driver;FeedbackProbe probe{ {},&e,&state,&driver };
 for(unsigned tick=1;tick<=101;++tick){
  e.Set({{0,false}});e.UpdateAngle(1);probe.UpdateAndGuard();
  CHECK(state.realPosition==300 && state.invalidFeedbackTicks==tick);
  CHECK(driver.sleeps==(tick>=100 ? tick-99 : 0));
 }
 CHECK(state.focPosition==0 && state.focCurrent==0 && state.clears==2);
 // One new valid, plausible sample resets the counter; output enable/latching
 // remains the separate protection state machine's responsibility.
 e.Set({{1,true},{0x91,true}});e.UpdateAngle(1);probe.UpdateAndGuard();
 CHECK(state.invalidFeedbackTicks==0 && state.transportMissingTicks==0 && driver.sleeps==2);
 // Parity-valid discontinuities never accumulate a growing movement allowance.
 for(unsigned tick=1;tick<=100;++tick){
  e.angleData.rectifiedAngle=3000;probe.UpdateAndGuard();
  CHECK(state.invalidFeedbackTicks==tick && state.transportMissingTicks==0 && state.realPosition==300);
 }
 CHECK(driver.sleeps==3);
 printf("encoder_attempt_budget_checks=%u\n",checks);
}
