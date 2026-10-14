// Compile old and current actual functions into independent controller states.
// No device or timing claim; compare output commands and every PID/DCE field.
#include "motor_current_limit_common.inc"
#include <array>
#include <random>
#include <cstdio>
#include <cstdlib>
static unsigned long long checks;
#define CHECK(x)do{++checks;if(!(x)){printf("FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
std::array<int32_t,23> values(const Controls& c) {
    return {c.pid.kp,c.pid.ki,c.pid.kd,c.pid.vError,c.pid.vErrorLast,
      c.pid.outputKp,c.pid.outputKi,c.pid.outputKd,c.pid.integralRound,
      c.pid.integralRemainder,c.pid.output,c.dce.kp,c.dce.kv,c.dce.ki,c.dce.kd,
      c.dce.pError,c.dce.vError,c.dce.outputKp,c.dce.outputKi,c.dce.outputKd,
      c.dce.integralRound,c.dce.integralRemainder,c.dce.output};
}
void compare(unsigned phase,bool calibration,bool local,int32_t cap,
             int32_t current,int32_t integral,int32_t position,int32_t speed) {
    Driver beforeDriver,afterDriver;Context beforeContext,afterContext;
    Controls beforeConfig{},afterConfig{};
    beforeContext.driver=&beforeDriver;afterContext.driver=&afterDriver;
    beforeContext.config.motionParams.ratedCurrent=afterContext.config.motionParams.ratedCurrent=cap;
    beforeConfig.pid.kp=8;beforeConfig.pid.ki=3;beforeConfig.pid.kd=2;
    beforeConfig.pid.outputKi=integral;beforeConfig.pid.integralRound=17;
    beforeConfig.dce.kp=200;beforeConfig.dce.kv=80;beforeConfig.dce.ki=300;beforeConfig.dce.kd=250;
    beforeConfig.dce.outputKi=integral;beforeConfig.dce.integralRound=-23;
    afterConfig=beforeConfig;
    baseline::Motor::Controller before;candidate::Motor::Controller after;
    before.context=&beforeContext;after.context=&afterContext;
    before.config=&beforeConfig;after.config=&afterConfig;
    before.protectionPhase=after.protectionPhase=uint8_t(phase);
    before.calibrationTask=after.calibrationTask=calibration;
    before.localStall=after.localStall=local;
    before.calibrationCurrent=after.calibrationCurrent=1100;
    before.retreatCurrent=after.retreatCurrent=500;
    before.holdingCurrent=after.holdingCurrent=250;
    before.estPosition=after.estPosition=123;
    before.estVelocity=after.estVelocity=-1600;
    for(unsigned operation=0;operation<3;++operation) {
      beforeDriver.calls=afterDriver.calls=0;
      baseline::limitCalls=candidate::limitCalls=0;
      if(operation==0){before.CalcCurrentToOutput(current);after.CalcCurrentToOutput(current);}
      else if(operation==1){before.CalcDceToOutput(position,speed);after.CalcDceToOutput(position,speed);}
      else {before.CalcPidToOutput(speed);after.CalcPidToOutput(speed);}
      CHECK(before.focCurrent==after.focCurrent && before.focPosition==after.focPosition);
      CHECK(beforeDriver.calls==1 && afterDriver.calls==1);
      CHECK(beforeDriver.position==afterDriver.position && beforeDriver.current==afterDriver.current);
      CHECK(values(beforeConfig)==values(afterConfig));
      CHECK(candidate::limitCalls==1 && baseline::limitCalls==(operation==0?1U:2U));
      CHECK(beforeConfig.pid.kpValid==afterConfig.pid.kpValid && beforeConfig.pid.kiValid==afterConfig.pid.kiValid && beforeConfig.pid.kdValid==afterConfig.pid.kdValid);
      CHECK(beforeConfig.stallProtectSwitch==afterConfig.stallProtectSwitch);
    }
}
int main() {
    // Exhaustive actual phase byte, local/calibration combinations and signed
    // cap/output/integral boundaries (including a disabled zero current cap).
    for(unsigned phase=0;phase<256;++phase)for(bool calibration:{false,true})for(bool local:{false,true})
      for(int32_t cap:{-1,0,1,250,500,1100,2500})
        for(int32_t current:{INT32_MIN,-2501,-1101,-501,-251,-1,0,1,251,501,1101,2501,INT32_MAX})
          for(int32_t integral:{-2560001,-2560000,-1126401,-1,0,1,1126401,2560000,2560001})
            compare(phase,calibration,local,cap,current,integral,-3077,510400);
    std::mt19937 random(0x544d0001U);
    for(unsigned n=0;n<200000;++n) {
      auto bounded=[&](int32_t bound){return int32_t(random()%uint32_t(2*bound+1))-bound;};
      compare(random()%256,random()%2,random()%2,random()%4001,bounded(2000000),bounded(4096000),bounded(2000000),bounded(1500000));
    }
    printf("motor_actual_current_limit_comparison_checks=%llu\n",checks);
}
