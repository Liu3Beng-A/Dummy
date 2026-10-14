// Actual production recovery allowance versus the d50efb1 expression. No HAL.
#include "motor_feedback_allowance_under_test.inc"
#include <array>
#include <random>
#include <cstdio>
#include <cstdlib>
static unsigned long long checks;
#define CHECK(x)do{++checks;if(!(x)){printf("FAIL line%u %s\n",__LINE__,#x);exit(1);}}while(0)
// Frozen unchanged expression from d50efb1 Motor::CloseLoopControlTick.
int64_t oldAllowed(int32_t velocity,uint32_t missing,int32_t frequency=20000) {
    return missing==0 ? velocity/frequency+64 : int64_t(velocity)*(missing+1)/frequency+64;
}
void compare(int32_t velocity,uint32_t missing,int32_t frequency=20000) {
    const int64_t before=oldAllowed(velocity,missing,frequency);
    const int64_t after=actualAllowed(velocity,missing,frequency);
    CHECK(before==after);
    // The exact signed inequality that accepts/rejects a parity-valid sample,
    // including both boundary points and immediately rejected neighbors.
    for(int64_t candidate:{-before-1,-before,-before+1,int64_t(0),before-1,before,before+1}) {
        CHECK((candidate>before || candidate<-before)==(candidate>after || candidate<-after));
    }
}
int main() {
    constexpr std::array<int32_t,19> velocities={INT32_MIN,-1536000,-20000,-1,0,1,63,64,19999,20000,20001,51199,51200,1536000,1535999,2000000000,2147480000,INT32_MAX-1,INT32_MAX};
    for(uint32_t missing=0;missing<=20000;++missing)for(int32_t velocity:velocities)compare(velocity,missing);
    // Malformed/negative configuration and huge elapsed counts retain the
    // original signed64 path, including unsigned missing+1 overflow tozero.
    for(uint32_t missing:{20001U,20002U,100000U,2147483647U,2147483648U,UINT32_MAX-1,UINT32_MAX})
      for(int32_t velocity:velocities)for(int32_t frequency:{1,19999,20000,20001,-20000})compare(velocity,missing,frequency);
    std::mt19937 random(0x544d0002U);
    for(unsigned n=0;n<300000;++n) {
        const auto velocity=int32_t(random());const auto missing=uint32_t(random());
        compare(velocity,missing);
    }
    // Both exact decomposition products and the sum remain in u32. These
    // inequalities are algebraic for all v>=0,t<=20001, not a timing claim.
    CHECK(uint64_t(19999)*20001==399999999);
    CHECK(uint64_t(INT32_MAX)*20001/20000==2147591021ULL);
    CHECK(uint64_t(INT32_MAX)*20001/20000<UINT32_MAX);
    printf("motor_exact_feedback_allowance_checks=%llu\n",checks);
}
