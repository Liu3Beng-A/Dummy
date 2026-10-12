#include "motion_values.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <fstream>
static unsigned checks;
#define CHECK(c) do { ++checks; if (!(c)) { std::printf("FAIL line %d: %s\n",__LINE__,#c); std::exit(1); } } while(0)
static void invalid(const std::string& text, unsigned lo=7, unsigned hi=8) {
    float out[8]; for(auto& v:out) v=1234.5f;
    CHECK(MotionValues::Parse(text.data(),text.size(),out,lo,hi)==0);
    for(float v:out) CHECK(v==1234.5f);
}
int main(int argc,char** argv) {
    float out[8]={};
    const char* crash="0.03,26.26,56.09,0,97.65,0.03,0.02,5";
    CHECK(MotionValues::Parse(crash,std::strlen(crash),out,7,8)==8);
    CHECK(out[0]==.03f && out[5]==.03f && out[6]==.02f && out[7]==5.f);
    const char* decimal=" -.03,+.02,1e-2,-2E+1,0,-0,35 ";
    CHECK(MotionValues::Parse(decimal,std::strlen(decimal),out,6,7)==7);
    CHECK(out[0]==-.03f && out[1]==.02f && out[2]==.01f && out[3]==-20.f);
    const char* optional="0,10,80,0,90,0,0";
    out[7]=0;
    CHECK(MotionValues::Parse(optional,std::strlen(optional),out,7,8)==7 && out[7]==0);
    for(const char* bad : {"", "0,0,0,0,0,0", "0,0,0,0,0,0,0,", "0,0,0,0,0,0,0,1,2",
        "0,0,0,0,0,0,0,5junk", "0,0,0,0,0,0,0,,5", "0,0,0,0,0,0,0,nan",
        "0,0,0,0,0,0,0,inf", "0,0,0,0,0,0,0,-inf", "0,0,0,0,0,0,0,1e99",
        "0,0,0,0,0,0,0,1e-99", "0,0,0,0,0,0,0,1e", "0,0,0,0,0,0,0,+"}) invalid(bad);
    invalid(std::string(64,'0'));
    invalid(std::string("0,0,0,0,0,0,0\0,5",16));
    CHECK(MotionValues::Parse(nullptr,1,out,7,8)==0);
    CHECK(MotionValues::Parse(optional,14,nullptr,7,8)==0);
    CHECK(MotionValues::Parse(optional,14,out,0,8)==0);
    CHECK(MotionValues::Parse(optional,14,out,7,9)==0);
    std::uint32_t random=124578U;
    for(unsigned n=0;n<10000;++n) {
        std::string text; float expected[8];
        for(unsigned j=0;j<8;++j) {
            random=random*1664525U+1013904223U;
            const int hundredths=int(random%18001)-9000;
            char token[24]; std::snprintf(token,sizeof(token),"%.2f",hundredths/100.0);
            expected[j]=std::strtof(token,nullptr);
            if(j)text+=',';text+=token;
        }
        CHECK(text.size()<=63);
        CHECK(MotionValues::Parse(text.data(),text.size(),out,7,8)==8);
        for(unsigned j=0;j<8;++j)CHECK(out[j]==expected[j]);
    }
    unsigned replayed=0;
    if(argc>1) {
        std::ifstream file(argv[1]);CHECK(bool(file));std::string command;
        while(std::getline(file,command)) {
            if(command.empty())continue;
            const unsigned minimum=command[0]=='@'?6:7;
            const unsigned count=MotionValues::Parse(command.data()+1,command.size()-1,out,minimum,minimum+1);
            CHECK(count>=minimum);
            float previous[8]={};
            const int old=std::sscanf(command.c_str()+1,"%f,%f,%f,%f,%f,%f,%f,%f",previous,previous+1,previous+2,previous+3,previous+4,previous+5,previous+6,previous+7);
            CHECK(old==int(count));
            for(unsigned j=0;j<count;++j)CHECK(out[j]==previous[j]);
            ++replayed;
        }
    }
    std::printf("PASS %u checks; 10000 generated CSV inputs; %u log commands replayed\n",checks,replayed);
}
