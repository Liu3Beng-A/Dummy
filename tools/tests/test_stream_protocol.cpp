#include "stream_protocol.h"
#include <cstdio>
#include <cstdlib>
#include <climits>
#include <cstring>
using namespace StreamProtocol;
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
static uint32_t randomState=0x71A812BC;
static uint32_t random32(){randomState=randomState*1664525u+1013904223u;return randomState;}
static void repair(uint8_t* b){Write16(b+90,Crc16(b,90));}
int main(){
    static_assert(sizeof(int32_t)==4,"32-bit signed wire integer");
    CHECK(Crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29B1);
    CHECK(VERSION==2 && MODE==7 && SEGMENT_TICKS==400 && HEX_LINE_SIZE==186);
    CHECK(SEED_TOLERANCE==0 && DIAG_HELD_POSITION==7 && DIAG_LAST==50);
    for(unsigned m=0;m<256;++m)CHECK(ValidMask(uint8_t(m))==(m==127||m==1||m==2||m==4||m==8||m==16||m==32||m==64));
    Record original{},decoded{};original.session=1234;original.sequence=1;
    uint8_t bytes[RECORD_SIZE];char line[HEX_LINE_SIZE+1];line[0]='~';line[1]='S';
    const char* digits="0123456789abcdef";
    for(unsigned trial=0;trial<10000;++trial){
        original.sequence=uint16_t(1+random32()%MAX_SEQUENCE);
        original.flags=trial%10==0?END:0;
        original.session=uint16_t(1+random32()%65535);
        for(auto& axis:original.axes){
            uint32_t value=random32();std::memcpy(&axis.q,&value,4);
            value=random32();std::memcpy(&axis.v,&value,4);
            value=random32();std::memcpy(&axis.a,&value,4);
            if(original.flags==END)axis.v=axis.a=0;
        }
        EncodeRecord(original,bytes);CHECK(DecodeRecord(bytes,sizeof(bytes),decoded));
        CHECK(decoded.session==original.session&&decoded.sequence==original.sequence&&decoded.flags==original.flags);
        for(unsigned axis=0;axis<AXES;++axis)CHECK(!std::memcmp(&original.axes[axis],&decoded.axes[axis],sizeof(Axis)));
        for(unsigned i=0;i<RECORD_SIZE;++i){line[2+2*i]=digits[bytes[i]>>4];line[3+2*i]=digits[bytes[i]&15];}
        CHECK(DecodeLine(line,HEX_LINE_SIZE,decoded));
        const unsigned bit=random32()%(8*RECORD_SIZE);bytes[bit/8]^=uint8_t(1u<<(bit%8));
        const Record before=decoded;CHECK(!DecodeRecord(bytes,sizeof(bytes),decoded));
        CHECK(!std::memcmp(&before,&decoded,sizeof(Record))); // failure publishes no partial record
    }
    original={};original.session=1;original.flags=SEED;EncodeRecord(original,bytes);
    CHECK(DecodeRecord(bytes,92,decoded));
    // Exhaustive single-bit corruption across the full packet, including the CRC itself.
    for(unsigned bit=0;bit<736;++bit){bytes[bit/8]^=uint8_t(1u<<(bit%8));CHECK(!DecodeRecord(bytes,92,decoded));bytes[bit/8]^=uint8_t(1u<<(bit%8));}
    for(size_t len=0;len<256;++len)if(len!=92)CHECK(!DecodeRecord(bytes,len,decoded));
    for(size_t len=0;len<256;++len)if(len!=186)CHECK(!DecodeLine(line,len,decoded));
    bytes[0]=1;repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));
    bytes[0]=VERSION+1;repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));bytes[0]=VERSION;
    bytes[1]=0;repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));bytes[1]=SEED;
    Write16(bytes+2,0);repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));Write16(bytes+2,1);
    Write16(bytes+4,1);repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));Write16(bytes+4,0);
    Write32(bytes+10,1);repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));Write32(bytes+10,0);
    bytes[1]=END;Write16(bytes+4,MAX_SEQUENCE);repair(bytes);CHECK(DecodeRecord(bytes,92,decoded));
    Write16(bytes+4,65535);repair(bytes);CHECK(!DecodeRecord(bytes,92,decoded));
    for(unsigned i=0;i<186;++i){char old=line[i];line[i]='z';CHECK(!DecodeLine(line,186,decoded));line[i]=old;}
    uint8_t endpoint[17];Write16(endpoint,0x1234);Write16(endpoint+2,0x5678);endpoint[4]=0;
    Write32(endpoint+5,0xFFFFFFFF);Write32(endpoint+9,0x80000000);Write32(endpoint+13,0x7FFFFFFF);
    CHECK(CrcEndpoint(0x1234,0x5678,0,-1,INT_MIN,INT_MAX)==Crc16(endpoint,17));
    // Golden record for comparison with independently packed Python.
    original={};original.session=0x1234;original.sequence=0x5678;
    for(unsigned i=0;i<AXES;++i)original.axes[i]={int32_t(i*1000)-3000,int32_t(i*77)-231,int32_t(i*33)-99};
    EncodeRecord(original,bytes);
    std::printf("golden=");for(auto byte:bytes)std::printf("%02x",byte);std::printf("\n");
    std::printf("PASS %u checks, 10000 roundtrips, 10736 corruption cases\n",checks);
}
