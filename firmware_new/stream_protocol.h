#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// STREAM_JOINT v2. All wire integers are little-endian. No packed structs on wire.
namespace StreamProtocol {
constexpr uint8_t VERSION=2, MODE=7, AXES=7, ALL_AXES=0x7F;
constexpr uint16_t SEGMENT_TICKS=400, START_DELAY_TICKS=2000;
constexpr uint16_t START_GUARD_TICKS=400, MAX_SEQUENCE=65534;
constexpr uint16_t MAX_PHASE_ERROR=100;
// v2 SEED matches the OPEN-latched held reference exactly. Actual tracking
// error is admitted separately by MotionCompletionProtocol, never by moving
// the held reference to the encoder position when switching controllers.
constexpr uint16_t SEED_TOLERANCE=0;
constexpr uint8_t CAPACITY=8, CORE_CAPACITY=16, PREFILL=5, CAP_TIMED=1, CAP_OPEN_SNAPSHOT=2, CAP_STABLE_ENTRY=4;
constexpr uint8_t SEED=1, END=2;
constexpr size_t RECORD_SIZE=92, RECORD_CRC_OFFSET=90, HEX_LINE_SIZE=186;
// DIAG values are u32 except PHASE_ERROR, HELD_POSITION, OPEN_SPEED..OPEN_HOME,
// START_SPEED..START_HOME and *_EVIDENCE_MEAN (signed i32).
// Without CAP_STABLE_ENTRY, OPEN bit12 is the legacy instantaneous speed gate.
// With this capability, bit12 means missing/expired 50ms steady evidence;
// raw instantaneous speed remains diagnostic, and START bit19 binds held q exactly.
// Tick values below use the local 20kHz clock; cycle values use DWT CYCCNT.
enum DiagnosticField : uint8_t {
    DIAG_PHASE_ERROR=0, DIAG_CURVE_MAX_CYCLES=1, DIAG_PREPARE_MAX_CYCLES=2,
    DIAG_MSP_MARGIN=3, DIAG_RX_OVERFLOW=4, DIAG_VELOCITY_LIMIT=5,
    DIAG_ACCELERATION_LIMIT=6, DIAG_HELD_POSITION=7,
    DIAG_NATIVE_CONTROL_MAX=8, DIAG_STREAM_CONTROL_MAX=9,
    DIAG_CALIBRATION_CONTROL_MAX=10, DIAG_NATIVE_CONTROL_LATE=11,
    DIAG_STREAM_CONTROL_LATE=12, DIAG_CALIBRATION_CONTROL_LATE=13,
    DIAG_NATIVE_CONTROL_GAP=14, DIAG_STREAM_CONTROL_GAP=15,
    DIAG_CALIBRATION_CONTROL_GAP=16, DIAG_REPLY_RETRIES=17,
    DIAG_REPLY_DROPS=18, DIAG_RX_MAIN_MAX_TICKS=19,
    DIAG_START_MAIN_MAX_TICKS=20, DIAG_ENCODER_MAX_CYCLES=21,
    DIAG_CLOSE_LOOP_MAX_CYCLES=22,
    // Latched at the first OPEN decision for a session; survives CLOSE/STOP.
    DIAG_OPEN_SESSION=23, DIAG_OPEN_BLOCKS=24, DIAG_OPEN_MODES=25,
    DIAG_OPEN_SPEED=26, DIAG_OPEN_REFERENCE_SPEED=27, DIAG_OPEN_GOAL=28,
    DIAG_OPEN_REFERENCE=29, DIAG_OPEN_ACTUAL=30, DIAG_OPEN_HOME=31,
    DIAG_OPEN_TICK=32, DIAG_OPEN_SETTLED=33,
    // CAP_STABLE_ENTRY: independent first START entry decision, never OPEN reuse.
    DIAG_START_SESSION=34, DIAG_START_BLOCKS=35, DIAG_START_MODES=36,
    DIAG_START_SPEED=37, DIAG_START_REFERENCE_SPEED=38, DIAG_START_GOAL=39,
    DIAG_START_REFERENCE=40, DIAG_START_ACTUAL=41, DIAG_START_HOME=42,
    DIAG_START_TICK=43, DIAG_START_SETTLED=44,
    DIAG_OPEN_EVIDENCE_AGE=45, DIAG_OPEN_EVIDENCE_RANGE=46, DIAG_OPEN_EVIDENCE_MEAN=47,
    DIAG_START_EVIDENCE_AGE=48, DIAG_START_EVIDENCE_RANGE=49, DIAG_START_EVIDENCE_MEAN=50,
    // Timing schema 1. Whole-callback phase peaks are observed every tick.
    // Component peaks are sampled once per 64 ticks; they are NOT WCET and
    // independent maxima must not be added as one measured callback.
    DIAG_TIMING_SCHEMA=51, DIAG_FEEDBACK_SAMPLE_MAX=52,
    DIAG_PROTECTION_SAMPLE_MAX=53, DIAG_OUTPUT_SAMPLE_MAX=54,
    DIAG_REFERENCE_SAMPLE_MAX=55, DIAG_COMPLETION_SAMPLE_MAX=56,
    DIAG_ENTRY_CALLBACK_MAX=57, DIAG_RUNNING_CALLBACK_MAX=58,
    DIAG_STOP_CALLBACK_MAX=59, DIAG_ENTRY_SAMPLE_COUNT=60,
    DIAG_RUNNING_SAMPLE_COUNT=61, DIAG_STOP_SAMPLE_COUNT=62,
    DIAG_TIMING_SAMPLE_DIVISOR=63,
    DIAG_LAST=DIAG_TIMING_SAMPLE_DIVISOR
};
constexpr uint32_t TIMING_SCHEMA=0x544d0001U;

enum Command : uint8_t {
    OPEN=0x40, Q=0x41, V=0x42, A=0x43, PREPARE=0x44, COMMIT=0x45,
    SYNC=0x46, STATUS=0x47, START=0x48, ABORT=0x49, CAPS=0x4A, DIAG=0x4B
};
enum State : uint8_t {
    STATE_CAPS=0, STATE_OPEN=1, STATE_PREPARED=2, STATE_COMMITTED=3,
    STATE_STARTED=4, STATE_RUNNING=5, STATE_COMPLETED=6, STATE_FAULT=7,
    STATE_IDLE=8
};
enum Reason : uint8_t {
    OK=0, BAD_SESSION=1, BAD_SEQUENCE=2, BAD_CRC=3, BAD_LIMIT=4,
    BUSY=5, UNDERRUN=6, SYNC_TIMEOUT=7, SYNC_DRIFT=8, BLOCKED=9,
    INCOMPLETE=10, BAD_STATE=11, ABORTED=12, BAD_VERSION=13,
    BAD_FLAGS=14, BAD_SEED=15, BAD_TYPE=16, DEADLINE=17, RX_OVERFLOW=18
};
struct Axis { int32_t q, v, a; };
struct Record { uint16_t session, sequence; uint8_t flags; Axis axes[AXES]; };
inline uint16_t Read16(const uint8_t* p) {
    return uint16_t(p[0]) | (uint16_t(p[1])<<8);
}
inline uint32_t Read32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1])<<8) |
           (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24);
}
inline int32_t ReadI32(const uint8_t* p) {
    uint32_t u=Read32(p); int32_t v; memcpy(&v,&u,sizeof(v)); return v;
}
inline void Write16(uint8_t* p, uint16_t v) { p[0]=uint8_t(v);p[1]=uint8_t(v>>8); }
inline void Write32(uint8_t* p, uint32_t v) {
    p[0]=uint8_t(v);p[1]=uint8_t(v>>8);p[2]=uint8_t(v>>16);p[3]=uint8_t(v>>24);
}
inline uint16_t Crc16(const uint8_t* p, size_t n) {
    uint16_t crc=0xFFFF;
    while(n--) {
        crc^=uint16_t(*p++)<<8;
        for(unsigned bit=0;bit<8;++bit)
            crc=(crc&0x8000) ? uint16_t((crc<<1)^0x1021) : uint16_t(crc<<1);
    }
    return crc;
}
inline uint16_t CrcEndpoint(uint16_t session,uint16_t seq,uint8_t flags,
                            int32_t q,int32_t v,int32_t a) {
    uint8_t b[17]; Write16(b,session);Write16(b+2,seq);b[4]=flags;
    Write32(b+5,uint32_t(q));Write32(b+9,uint32_t(v));Write32(b+13,uint32_t(a));
    return Crc16(b,sizeof(b));
}
inline bool ValidMask(uint8_t mask) {
    return mask==ALL_AXES || (mask && mask<=ALL_AXES && !(mask&(mask-1)));
}
inline bool ValidFlags(uint16_t seq,uint8_t flags) {
    return seq<=MAX_SEQUENCE && ((seq==0 && flags==SEED) ||
           (seq!=0 && (flags==0 || flags==END)));
}
inline bool DecodeRecord(const uint8_t* bytes,size_t n,Record& out) {
    if(n!=RECORD_SIZE || bytes[0]!=VERSION || !Read16(bytes+2) ||
       !ValidFlags(Read16(bytes+4),bytes[1]) ||
       Crc16(bytes,RECORD_CRC_OFFSET)!=Read16(bytes+RECORD_CRC_OFFSET)) return false;
    Record value{}; value.flags=bytes[1];value.session=Read16(bytes+2);
    value.sequence=Read16(bytes+4);
    for(unsigned i=0;i<AXES;++i) {
        const uint8_t* p=bytes+6+i*12;
        value.axes[i]={ReadI32(p),ReadI32(p+4),ReadI32(p+8)};
        if((value.flags&(SEED|END)) && (value.axes[i].v || value.axes[i].a)) return false;
    }
    out=value; return true;
}
inline void EncodeRecord(const Record& r,uint8_t* bytes) {
    bytes[0]=VERSION;bytes[1]=r.flags;Write16(bytes+2,r.session);Write16(bytes+4,r.sequence);
    for(unsigned i=0;i<AXES;++i) {
        uint8_t* p=bytes+6+i*12;
        Write32(p,uint32_t(r.axes[i].q));Write32(p+4,uint32_t(r.axes[i].v));
        Write32(p+8,uint32_t(r.axes[i].a));
    }
    Write16(bytes+RECORD_CRC_OFFSET,Crc16(bytes,RECORD_CRC_OFFSET));
}
inline int HexNibble(char c) {
    return c>='0'&&c<='9' ? c-'0' : c>='a'&&c<='f' ? c-'a'+10 :
           c>='A'&&c<='F' ? c-'A'+10 : -1;
}
inline bool DecodeLine(const char* line,size_t n,Record& out) {
    if(n!=HEX_LINE_SIZE || line[0]!='~' || line[1]!='S') return false;
    uint8_t bytes[RECORD_SIZE];
    for(size_t i=0;i<RECORD_SIZE;++i) {
        const int high=HexNibble(line[2+i*2]),low=HexNibble(line[3+i*2]);
        if(high<0 || low<0) return false;
        bytes[i]=uint8_t(high*16+low);
    }
    return DecodeRecord(bytes,sizeof(bytes),out);
}
} // namespace StreamProtocol
