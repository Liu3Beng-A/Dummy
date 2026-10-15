"""Native checks of production RGB motion, protocol, and persisted records.

Compiles the complete rgb.cpp and the actual RGB ASCII handlers with only
HAL/RTOS/transport shims. A fixed-address memory buffer models the real shared
Flash sector, including erase/program behavior. No board or device is used.
Requires Windows Visual Studio C++ tools. Build outputs use a temporary folder.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from test_rgb_render_native import function


HAL = r'''
#pragma once
#include <cstdint>
struct TIM_HandleTypeDef {};
extern TIM_HandleTypeDef htim2;
using HAL_StatusTypeDef = int;
constexpr int HAL_OK=0, HAL_ERROR=1, TIM_CHANNEL_4=4;
constexpr uint32_t FLASH_FLAG_EOP=1, FLASH_FLAG_OPERR=2, FLASH_FLAG_WRPERR=4,
    FLASH_FLAG_PGAERR=8, FLASH_FLAG_PGPERR=16, FLASH_FLAG_PGSERR=32;
constexpr int FLASH_TYPEERASE_SECTORS=1, FLASH_SECTOR_11=11,
    FLASH_VOLTAGE_RANGE_3=3, FLASH_TYPEPROGRAM_WORD=2;
struct FLASH_EraseInitTypeDef { unsigned TypeErase, Sector, NbSectors, VoltageRange; };
HAL_StatusTypeDef HAL_FLASH_Unlock();
HAL_StatusTypeDef HAL_FLASH_Lock();
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef*, uint32_t*);
HAL_StatusTypeDef HAL_FLASH_Program(unsigned, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef*, unsigned, uint32_t*, unsigned);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef*, unsigned);
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)(x))
#define __HAL_TIM_SET_COMPARE(a,b,c) ((void)0)
#define __HAL_TIM_SET_COUNTER(a,b) ((void)0)
'''


HARNESS = r'''
#define NOMINMAX
#include <Windows.h>
#undef RGB
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <io.h>
#include <fcntl.h>
#include <string>
#include <vector>
#define private public
#include "rgb.hpp"
#undef private
TIM_HandleTypeDef htim2;
static constexpr uintptr_t SECTOR=0x080E0000, SECTOR_BYTES=0x20000;
static unsigned erases=0, toolProofs=0;
static bool failErase=false, failProgram=false;
HAL_StatusTypeDef HAL_FLASH_Unlock() { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock() { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef* info, uint32_t* error) {
    if (info->Sector!=11 || info->NbSectors!=1 || failErase) return HAL_ERROR;
    std::memset(reinterpret_cast<void*>(SECTOR), 0xFF, SECTOR_BYTES);
    ++erases; *error=0; return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(unsigned type, uint32_t address, uint32_t word) {
    if (type!=FLASH_TYPEPROGRAM_WORD || address<SECTOR || address+4>SECTOR+SECTOR_BYTES || failProgram)
        return HAL_ERROR;
    auto* dst=reinterpret_cast<uint32_t*>(uintptr_t(address));
    if ((*dst & word)!=word) return HAL_ERROR;
    *dst=word; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef*, unsigned, uint32_t*, unsigned) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef*, unsigned) { return HAL_OK; }
namespace TcpTool { void RefreshStorageProof() { ++toolProofs; } }
#include "rgb.cpp"
__BASELINE__

RGB rgb;
struct DummyRobot {
    RGB::State state=RGB::RUN_SEQ;
    bool saveBusy=false;
    RGB::State GetRGBState() const { return state; }
    bool IsRGBSaveBusy() const { return saveBusy; }
    int SaveRGBConfig() { return rgb.SaveConfig() ? 1 : -1; }
} dummy;
struct StreamSink { std::vector<std::string> lines; };
static void Respond(StreamSink& out, const char* format, ...) {
    char buffer[512]; va_list args; va_start(args, format);
    const int n=vsnprintf(buffer, sizeof(buffer), format, args); va_end(args);
    if (n<0 || n>62) { std::fprintf(stderr,"transport payload overflow: %s\n",buffer); std::exit(2); }
    out.lines.emplace_back(buffer);
}
__PROTOCOL__

static unsigned checks=0;
static void check(bool condition, const char* message) {
    ++checks;
    if (!condition) { std::fprintf(stderr,"FAIL: %s\n",message); std::exit(1); }
}
static unsigned stateFor(unsigned slot) { return slot<3 ? RGB::RUN_SEQ+slot : RGB::CALIBRATION; }
static void clearFlash() { std::memset(reinterpret_cast<void*>(SECTOR),0xFF,SECTOR_BYTES); }
static std::vector<uint8_t> flashBytes(uint32_t address, size_t size) {
    const auto* first=reinterpret_cast<const uint8_t*>(uintptr_t(address)); return {first,first+size};
}
static void apply(RGB& target, const RGB::Config& c) {
    uint32_t rev; target.GetConfig(&rev); check(target.ApplyConfig(c,rev),"apply validated config");
}
static StreamSink command(const char* line, bool success=true) {
    StreamSink out;
    check(HandleRGBCommand(line,std::strlen(line),out),"RGB command recognized");
    check(!out.lines.empty(),"RGB command responds");
    check((out.lines.front().rfind("error ",0)!=0)==success,"command expected success/error");
    return out;
}
static bool has(const StreamSink& out, const std::string& line) {
    return std::find(out.lines.begin(),out.lines.end(),line)!=out.lines.end();
}
static unsigned starts(const StreamSink& out,const std::string& prefix) {
    return unsigned(std::count_if(out.lines.begin(),out.lines.end(),[&](const std::string& s){return s.rfind(prefix,0)==0;}));
}
static bool equalColor(RGB::Color a, RGB::Color b) { return a.r==b.r && a.g==b.g && a.b==b.b; }
static RGB::Config legacyConfig(unsigned version) {
    auto c=RGB::Defaults();
    c.manualMode=8;
    c.effects[8]={{17,95,203},{7,6,5},2250,200,1000,1,6};
    for (unsigned s=0;s<RGB::STATE_COUNT;++s) {
        auto& b=c.bindings[s];
        if (b.mode>=RGB::MOTION_CIRCLE) b.mode=8;
        if (version==1) {
            if (b.mode>=10) b.mode=0;
            if (b.customColor>1) { b.color=c.autoColors[b.customColor-2]; b.customColor=1; }
        }
    }
    c.bindings[RGB::RUN_SEQ]={8,0,{0,0,0}};
    c.bindings[RGB::RUN_INT]={8,1,{40,50,60}};
    c.bindings[RGB::RUN_TRJ]={5,0,{0,0,0}};
    c.bindings[RGB::CALIBRATION]={8,1,{255,180,0}};
    c.bindings[RGB::READY_SEQ]={4,1,{90,70,50}};
    c.bindings[RGB::STOP]={9,1,{255,180,0}};
    return c;
}
static void writeRecord(unsigned version,const RGB::Config& c,bool badCrc=false) {
    clearFlash();
    if (version==1) {
        RGB::LegacyFlashRecord r{}; r.magic=RGB_MAGIC; r.version=1; r.size=sizeof(r.config);
        std::memcpy(&r.config,&c,sizeof(r.config)); r.crc=RGB::Crc(&r,offsetof(RGB::LegacyFlashRecord,crc));
        if (badCrc) r.crc^=1;
        std::memcpy(reinterpret_cast<void*>(RGB::FLASH_ADDRESS),&r,sizeof(r));
    } else if (version==2 || version==3) {
        RGB::PreviousFlashRecord r{}; r.magic=RGB_MAGIC; r.version=uint16_t(version); r.size=sizeof(r.config);
        std::memcpy(&r.config,&c,sizeof(r.config)); r.crc=RGB::Crc(&r,offsetof(RGB::PreviousFlashRecord,crc));
        if (badCrc) r.crc^=1;
        std::memcpy(reinterpret_cast<void*>(RGB::FLASH_ADDRESS),&r,sizeof(r));
    } else {
        RGB::FlashRecord r{}; r.magic=RGB_MAGIC; r.version=uint16_t(version); r.size=sizeof(r.config); r.config=c;
        r.crc=RGB::Crc(&r,offsetof(RGB::FlashRecord,crc)); if (badCrc) r.crc^=1;
        std::memcpy(reinterpret_cast<void*>(RGB::FLASH_ADDRESS),&r,sizeof(r));
    }
}
static void testDefaultsAndRestrictions() {
    const auto c=RGB::Defaults(); check(RGB::Valid(c),"new defaults valid");
    check(c.effects[8].periodMs==1400 && c.effects[8].tail==3 && equalColor(c.effects[8].first,{0,255,0}),"preset 8 defaults unchanged");
    check(RGB::PRESET_MODE_COUNT==13 && RGB::MODE_COUNT==17,"separate manual and total mode bounds");
    for (unsigned slot=0;slot<4;++slot) {
        const auto state=stateFor(slot);
        check(c.bindings[state].mode==13,"automatic motion defaults to circle");
        check(c.motion[slot].periodMs==1400 && c.motion[slot].reverse==0 && c.motion[slot].width==3,"independent motion defaults");
        check(RGB::MotionSlot(state)==int(slot),"motion slot/state identity");
    }
    check(c.bindings[RGB::RUN_SEQ].customColor==2 && c.bindings[RGB::RUN_INT].customColor==3 &&
          c.bindings[RGB::RUN_TRJ].customColor==4,"RUN default colors use mode banks");
    check(equalColor(c.bindings[RGB::CALIBRATION].color,{255,180,0}),"calibration default yellow");
    for (unsigned s=0;s<RGB::STATE_COUNT;++s) for (unsigned mode=13;mode<17;++mode) {
        auto edit=c; edit.bindings[s].mode=uint8_t(mode);
        check(RGB::Valid(edit)==(RGB::MotionSlot(s)>=0),"only four states permit motion algorithms");
    }
    for (unsigned mode=0;mode<17;++mode) {
        auto edit=c; edit.manualMode=uint8_t(mode);
        check(RGB::Valid(edit)==(mode<13),"manual mode remains 0..12");
    }
    auto edit=c; edit.motion[0].periodMs=249; check(!RGB::Valid(edit),"period lower bound");
    edit=c; edit.motion[0].periodMs=30001; check(!RGB::Valid(edit),"period upper bound");
    edit=c; edit.motion[0].reverse=2; check(!RGB::Valid(edit),"reverse bound");
    edit=c; edit.motion[0].width=0; check(!RGB::Valid(edit),"width lower bound");
    edit=c; edit.motion[0].width=13; check(!RGB::Valid(edit),"width upper bound");
}
static void testStorage() {
    uint32_t tool[16]; for (unsigned i=0;i<16;++i) tool[i]=0xA5123400+i;
    for (unsigned version=1;version<=4;++version) {
        auto old=version<4 ? legacyConfig(version) : RGB::Defaults();
        if (version==4) for (unsigned slot=0;slot<4;++slot) {
            old.bindings[stateFor(slot)].mode=uint8_t(13+slot); old.motion[slot]={uint16_t(1700+slot*250),uint8_t(slot%2),uint8_t(2+slot)};
        }
        writeRecord(version,old);
        const auto original=flashBytes(RGB::FLASH_ADDRESS,1024);
        const unsigned initialErases=erases;
        RGB loaded; loaded.LoadConfig(); auto c=loaded.GetConfig();
        check(loaded.GetStatus(RGB::RUN_SEQ).storage==1,"all supported records recognized");
        check(bool(loaded.GetStatus(RGB::RUN_SEQ).dirty)==(version<4),"old upgrade stays dirty until explicit RGB save");
        check(flashBytes(RGB::FLASH_ADDRESS,1024)==original && erases==initialErases,"load migration does not write Flash");
        if (version<4) {
            for (unsigned slot=0;slot<4;++slot)
                check(c.motion[slot].periodMs==2250 && c.motion[slot].reverse==1 && c.motion[slot].width==6,"all old motion parameters snapshotted");
            check(c.bindings[RGB::RUN_SEQ].mode==13 && c.bindings[RGB::RUN_SEQ].customColor==1 &&
                  equalColor(c.bindings[RGB::RUN_SEQ].color,old.effects[8].first),"legacy source-0 color frozen");
            check(c.bindings[RGB::RUN_INT].mode==13 && equalColor(c.bindings[RGB::RUN_INT].color,{40,50,60}),"legacy explicit color preserved");
            check(c.bindings[RGB::CALIBRATION].mode==13 && equalColor(c.bindings[RGB::CALIBRATION].color,{255,180,0}),"legacy calibration migrates independently");
            check(std::memcmp(&c.bindings[RGB::RUN_TRJ],&old.bindings[RGB::RUN_TRJ],sizeof(RGB::Binding))==0,"non-spin RUN binding retained");
            check(std::memcmp(&c.bindings[RGB::READY_SEQ],&old.bindings[RGB::READY_SEQ],sizeof(RGB::Binding))==0,"custom READY binding retained");
            check(c.bindings[RGB::STOP].mode==(version<3 ? 7 : 9),"STOP migration respects record version");
        } else check(std::memcmp(&c,&old,sizeof(c))==0,"v4 exact configuration restored");
        c.motion[0]={4200,0,11}; c.brightness=37; c.autoColors[0]={9,8,7}; apply(loaded,c);
        check(loaded.SaveToolRecord(tool,sizeof(tool)),"TCP save succeeds beside RGB draft");
        check(flashBytes(RGB::FLASH_ADDRESS,1024)==original,"TCP save preserves actual saved RGB bytes including legacy");
        check(loaded.GetStatus(RGB::RUN_SEQ).storage==1 && loaded.GetStatus(RGB::RUN_SEQ).dirty,"proof accepts saved version without accepting RGB draft");
        RGB restartBefore; restartBefore.LoadConfig();
        check(restartBefore.GetConfig().motion[0].width!=11 && restartBefore.GetConfig().brightness!=37,"TCP save does not persist RGB draft");
        const auto toolSaved=flashBytes(TOOL_FLASH_ADDRESS,256);
        const unsigned proofs=toolProofs;
        check(loaded.SaveConfig(),"explicit RGB save writes v4");
        check(toolProofs==proofs+1,"RGB save refreshes TCP storage proof");
        check(flashBytes(TOOL_FLASH_ADDRESS,256)==toolSaved,"RGB save retains persisted TCP bytes");
        check(reinterpret_cast<const RGB::FlashRecord*>(RGB::FLASH_ADDRESS)->version==4,"migration becomes v4 only on RGB save");
        check(!loaded.GetStatus(RGB::RUN_SEQ).dirty,"explicit save marks snapshot clean");
        RGB restart; restart.LoadConfig(); const auto roundTrip=restart.GetConfig();
        check(std::memcmp(&roundTrip,&c,sizeof(c))==0,"v4 save/restart preserves settings");
        const auto rgbSaved=flashBytes(RGB::FLASH_ADDRESS,1024);
        auto draft=c; draft.motion[0].width=1; draft.autoColors[0]={44,55,66}; apply(loaded,draft);
        tool[0]^=0x10;
        check(loaded.SaveToolRecord(tool,sizeof(tool)),"second TCP save succeeds with current RGB draft");
        check(flashBytes(RGB::FLASH_ADDRESS,1024)==rgbSaved && loaded.GetStatus(RGB::RUN_SEQ).dirty,"v4 proof keeps actual saved RGB snapshot");
        RGB restartAfter; restartAfter.LoadConfig(); check(restartAfter.GetConfig().motion[0].width==11,"later TCP save still does not persist RGB draft");
        const unsigned unchangedErases=erases; check(restartAfter.SaveConfig() && erases==unchangedErases,"unchanged RGB save does not erase sector");
    }
    for (unsigned version=1;version<=4;++version) {
        writeRecord(version,version<4 ? legacyConfig(version) : RGB::Defaults(),true);
        const auto invalid=flashBytes(RGB::FLASH_ADDRESS,1024);
        RGB bad; bad.LoadConfig();
        check(bad.GetStatus(RGB::RUN_SEQ).storage==2 && bad.GetConfig().motion[0].width==3,"bad CRC rejects entire record");
        check(bad.SaveToolRecord(tool,sizeof(tool)),"TCP save beside bad RGB record succeeds");
        check(flashBytes(RGB::FLASH_ADDRESS,1024)==invalid && bad.GetStatus(RGB::RUN_SEQ).storage==2,"proof continues to reject bad CRC after TCP save");
    }
    writeRecord(3,legacyConfig(3));
    auto* previous=reinterpret_cast<RGB::PreviousFlashRecord*>(RGB::FLASH_ADDRESS);
    previous->config.bindings[RGB::RUN_SEQ].mode=13; previous->crc=RGB::Crc(previous,offsetof(RGB::PreviousFlashRecord,crc));
    RGB invalidOld; invalidOld.LoadConfig(); check(invalidOld.GetStatus().storage==2,"old record cannot smuggle new mode identifiers");
    auto invalidNew=RGB::Defaults(); invalidNew.motion[3].width=0; writeRecord(4,invalidNew);
    RGB badMotion; badMotion.LoadConfig(); check(badMotion.GetStatus().storage==2,"v4 CRC alone cannot validate invalid motion");
    clearFlash(); RGB erased; erased.LoadConfig(); check(erased.GetStatus().storage==0,"erased storage distinguished from bad record");
}
static void testStorageFailures() {
    for (bool toolSave: {false,true}) for (bool programFailure: {false,true}) {
        auto persisted=RGB::Defaults(); persisted.brightness=77; persisted.motion[0].width=5;
        writeRecord(4,persisted);
        uint32_t savedTool[16],draftTool[16];
        for (unsigned i=0;i<16;++i) { savedTool[i]=0xA5314600+i; draftTool[i]=0xB6425700+i; }
        std::memcpy(reinterpret_cast<void*>(TOOL_FLASH_ADDRESS),savedTool,sizeof(savedTool));
        const auto beforeRgb=flashBytes(RGB::FLASH_ADDRESS,1024), beforeTool=flashBytes(TOOL_FLASH_ADDRESS,256);
        RGB loaded; loaded.LoadConfig();
        auto draft=loaded.GetConfig(); draft.brightness=33; draft.motion[0].width=12; apply(loaded,draft);
        failErase=!programFailure; failProgram=programFailure;
        const bool ok=toolSave ? loaded.SaveToolRecord(draftTool,sizeof(draftTool)) : loaded.SaveConfig();
        failErase=failProgram=false;
        check(!ok,"Flash erase/program failure must report save failure");
        check(loaded.GetStatus(RGB::RUN_SEQ).dirty,"failed save cannot mark RGB draft clean");
        check(!loaded.savedValid || std::memcmp(&loaded.savedConfig,&persisted,sizeof(persisted))==0,"storage proof never accepts RGB draft after a failed write");
        RGB restart; restart.LoadConfig();
        if (!programFailure) {
            check(flashBytes(RGB::FLASH_ADDRESS,1024)==beforeRgb && flashBytes(TOOL_FLASH_ADDRESS,256)==beforeTool,"erase failure leaves both actual saved records intact");
            check(restart.GetConfig().brightness==77 && restart.GetConfig().motion[0].width==5,"restart after erase failure loads saved RGB rather than draft");
            check(!std::memcmp(reinterpret_cast<const void*>(TOOL_FLASH_ADDRESS),savedTool,sizeof(savedTool)),"erase failure cannot persist TCP draft");
            if (toolSave) check(loaded.GetStatus(RGB::RUN_SEQ).storage==1,"failed TCP save proves only the still-valid RGB record");
            else check(loaded.GetStatus(RGB::RUN_SEQ).storage==2 && !loaded.savedValid,"failed RGB save does not claim successful storage");
        } else {
            check(restart.GetStatus(RGB::RUN_SEQ).storage==0,"first-word program failure leaves no valid RGB record after erase");
            check(loaded.GetStatus(RGB::RUN_SEQ).storage!=1 && !loaded.savedValid,"program failure cannot retain a false valid storage proof");
            check(restart.GetConfig().brightness!=33 && restart.GetConfig().motion[0].width!=12,"program failure cannot make RGB draft survive restart");
            check(std::memcmp(reinterpret_cast<const void*>(TOOL_FLASH_ADDRESS),draftTool,sizeof(draftTool))!=0,"program failure cannot make TCP draft appear saved");
        }
    }
}
static void testIsolationAndFrames() {
    RGB renderer;
    auto c=RGB::Defaults(); c.bindings[RGB::RUN_SEQ]={13,1,{255,91,37}};
    for (unsigned reverse=0;reverse<2;++reverse) for (unsigned width=1;width<=12;++width) for (unsigned period: {250u,1400u,30000u}) {
        c.motion[0]={uint16_t(period),uint8_t(reverse),uint8_t(width)};
        c.effects[8].first={255,91,37}; c.effects[8].periodMs=uint16_t(period); c.effects[8].reverse=uint8_t(reverse); c.effects[8].tail=uint8_t(width);
        c.automatic=1; const auto automatic=RGB::SelectScene(c,RGB::RUN_SEQ);
        c.automatic=0; c.manualMode=8; const auto manual=RGB::SelectScene(c,RGB::RUN_SEQ);
        for (unsigned t: {0u,1u,period/3,period-1,period,period+1,0xFFFFFFFFu}) {
            float a[24][3],b[24][3]; renderer.Render(automatic,t,a); renderer.Render(manual,t,b);
            check(std::memcmp(a,b,sizeof(a))==0,"automatic circle is channel-exact with preset 8");
        }
    }
    c=RGB::Defaults();
    for (unsigned slot=0;slot<4;++slot) for (unsigned kind=0;kind<4;++kind) {
        const auto state=RGB::State(stateFor(slot)); c.bindings[state].mode=uint8_t(13+kind);
        const auto before=RGB::SelectScene(c,state);
        auto edit=c; edit.effects[8]={{123,45,67},{89,90,91},9999,333,444,1,12};
        check(RGB::SameScene(before,RGB::SelectScene(edit,state)),"preset 8 edits cannot change automatic scenes");
        edit=c; edit.motion[(slot+1)%4].width=10;
        check(RGB::SameScene(before,RGB::SelectScene(edit,state)),"each state owns independent motion parameters");
        apply(renderer,c); check(renderer.GetStatus(state).mode==13+kind,"status reports automatic algorithm identifier");
    }
    c=RGB::Defaults(); c.bindings[RGB::RUN_SEQ]={13,0,{0,0,0}};
    check(equalColor(RGB::SelectScene(c,RGB::RUN_SEQ).solid,{0,255,0}),"new source 0 is fixed green without color-bank overrun");
    for (unsigned mode=10;mode<=12;++mode) {
        c.manualMode=uint8_t(mode); c.automatic=0;
        check(equalColor(RGB::SelectScene(c,RGB::RUN_SEQ).solid,c.autoColors[mode-10]),"existing manual auto-color presets retain correct indexing");
    }
    for (unsigned kind=0;kind<4;++kind) for (unsigned reverse=0;reverse<2;++reverse) for (unsigned width: {1u,3u,12u}) {
        c=RGB::Defaults(); c.bindings[RGB::RUN_SEQ].mode=uint8_t(13+kind); c.motion[0]={1400,uint8_t(reverse),uint8_t(width)};
        const auto s=RGB::SelectScene(c,RGB::RUN_SEQ);
        float first[24][3],last[24][3]; renderer.Render(s,0,first); renderer.Render(s,1400,last);
        check(std::memcmp(first,last,sizeof(first))==0,"all motion algorithms loop exactly at their period");
        for (unsigned t=0;t<1400;t+=13) {
            float f[24][3]; renderer.Render(s,t,f);
            for (const auto& led:f) for (float v:led) check(std::isfinite(v) && v>=0 && v<=255,"all legal widths render finite bounded RGB");
            if (kind==2) for (unsigned i=0;i<12;++i) for (unsigned ch=0;ch<3;++ch)
                check(std::fabs(f[i][ch]-f[i+12][ch])<0.01f,"wave has two opposing equal peaks within float tolerance");
        }
    }
}
static void testLegacyFrames() {
#ifdef HAS_BASELINE
    RGB current; PriorRenderer prior;
    for (unsigned mode=0;mode<13;++mode) for (unsigned reverse=0;reverse<2;++reverse)
        for (unsigned width=1;width<=12;++width) for (unsigned period: {250u,1400u,30000u})
        for (float scale: {0.15f,0.25f,1.0f}) {
            RGB::Scene s{}; s.mode=uint8_t(mode); s.scale=scale; s.solid={255,91,37};
            s.effect={{255,91,37},{17,180,255},uint16_t(period),200,1000,uint8_t(reverse),uint8_t(width)};
            for (unsigned t: {0u,1u,period/3,period-1,period,period+1,0xFFFFFFFFu}) {
                float a[24][3],b[24][3]; current.Render(s,t,a); prior.RenderPrior(s,t,b);
                check(std::memcmp(a,b,sizeof(a))==0,"all legacy 0..12 frames remain bit-identical to baseline");
            }
        }
#endif
}
static void testProtocol() {
    rgb=RGB(); clearFlash(); rgb.LoadConfig();
    auto status=command("#RGB_STATUS"); check(!has(status,"RGB_CAPS motion=1") && starts(status,"RGB_MOTION ")==0,"short status remains short");
    auto full=command("#RGB_GET"); check(has(full,"RGB_CAPS motion=1") && starts(full,"RGB_MOTION ")==4 && has(full,"RGB_GET end"),"full query advertises and reports independent motion settings");
    for (unsigned slot=0;slot<4;++slot) for (unsigned kind=0;kind<4;++kind) {
        const auto before=rgb.GetConfig(); char line[96]; std::snprintf(line,sizeof(line),"!RGB_MOTION %u %u %u %u %u",slot,kind,1000+slot*300,slot%2,slot+1);
        auto out=command(line); const auto after=rgb.GetConfig(); const auto state=stateFor(slot);
        check(after.bindings[state].mode==13+kind && after.bindings[state].customColor==before.bindings[state].customColor && equalColor(after.bindings[state].color,before.bindings[state].color),"motion command preserves state color source and RGB");
        check(after.motion[slot].periodMs==1000+slot*300 && after.motion[slot].reverse==slot%2 && after.motion[slot].width==slot+1,"motion command updates all parameters atomically");
        check(has(out,"RGB_CAPS motion=1") && starts(out,"RGB_MOTION ")==4,"motion mutation returns complete confirmed config");
        for (unsigned other=0;other<4;++other) if (other!=slot)
            check(std::memcmp(&before.motion[other],&after.motion[other],sizeof(RGB::Motion))==0,"motion mutation leaves other slots unchanged");
    }
    command("!RGB_SET_STATE CALIBRATION 16 255 180 0");
    check(rgb.GetConfig().bindings[RGB::CALIBRATION].mode==16,"state command accepts calibration motion algorithm");
    command("!RGB_SET_STATE RUN_INT 15 AUTO_INT");
    check(rgb.GetConfig().bindings[RGB::RUN_INT].customColor==3,"state command accepts automatic color bank with new motion");
    for (const char* invalid: {"!RGB_MODE 13","!RGB_MODE 16","!RGB_SET_STATE BOOT 13","!RGB_SET_STATE READY_SEQ 14",
            "!RGB_SET_STATE RUN_TUN 15","!RGB_SET_STATE STOP 16","!RGB_SET_STATE FAULT 13",
            "!RGB_SET_START 13","!RGB_SET_ENABLE 14","!RGB_SET_DISABLE 16","!RGB_MOTION 4 0 1400 0 3",
            "!RGB_MOTION 0 4 1400 0 3","!RGB_MOTION 0 0 249 0 3","!RGB_MOTION 0 0 30001 0 3",
            "!RGB_MOTION 0 0 1400 2 3","!RGB_MOTION 0 0 1400 0 0","!RGB_MOTION 0 0 1400 0 13",
            "!RGB_MOTION 0 -1 1400 0 3","!RGB_MOTION 0 0 4294967296 0 3","!RGB_MOTION 0 0 1400 0",
            "!RGB_MOTION 0 0 1400 0 3 extra"}) {
        uint32_t rev; const auto before=rgb.GetConfig(&rev); command(invalid,false); uint32_t now;
        const auto after=rgb.GetConfig(&now); check(rev==now && std::memcmp(&before,&after,sizeof(before))==0,"invalid command cannot partially update config");
    }
    command("!RGB_SET_STATE RUN_SEQ 8");
    const auto oldColor=rgb.GetConfig().effects[8].first;
    command("!RGB_MOTION 0 1 250 1 12");
    const auto frozen=rgb.GetConfig().bindings[RGB::RUN_SEQ];
    check(frozen.mode==14 && frozen.customColor==1 && equalColor(frozen.color,oldColor),"motion command freezes legacy source-0 spin color");
    const auto beforeEdit=RGB::SelectScene(rgb.GetConfig(),RGB::RUN_SEQ);
    command("!RGB_EFFECT 8 1 2 3 4 5 6 30000 0 1 20 20");
    check(RGB::SameScene(beforeEdit,RGB::SelectScene(rgb.GetConfig(),RGB::RUN_SEQ)),"protocol preset-8 edit does not affect automatic animation");
    const auto beforeManual=rgb.GetConfig(); rgb.SetManualMode(13); const auto afterManual=rgb.GetConfig();
    check(std::memcmp(&beforeManual,&afterManual,sizeof(beforeManual))==0,"direct manual setter rejects automatic modes");
    command("!RGB_MOTION 3 3 30000 1 12 &");
    RGB restart; restart.LoadConfig();
    check(restart.GetConfig().bindings[RGB::CALIBRATION].mode==16 && restart.GetConfig().motion[3].periodMs==30000,"motion protocol explicit save survives restart");
}
static int dumpMotionSamples() {
    _setmode(_fileno(stdout),_O_BINARY);
    RGB renderer; unsigned slot,kind,period,reverse,width,elapsed,explicitColor,bright;
    while (std::scanf("%u %u %u %u %u %u %u %u",&slot,&kind,&period,&reverse,&width,&elapsed,&explicitColor,&bright)==8) {
        if (slot>=4 || kind>=4 || period<250 || period>30000 || reverse>1 || width<1 || width>12 || explicitColor>1 || bright>100) return 3;
        auto c=RGB::Defaults();
        // Distinct parameters in every slot expose accidental cross-slot reads.
        for (unsigned i=0;i<4;++i) c.motion[i]={uint16_t(2400+i*300),uint8_t(i%2),uint8_t(2+i)};
        const unsigned state=stateFor(slot); c.bindings[state].mode=uint8_t(13+kind);
        if (explicitColor) c.bindings[state]={uint8_t(13+kind),1,{255,91,37}};
        c.motion[slot]={uint16_t(period),uint8_t(reverse),uint8_t(width)};
        const auto s=RGB::SelectScene(c,RGB::State(state));
        float frame[24][3]; renderer.Render(s,elapsed,frame);
        uint8_t wire[72]; QuantizeFrame(frame,Gain(float(bright)/100.0f),s.source,s.mode,s.solid,wire);
        if (std::fwrite(frame,sizeof(frame),1,stdout)!=1 || std::fwrite(wire,sizeof(wire),1,stdout)!=1) return 4;
    }
    return std::ferror(stdin) ? 5 : 0;
}
int main(int argc,char** argv) {
    if (argc==2 && !std::strcmp(argv[1],"--dump-motion")) return dumpMotionSamples();
    void* mapped=VirtualAlloc(reinterpret_cast<void*>(SECTOR),SECTOR_BYTES,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    check(mapped==reinterpret_cast<void*>(SECTOR),"fixed-address simulated Flash allocated");
    testDefaultsAndRestrictions(); testStorage(); testStorageFailures(); testIsolationAndFrames(); testLegacyFrames(); testProtocol();
    std::printf("{\"checks\":%u,\"erases\":%u,\"config_bytes\":%zu,\"flash_record_bytes\":%zu,\"hardware\":false}\n",
        checks,erases,sizeof(RGB::Config),sizeof(RGB::FlashRecord));
    VirtualFree(mapped,0,MEM_RELEASE); return 0;
}
'''


def compare_host_motion(executable, repo, out):
    """Compare production scene selection, raw float frames and wire bytes."""
    sys.path.insert(0, str(repo / "Dummy上位机"))
    from app.core.rgb_simulator import (
        MOTION_STATES, _config, _gain, _quantize_frame, _render, _select_scene,
        default_config,
    )
    cases, scenes = [], {}
    times_by_period = {}
    for period in (250, 1400, 30000):
        times = set(range(0, period, max(1, period // 48)))
        times.update((0, 1, period - 1, period, period + 1, 0xFFFFFFFF))
        for fraction in (.09, .5, .7):
            at = int(round(period * fraction))
            times.update((at - 1, at, at + 1))
        times_by_period[period] = sorted(times)
    for slot in range(4):
        for kind in range(4):
            for width in (1, 3, 12):
                for reverse in (0, 1):
                    for period, times in times_by_period.items():
                        for explicit in (0, 1):
                            key = slot, kind, period, reverse, width, explicit
                            config = default_config()
                            config["motion"] = {i: (2400 + i * 300, i % 2, 2 + i) for i in range(4)}
                            state = MOTION_STATES[slot]
                            binding = config["states"][state]
                            config["states"][state] = ((13 + kind, 1, 255, 91, 37) if explicit
                                                       else (13 + kind, *binding[1:]))
                            config["motion"][slot] = period, reverse, width
                            scenes[key] = _select_scene(_config(config), state, None)
                            for n, elapsed in enumerate(times):
                                brightness = (1, 10, 50, 100)[n % 4]
                                cases.append((slot, kind, period, reverse, width, elapsed, explicit, brightness))
    payload = "".join(" ".join(map(str, row)) + "\n" for row in cases).encode("ascii")
    exported = subprocess.run([str(executable), "--dump-motion"], input=payload,
                              capture_output=True, check=True)
    (out / "motion-parity-native-stderr.log").write_bytes(exported.stderr)
    assert len(exported.stdout) == len(cases) * 360, (len(exported.stdout), len(cases))
    samples = memoryview(exported.stdout)
    floats = samples.cast("f")
    maximum_float = 0.0
    maximum_byte = 0
    worst_float_case = worst_byte_case = None
    failing_frames = 0
    by_mode = {mode: {"max_float_difference": 0.0, "max_byte_difference": 0} for mode in range(13, 17)}
    for n, (slot, kind, period, reverse, width, elapsed, explicit, brightness) in enumerate(cases):
        scene = scenes[slot, kind, period, reverse, width, explicit]
        expected = _render(scene, elapsed)
        frame = floats[n * 90:n * 90 + 72]
        delta = max(abs(actual - channel) for actual, channel in
                    zip(frame, (v for pixel in expected for v in pixel)))
        expected_rgb = _quantize_frame(expected, _gain(brightness / 100), scene)
        expected_grb = bytes(v for pixel in expected_rgb for v in (pixel[1], pixel[0], pixel[2]))
        actual_grb = samples[n * 360 + 288:(n + 1) * 360]
        byte_delta = max(abs(actual - expected) for actual, expected in zip(actual_grb, expected_grb))
        if delta >= .01 or byte_delta > 1:
            failing_frames += 1
        if delta > maximum_float:
            worst_float_case = cases[n]
        if byte_delta > maximum_byte:
            worst_byte_case = cases[n]
        maximum_float = max(maximum_float, delta)
        maximum_byte = max(maximum_byte, byte_delta)
        by_mode[13 + kind]["max_float_difference"] = max(by_mode[13 + kind]["max_float_difference"], delta)
        by_mode[13 + kind]["max_byte_difference"] = max(by_mode[13 + kind]["max_byte_difference"], byte_delta)
    result = {
        "frames": len(cases), "channels": len(cases) * 72,
        "slots": list(range(4)), "modes": list(range(13, 17)),
        "widths": [1, 3, 12], "reverse": [0, 1], "periods_ms": list(times_by_period),
        "brightness_percent": [1, 10, 50, 100],
        "colors": ["default state colors and color banks", "explicit RGB 255,91,37"],
        "phase_samples_by_period": {str(p): len(ts) for p, ts in times_by_period.items()},
        "max_float_difference": maximum_float, "max_byte_difference": maximum_byte,
        "worst_float_case": worst_float_case, "worst_byte_case": worst_byte_case,
        "case_fields": ["slot", "kind", "period_ms", "reverse", "width", "elapsed_ms", "explicit_color", "brightness_percent"],
        "failing_frames": failing_frames, "passed": failing_frames == 0,
        "raw_float_tolerance": .01, "wire_byte_tolerance": 1, "by_mode": by_mode,
        "hardware": False, "mcu_timing": False,
    }
    (out / "motion-parity.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baseline", type=Path, help="Optional earlier rgb.cpp: assert all legacy 0..12 frames remain bit-identical")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    core = repo / "firmware_new/dummy-ref-core-fw"
    inputs = [core / "Bsp/gpio/rgb.cpp", core / "Bsp/gpio/rgb.hpp", core / "Bsp/gpio/rgb_rainbow.hpp",
              core / "UserApp/protocols/ascii_protocol.cpp"]
    inputs.extend([repo / "Dummy上位机/app/core/rgb_simulator.py",
                   repo / "Dummy上位机/app/core/rgb_rainbow.py", core / "UserApp/main.cpp"])
    if args.baseline:
        inputs.append(args.baseline.resolve())
    hashes = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    before = hashes()
    out = (args.output or Path(tempfile.mkdtemp(prefix="rgb-motion-native-"))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    for p in inputs[:3]:
        (out / p.name).write_bytes(p.read_bytes())
    (out / "tim.h").write_text(HAL, encoding="utf-8")
    (out / "FreeRTOS.h").write_text("#pragma once\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n", encoding="utf-8")
    (out / "task.h").write_text("#pragma once\n", encoding="utf-8")
    protocol = inputs[3].read_text(encoding="utf-8")
    protocol = protocol[protocol.index("static void PrintRGBStatus("):protocol.index("static bool HandleMotionCommand(")]
    cpp = out / "rgb_motion_native.cpp"
    prior = ""
    if args.baseline:
        old_render = function(args.baseline.read_text(encoding="utf-8"), "void RGB::Render(")
        prior = "#define HAS_BASELINE 1\nstruct PriorRenderer : RGB {\n" + old_render.replace("void RGB::Render(", "void RenderPrior(", 1) + "\n};\n"
    cpp.write_text(HARNESS.replace("__PROTOCOL__", protocol).replace("__BASELINE__", prior), encoding="utf-8")
    vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
    vs = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
                                  "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"], text=True).strip()
    vcvars = Path(vs) / "VC/Auxiliary/Build/vcvars64.bat"
    cmd = out / "build.cmd"
    cmd.write_text('@echo off\ncall "' + str(vcvars) + '" > nul\nif errorlevel 1 exit /b 1\n'
                   + f'cl /nologo /utf-8 /std:c++17 /O2 /EHsc /fp:precise /I"{out}" "{cpp}" /Fe:rgb_motion_native.exe\nexit /b %errorlevel%\n')
    built = subprocess.run(["cmd.exe", "/d", "/c", str(cmd)], cwd=out, capture_output=True)
    (out / "build.log").write_bytes(built.stdout + built.stderr)
    if built.returncode:
        raise RuntimeError(f"Native compile failed; see {out / 'build.log'}\n{built.stdout.decode(errors='replace')}\n{built.stderr.decode(errors='replace')}")
    run = subprocess.run([str(out / "rgb_motion_native.exe")], cwd=out, capture_output=True, text=True)
    (out / "run.log").write_text(run.stdout + run.stderr, encoding="utf-8")
    if run.returncode:
        raise RuntimeError(f"Native checks failed; see {out / 'run.log'}\n{run.stdout}\n{run.stderr}")
    result = json.loads(run.stdout)
    result["legacy_baseline_compared"] = bool(args.baseline)
    result["host_motion_parity"] = compare_host_motion(out / "rgb_motion_native.exe", repo, out)
    task_source = (core / "UserApp/main.cpp").read_text(encoding="utf-8")
    assert "vTaskDelayUntil(&wake, pdMS_TO_TICKS(20))" in task_source
    result["render_cost_review"] = {
        "basis": "source-level calls per 24-LED frame; no MCU cycle measurement",
        "rgb_task_period_ms": 20, "rgb_task_nominal_hz": 50,
        "rgb_task_priority": "osPriorityNormal", "rgb_task_stack_bytes_unchanged": 2000,
        "mode13_circle": {"cosf_max": 16},
        "mode14_arc": {"cosf": 1, "floorf": 24, "smooth": 48},
        "mode15_wave": {"sinf": 24, "powf": 24, "integer_valley_phase": True},
        "mode16_converge": {"expf": 48, "floorf": 48, "smooth_max": 26},
        "expensive_candidates": [15, 16],
        "timing_limit": "PC parity and call counts do not establish STM32F405 worst-case time or a 20 ms deadline",
        "motor_20khz_path_changed": False, "task_stack_changed": False,
    }
    assert before == hashes(), "Concurrent production edit; rerun against final sources"
    result["input_sha256"] = before
    (out / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"Native output: {out}")
    assert result["host_motion_parity"]["passed"], f"Host motion parity failed; see {out / 'motion-parity.json'}"


if __name__ == "__main__":
    main()
