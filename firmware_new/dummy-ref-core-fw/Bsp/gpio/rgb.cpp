#include "rgb.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>
#include <cstring>

namespace TcpTool { void RefreshStorageProof(); }

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr uint32_t RGB_MAGIC = 0x52474231UL;
constexpr uint16_t RGB_VERSION = 3;
constexpr uint32_t TOOL_FLASH_ADDRESS = 0x080FFB00UL, SHARED_BYTES = 1280;
alignas(4) uint32_t sharedStorage[SHARED_BYTES / 4]{};
bool sharedStorageBusy = false;
enum SharedWrite { WRITE_BUSY, WRITE_FAILED, WRITE_VERIFIED };
SharedWrite RewriteSharedSector(uint32_t address, const void* record, size_t length) {
    taskENTER_CRITICAL();
    if (sharedStorageBusy) { taskEXIT_CRITICAL(); return WRITE_BUSY; }
    sharedStorageBusy = true;
    taskEXIT_CRITICAL();
    const auto* flash = reinterpret_cast<const uint8_t*>(TOOL_FLASH_ADDRESS);
    SharedWrite result = WRITE_VERIFIED;
    if (std::memcmp(reinterpret_cast<const void*>(address), record, length)) {
        std::memcpy(sharedStorage, flash, sizeof(sharedStorage));
        std::memcpy(reinterpret_cast<uint8_t*>(sharedStorage) + address - TOOL_FLASH_ADDRESS, record, length);
        bool ok = HAL_FLASH_Unlock() == HAL_OK;
        if (ok) {
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
            FLASH_EraseInitTypeDef erase{};
            erase.TypeErase = FLASH_TYPEERASE_SECTORS; erase.Sector = FLASH_SECTOR_11;
            erase.NbSectors = 1; erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            uint32_t sectorError = 0;
            ok = HAL_FLASHEx_Erase(&erase, &sectorError) == HAL_OK;
            for (unsigned offset = 0; ok && offset < sizeof(sharedStorage); offset += 4) {
                const uint32_t word = sharedStorage[offset / 4];
                if (word != 0xFFFFFFFFUL)
                    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TOOL_FLASH_ADDRESS + offset, word) == HAL_OK;
            }
            HAL_FLASH_Lock();
        }
        if (!ok || std::memcmp(flash, sharedStorage, sizeof(sharedStorage))) result = WRITE_FAILED;
    }
    taskENTER_CRITICAL(); sharedStorageBusy = false; taskEXIT_CRITICAL();
    return result;
}
const char* const names[RGB::STATE_COUNT] = {
    "BOOT", "DISABLED", "READY_SEQ", "READY_INT", "READY_TRJ", "READY_TUN",
    "RUN_SEQ", "RUN_INT", "RUN_TRJ", "RUN_TUN", "CALIBRATION", "STOP", "STARTING", "FAULT"
};
// Monotonic perceptual dimming; first full-scale color level is visible at 1%.
const uint8_t gammaPercent[101] = {0,1,1,1,1,1,2,2,3,3,4,5,6,6,7,8,9,11,12,13,14,15,17,18,20,21,23,24,26,27,29,31,33,35,37,39,41,43,45,47,49,51,54,56,58,61,63,66,68,71,73,76,79,81,84,87,90,93,96,99,102,105,108,111,114,117,121,124,127,131,134,138,141,145,148,152,156,159,163,167,171,175,178,182,186,190,194,198,203,207,211,215,219,224,228,233,237,241,246,250,255};
float Gain(float brightness) {
    float index = brightness * 100;
    if (index <= 0) return 0;
    if (index >= 100) return 1;
    unsigned lo = unsigned(index);
    return (gammaPercent[lo] + (gammaPercent[lo+1] - gammaPercent[lo]) * (index-lo)) / 255.0f;
}
void MigrateStopBinding(RGB::Config& c) {
    auto& b = c.bindings[RGB::STOP];
    if (b.mode == RGB::BLINK && b.customColor == 1 &&
        b.color.r == 255 && b.color.g == 180 && b.color.b == 0)
        b.mode = RGB::RED_HEARTBEAT;
}
}
RGB::RGB(uint8_t) { config = Defaults(); status.state = DISABLED; }
RGB::Config RGB::Defaults() {
    Config c{};
    c.enabled = c.automatic = 1; c.brightness = 100;
    c.fadeOnMs = c.fadeOffMs = 1000; c.brightnessMs = c.transitionMs = 300;
    c.colors[0] = {0,0,255}; c.colors[1] = {0,255,0}; c.colors[2] = {255,255,255};
    c.autoColors[0] = {0,255,0}; c.autoColors[1] = {0,255,255}; c.autoColors[2] = {0,80,255};
    for (auto& e : c.effects) e = {{255,255,255}, {0,0,0}, 4000, 200, 1000, 0, 3};
    c.effects[BLUE_TIDE].first = {0,0,255};
    c.effects[WHITE_BREATH].periodMs = 3800;
    c.effects[CYBER_BREATH].first = {0,255,255};
    c.effects[CYBER_BREATH].second = {160,0,255};
    c.effects[CYBER_BREATH].periodMs = 4200;
    c.effects[RED_HEARTBEAT].first = {255,0,0};
    c.effects[RED_HEARTBEAT].periodMs = 1800;
    c.effects[GREEN_SPIN].first = {0,255,0};
    c.effects[GREEN_SPIN].periodMs = 1400;
    auto bind = [&](State s, uint8_t mode, Color color, uint8_t custom = 1) {
        c.bindings[s] = {mode, custom, color};
    };
    bind(BOOT, RAINBOW, c.colors[0], 0); bind(DISABLED, WHITE_BREATH, c.colors[2], 0);
    for (unsigned i=0; i<AUTO_COLOR_COUNT; ++i) {
        bind(State(READY_SEQ+i), AUTO_SEQ_COLOR+i, {0,0,0}, 0);
        bind(State(RUN_SEQ+i), GREEN_SPIN, {0,0,0}, 2+i);
    }
    bind(READY_TUN, 2, {160,0,255}); bind(RUN_TUN, WHITE_BREATH, {160,0,255});
    bind(CALIBRATION, GREEN_SPIN, {255,180,0}); bind(STOP, RED_HEARTBEAT, {255,180,0});
    bind(STARTING, BLUE_TIDE, {0,255,255}); bind(FAULT, RED_HEARTBEAT, {255,0,0});
    return c;
}
bool RGB::Valid(const Config& c) {
    if (c.enabled > 1 || c.automatic > 1 || c.brightness > 100 || c.manualMode >= MODE_COUNT ||
        c.fadeOnMs > 10000 || c.fadeOffMs > 10000 || c.brightnessMs > 10000 || c.transitionMs > 5000) return false;
    for (const auto& e : c.effects)
        if (e.periodMs < 250 || e.periodMs > 30000 || e.onMs < 20 || e.onMs > 30000 ||
            e.offMs < 20 || e.offMs > 30000 || e.reverse > 1 || e.tail < 1 || e.tail > 12) return false;
    for (const auto& b : c.bindings) if (b.mode >= MODE_COUNT || b.customColor > 1+AUTO_COLOR_COUNT) return false;
    const auto& f = c.bindings[FAULT];
    return f.mode == RED_HEARTBEAT && f.customColor == 1 && f.color.r == 255 && !f.color.g && !f.color.b;
}
const char* RGB::StateName(unsigned s) { return s < STATE_COUNT ? names[s] : s == STATE_COUNT ? "MANUAL" : "OFF"; }
int RGB::FindState(const char* name) {
    for (unsigned s = 0; s < STATE_COUNT; ++s) if (!std::strcmp(name, names[s])) return int(s);
    return -1;
}
uint32_t RGB::Crc(const void* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    auto bytes = static_cast<const uint8_t*>(data);
    while (length--) {
        crc ^= *bytes++;
        for (unsigned bit=0; bit<8; ++bit) crc = (crc>>1) ^ ((crc&1) ? 0xEDB88320UL : 0);
    }
    return ~crc;
}
void RGB::LoadConfig() {
    static_assert(sizeof(FlashRecord) <= 1024 && sizeof(FlashRecord)%4 == 0, "RGB flash record size");
    static_assert(offsetof(Config, autoColors) == sizeof(LegacyConfig), "RGB v1 prefix layout");
    FlashRecord record{};
    std::memcpy(&record, reinterpret_cast<const void*>(FLASH_ADDRESS), sizeof(record));
    if (record.magic == RGB_MAGIC && (record.version == RGB_VERSION || record.version == 2) && record.size == sizeof(Config) &&
        record.crc == Crc(&record, offsetof(FlashRecord, crc)) && Valid(record.config)) {
        config = savedConfig = record.config; savedValid = true; storage = 1;
        if (record.version == 2) {
            MigrateStopBinding(config);
            savedValid = false; // Upgrade in RAM; explicit save writes v3 without repeated migration.
        }
        return;
    }
    if (record.magic == RGB_MAGIC && record.version == 1 && record.size == sizeof(LegacyConfig)) {
        LegacyFlashRecord old{};
        std::memcpy(&old, reinterpret_cast<const void*>(FLASH_ADDRESS), sizeof(old));
        Config migrated = Defaults();
        std::memcpy(&migrated, &old.config, sizeof(old.config));
        bool valid = old.crc == Crc(&old, offsetof(LegacyFlashRecord, crc)) &&
            old.config.manualMode < EFFECT_COUNT && Valid(migrated);
        for (const auto& b : old.config.bindings) valid = valid && b.mode < EFFECT_COUNT && b.customColor <= 1;
        if (valid) {
            for (unsigned i=0; i<AUTO_COLOR_COUNT; ++i) {
                const auto& b = old.config.bindings[READY_SEQ+i];
                migrated.autoColors[i] = b.customColor ? b.color : b.mode < 3 ?
                    old.config.colors[b.mode] : old.config.effects[b.mode].first;
                migrated.bindings[READY_SEQ+i] = {uint8_t(AUTO_SEQ_COLOR+i), 0, {0,0,0}};
                migrated.bindings[RUN_SEQ+i] = {GREEN_SPIN, uint8_t(2+i), {0,0,0}};
            }
            MigrateStopBinding(migrated);
            config = savedConfig = migrated; storage = 1;
            savedValid = false; // Valid legacy data, but the v3 migration still needs an explicit save.
            return;
        }
    }
    config = Defaults(); savedValid = false; storage = record.magic == 0xFFFFFFFFUL ? 0 : 2;
}
bool RGB::SaveConfig() {
    static_assert(sizeof(FlashRecord) <= 1024 && sizeof(FlashRecord) % 4 == 0, "RGB slot boundary");
    FlashRecord record{};
    record.magic = RGB_MAGIC; record.version = RGB_VERSION; record.size = sizeof(Config);
    record.config = GetConfig(); record.crc = Crc(&record, offsetof(FlashRecord, crc));
    taskENTER_CRITICAL();
    bool unchanged = savedValid && !std::memcmp(&record.config, &savedConfig, sizeof(Config));
    taskEXIT_CRITICAL();
    if (unchanged) return true;
    const auto written = RewriteSharedSector(FLASH_ADDRESS, &record, sizeof(record));
    if (written == WRITE_BUSY) return false;
    const bool ok = written == WRITE_VERIFIED;
    TcpTool::RefreshStorageProof();
    taskENTER_CRITICAL();
    if (ok) { savedConfig = record.config; savedValid = true; storage = 1; }
    else { savedValid = false; storage = 2; }
    taskEXIT_CRITICAL();
    return ok;
}
void RGB::RefreshStorageProof() {
    FlashRecord record{};
    std::memcpy(&record, reinterpret_cast<const void*>(FLASH_ADDRESS), sizeof(record));
    bool valid = record.magic == RGB_MAGIC && (record.version == RGB_VERSION || record.version == 2) &&
        record.size == sizeof(Config) && record.crc == Crc(&record, offsetof(FlashRecord, crc)) && Valid(record.config);
    bool current = valid && record.version == RGB_VERSION;
    if (!valid && record.magic == RGB_MAGIC && record.version == 1 && record.size == sizeof(LegacyConfig)) {
        LegacyFlashRecord legacy{};
        std::memcpy(&legacy, reinterpret_cast<const void*>(FLASH_ADDRESS), sizeof(legacy));
        Config migrated = Defaults();
        std::memcpy(&migrated, &legacy.config, sizeof(legacy.config));
        valid = legacy.crc == Crc(&legacy, offsetof(LegacyFlashRecord, crc)) &&
            legacy.config.manualMode < EFFECT_COUNT && Valid(migrated);
        for (const auto& binding : legacy.config.bindings)
            valid = valid && binding.mode < EFFECT_COUNT && binding.customColor <= 1;
    }
    taskENTER_CRITICAL();
    savedValid = current;
    if (current) savedConfig = record.config;
    storage = valid ? 1 : record.magic == 0xFFFFFFFFUL ? 0 : 2;
    taskEXIT_CRITICAL();
}
bool RGB::SaveToolRecord(const void* record, size_t length) {
    if (!record || !length || length > 256 || length % 4) return false;
    const auto written = RewriteSharedSector(TOOL_FLASH_ADDRESS, record, length);
    if (written == WRITE_BUSY) return false;
    RefreshStorageProof();
    return written == WRITE_VERIFIED;
}
RGB::Config RGB::GetConfig(uint32_t* rev) const {
    taskENTER_CRITICAL(); Config result = config;
    if (rev) *rev = revision;
    taskEXIT_CRITICAL(); return result;
}
bool RGB::ApplyConfig(const Config& value, uint32_t expectedRevision) {
    if (!Valid(value)) return false;
    taskENTER_CRITICAL();
    bool ok = revision == expectedRevision;
    if (ok && std::memcmp(&config, &value, sizeof(Config))) { config = value; ++revision; }
    taskEXIT_CRITICAL(); return ok;
}
void RGB::SetEnabled(bool on) {
    taskENTER_CRITICAL();
    if (config.enabled != on) { config.enabled = on; ++revision; }
    taskEXIT_CRITICAL();
}
void RGB::SetManualMode(uint32_t mode) {
    if (mode >= MODE_COUNT) return;
    taskENTER_CRITICAL();
    if (config.manualMode != mode || config.automatic) {
        config.manualMode = uint8_t(mode); config.automatic = 0; ++revision;
    }
    taskEXIT_CRITICAL();
}
void RGB::EndBoot() {
    taskENTER_CRITICAL(); bootPending = false; taskEXIT_CRITICAL();
}
RGB::Status RGB::GetStatus(State state) const {
    taskENTER_CRITICAL(); Status result = status;
    if (state == STATE_COUNT) state = State(status.state);
    if (state >= STATE_COUNT) state = FAULT;
    const State visible = bootPending && config.automatic && state == DISABLED ? BOOT : state;
    const Scene selected = SelectScene(config, visible);
    result.state = state; result.mode = selected.mode;
    result.source = !config.enabled || !config.brightness ? STATE_COUNT+1 : selected.source;
    result.dirty = !savedValid || std::memcmp(&config, &savedConfig, sizeof(Config));
    result.storage = storage; result.dmaBusy = dmaBusy;
    result.frames = frames; result.errors = errors; result.timeouts = timeouts;
    taskEXIT_CRITICAL(); return result;
}
float RGB::Smooth(float t) { return t>=1 ? 1 : t<=0 ? 0 : t*t*(3-2*t); }
float RGB::Phase(uint32_t elapsed, uint32_t period) { return float(elapsed%period)/float(period); }
bool RGB::SameScene(const Scene& a, const Scene& b) {
    return a.mode==b.mode && a.source==b.source && a.scale==b.scale &&
        !std::memcmp(&a.solid, &b.solid, sizeof(Color)) && !std::memcmp(&a.effect, &b.effect, sizeof(Effect));
}
RGB::Scene RGB::SelectScene(const Config& c, State state) {
    Scene s{};
    bool overlay = state==FAULT || state==STOP || state==CALIBRATION || state==STARTING;
    const auto& binding = c.bindings[state];
    s.source = c.automatic || overlay ? uint8_t(state) : uint8_t(STATE_COUNT);
    s.mode = s.source==STATE_COUNT ? c.manualMode : binding.mode;
    s.effect = c.effects[s.mode<EFFECT_COUNT ? s.mode : 0];
    s.solid = s.mode<3 ? c.colors[s.mode] : s.mode>=AUTO_SEQ_COLOR ? c.autoColors[s.mode-AUTO_SEQ_COLOR] : s.effect.first;
    if (s.source!=STATE_COUNT && binding.customColor)
        s.solid = s.effect.first = binding.customColor==1 ? binding.color : c.autoColors[binding.customColor-2];
    s.scale = s.source==DISABLED ? 0.15f : s.source>=READY_SEQ && s.source<=READY_TUN ? 0.25f : 1;
    if (state==FAULT) {
        s.mode = RED_HEARTBEAT; s.solid = {255,0,0}; s.effect = Defaults().effects[RED_HEARTBEAT];
    } else if (state==STOP && s.mode==RED_HEARTBEAT) {
        // Match the fault heartbeat timing while retaining the STOP binding's color.
        s.effect = Defaults().effects[RED_HEARTBEAT]; s.effect.first = s.solid;
    }
    return s;
}
void RGB::Render(const Scene& s, uint32_t elapsed, float output[LED_COUNT][3]) {
    float p = Phase(elapsed, s.effect.periodMs);
    if (s.effect.reverse) p = 1-p;
    float amplitude = 1, mix = 0;
    switch (s.mode) {
        case BLUE_TIDE: amplitude = 0.08f+0.52f*(0.5f+0.5f*cosf(2*PI_F*p)); break;
        case WHITE_BREATH:
        case CYBER_BREATH: amplitude = 0.02f+0.98f*(0.5f-0.5f*cosf(2*PI_F*p)); break;
        case RED_HEARTBEAT:
            amplitude = p<0.16f ? 0.04f+0.96f*sinf(PI_F*p/0.16f) :
                p>=0.20f && p<0.36f ? 0.04f+0.74f*sinf(PI_F*(p-0.20f)/0.16f) : 0.04f;
            break;
        case BLINK: amplitude = elapsed%(uint32_t(s.effect.onMs)+s.effect.offMs)<s.effect.onMs ? 1 : 0; break;
        default: break;
    }
    if (s.mode==CYBER_BREATH) mix = 0.5f-0.5f*cosf(2*PI_F*p);
    for (unsigned i=0; i<LED_COUNT; ++i) {
        float color[3] = {float(s.solid.r), float(s.solid.g), float(s.solid.b)}, level = amplitude;
        if (s.mode==RAINBOW) {
            float hue = p+float(i)/LED_COUNT; hue -= floorf(hue);
            float x=hue*6, f=x-floorf(x);
            switch (unsigned(x)) {
                case 0: color[0]=255; color[1]=255*f; color[2]=0; break;
                case 1: color[0]=255*(1-f); color[1]=255; color[2]=0; break;
                case 2: color[0]=0; color[1]=255; color[2]=255*f; break;
                case 3: color[0]=0; color[1]=255*(1-f); color[2]=255; break;
                case 4: color[0]=255*f; color[1]=0; color[2]=255; break;
                default: color[0]=255; color[1]=0; color[2]=255*(1-f); break;
            }
        } else if (s.mode==CYBER_BREATH) {
            color[0]+=(float(s.effect.second.r)-color[0])*mix;
            color[1]+=(float(s.effect.second.g)-color[1])*mix;
            color[2]+=(float(s.effect.second.b)-color[2])*mix;
        } else if (s.mode==GREEN_SPIN) {
            float head=p*LED_COUNT, distance=s.effect.reverse ? float(i)-head : head-float(i);
            if (distance<0) distance+=LED_COUNT;
            if (distance>=LED_COUNT) distance-=LED_COUNT;
            level=0.04f;
            if (distance<s.effect.tail) level+=0.96f*Smooth(1-distance/s.effect.tail);
        }
        for (unsigned ch=0; ch<3; ++ch) output[i][ch]=color[ch]*level*s.scale;
    }
}
void RGB::Update(State state, uint32_t now) {
    Config c=GetConfig();
    if (state>=STATE_COUNT) state=FAULT;
    taskENTER_CRITICAL();
    if (state!=DISABLED) bootPending=false;
    const State visible=bootPending && c.automatic && state==DISABLED ? BOOT : state;
    taskEXIT_CRITICAL();
    Scene next=SelectScene(c, visible);
    if (!sceneValid || !SameScene(scene, next)) {
        std::memcpy(fromFrame, frame, sizeof(frame)); scene=next; sceneAt=blendAt=now;
        blendMs=!sceneValid || visible==FAULT || visible==STOP ? 0 : c.transitionMs;
        sceneValid=true;
    }
    Render(scene, now-sceneAt, frame);
    float blend=blendMs ? Smooth(float(now-blendAt)/blendMs) : 1;
    for (unsigned i=0; i<LED_COUNT; ++i)
        for (unsigned ch=0; ch<3; ++ch) frame[i][ch]=fromFrame[i][ch]+(frame[i][ch]-fromFrame[i][ch])*blend;
    float progress=rampMs ? Smooth(float(now-rampAt)/rampMs) : 1;
    brightness=rampFrom+(rampTarget-rampFrom)*progress;
    float target=c.enabled ? c.brightness/100.0f : 0;
    if (target!=rampTarget || bool(c.enabled)!=lastEnabled) {
        rampFrom=brightness; rampTarget=target; rampAt=now;
        rampMs=bool(c.enabled)!=lastEnabled ? (c.enabled ? c.fadeOnMs : c.fadeOffMs) : c.brightnessMs;
        lastEnabled=c.enabled;
        if (!rampMs) brightness=target;
    }
    taskENTER_CRITICAL();
    status.state=state; status.mode=scene.mode;
    status.source=!c.enabled || !c.brightness ? STATE_COUNT+1 : scene.source;
    status.currentBrightness=uint8_t(brightness*100+0.5f);
    taskEXIT_CRITICAL();
    if (dmaBusy && now-dmaStarted>10) {
        taskENTER_CRITICAL();
        if (dmaBusy) {
            HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
            dmaBusy=false; wireValid=false; ++timeouts;
        }
        taskEXIT_CRITICAL();
    }
    if (!dmaBusy) SendFrame(now);
}
void RGB::SendFrame(uint32_t now) {
    uint8_t wire[LED_COUNT*3]{}; float gain=Gain(brightness);
    for (unsigned i=0; i<LED_COUNT; ++i) {
        wire[i*3]=uint8_t(frame[i][1]*gain+0.5f); // GRB wire order only.
        wire[i*3+1]=uint8_t(frame[i][0]*gain+0.5f); wire[i*3+2]=uint8_t(frame[i][2]*gain+0.5f);
    }
    if (wireValid && !std::memcmp(wire, lastWire, sizeof(wire))) return;
    unsigned index=0;
    for (uint8_t byte : wire)
        for (int bit=7; bit>=0; --bit) pwmData[index++]=byte&(1U<<bit) ? 70 : 30;
    while (index<sizeof(pwmData)/sizeof(pwmData[0])) pwmData[index++]=0;
    taskENTER_CRITICAL();
    dmaStarted=now; dmaBusy=true; // Before HAL start, protected against a completion ISR.
    __HAL_TIM_SET_COUNTER(&htim2, 0); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_StatusTypeDef result=HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, pwmData, index);
    if (result==HAL_OK) { std::memcpy(lastWire, wire, sizeof(wire)); wireValid=true; ++frames; }
    else {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4); __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        dmaBusy=false; wireValid=false; ++errors;
    }
    taskEXIT_CRITICAL();
}
void RGB::Interrupt(uint8_t flag) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
    if (flag!=1) { ++errors; wireValid=false; }
    dmaBusy=false;
}


