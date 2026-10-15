#ifndef REF_STM32F4_RGB_H
#define REF_STM32F4_RGB_H
#include <cstddef>
#include <cstdint>
#include <tim.h>

// Only the RGB task owns rendering/PWM storage. Commands exchange small snapshots.
class RGB {
public:
    static constexpr unsigned LED_COUNT = 24, PRESET_MODE_COUNT = 13, MODE_COUNT = 17,
        EFFECT_COUNT = 10, AUTO_COLOR_COUNT = 3, MOTION_COUNT = 4;
    enum Rgb_style_t : uint8_t {
        PURE_COLOR_0, PURE_COLOR_1, PURE_COLOR_2, RAINBOW, BLUE_TIDE,
        WHITE_BREATH, CYBER_BREATH, RED_HEARTBEAT, GREEN_SPIN, BLINK,
        AUTO_SEQ_COLOR, AUTO_INT_COLOR, AUTO_TRJ_COLOR,
        MOTION_CIRCLE, MOTION_ARC, MOTION_WAVE, MOTION_CONVERGE
    };
    enum State : uint8_t {
        BOOT, DISABLED, READY_SEQ, READY_INT, READY_TRJ, READY_TUN,
        RUN_SEQ, RUN_INT, RUN_TRJ, RUN_TUN, CALIBRATION, STOP, STARTING, FAULT, STATE_COUNT
    };
    struct Color { uint8_t r, g, b; };
    struct Effect {
        Color first, second;
        uint16_t periodMs, onMs, offMs;
        uint8_t reverse, tail;
    };
    // color source: 0=preset/effect, 1=explicit RGB, 2/3/4=auto SEQ/INT/TRJ bank.
    struct Binding { uint8_t mode, customColor; Color color; };
    struct Motion { uint16_t periodMs; uint8_t reverse, width; };
    struct Config {
        uint8_t enabled, automatic, brightness, manualMode;
        uint16_t fadeOnMs, fadeOffMs, brightnessMs, transitionMs;
        Color colors[3];
        Effect effects[EFFECT_COUNT];
        Binding bindings[STATE_COUNT];
        Color autoColors[AUTO_COLOR_COUNT];
        Motion motion[MOTION_COUNT]; // RUN_SEQ/INT/TRJ, CALIBRATION; independent of preset 8.
    };
    struct Status {
        uint8_t mode, state, source; // state index, STATE_COUNT=manual, +1=off
        uint8_t currentBrightness, dirty, storage; // storage 0=defaults,1=valid,2=invalid
        bool dmaBusy;
        uint32_t frames, errors, timeouts;
    };
    explicit RGB(uint8_t = 0);
    static Config Defaults();
    static bool Valid(const Config& value);
    static const char* StateName(unsigned state);
    static int FindState(const char* name);
    static int MotionSlot(unsigned state);
    void LoadConfig(); // Before communication/RGB tasks.
    bool SaveConfig(); // Caller holds the robot's disabled-only save reservation.
    // The same reservation protects the companion TCP slot. Each save preserves
    // the other slot's actual persisted bytes, including legacy RGB records.
    bool SaveToolRecord(const void* record, size_t length);
    Config GetConfig(uint32_t* revision = nullptr) const;
    bool ApplyConfig(const Config& value, uint32_t expectedRevision);
    void SetEnabled(bool enabled);
    void SetManualMode(uint32_t mode);
    void EndBoot(); // Accepted robot operations end the boot scene, even between RGB frames.
    // mode/source reflect accepted settings; currentBrightness is the rendered ramp.
    Status GetStatus(State state = STATE_COUNT) const;
    void Update(State state, uint32_t nowMs);
    void Interrupt(uint8_t flag); // 1=complete,2=error; only TIM2/CH4 ISR.
private:
    static constexpr unsigned RESET_SLOTS = 256; // 320us low at 800kHz
    static constexpr uint32_t FLASH_ADDRESS = 0x080FFC00UL;
    struct FlashRecord {
        uint32_t magic;
        uint16_t version, size;
        Config config;
        uint32_t crc;
    };
    struct LegacyConfig {
        uint8_t enabled, automatic, brightness, manualMode;
        uint16_t fadeOnMs, fadeOffMs, brightnessMs, transitionMs;
        Color colors[3];
        Effect effects[EFFECT_COUNT];
        Binding bindings[STATE_COUNT];
    };
    struct LegacyFlashRecord {
        uint32_t magic;
        uint16_t version, size;
        LegacyConfig config;
        uint32_t crc;
    };
    struct PreviousConfig { // v2/v3; keep the persisted prefix and its padding.
        uint8_t enabled, automatic, brightness, manualMode;
        uint16_t fadeOnMs, fadeOffMs, brightnessMs, transitionMs;
        Color colors[3];
        Effect effects[EFFECT_COUNT];
        Binding bindings[STATE_COUNT];
        Color autoColors[AUTO_COLOR_COUNT];
    };
    struct PreviousFlashRecord {
        uint32_t magic;
        uint16_t version, size;
        PreviousConfig config;
        uint32_t crc;
    };
    struct Scene {
        uint8_t mode, source;
        Color solid;
        Effect effect;
        float scale;
    };
    Config config{}, savedConfig{};
    uint32_t revision = 0;
    bool savedValid = false;
    uint8_t storage = 0;
    volatile bool dmaBusy = false, wireValid = false;
    volatile uint32_t errors = 0;
    uint32_t frames = 0, timeouts = 0, dmaStarted = 0;
    alignas(4) uint32_t pwmData[LED_COUNT * 24 + RESET_SLOTS]{};
    uint8_t lastWire[LED_COUNT * 3]{};
    float frame[LED_COUNT][3]{}, fromFrame[LED_COUNT][3]{};
    Scene scene{};
    bool sceneValid = false, bootPending = true;
    uint32_t sceneAt = 0, blendAt = 0;
    uint16_t blendMs = 0;
    float brightness = 0, rampFrom = 0, rampTarget = 0;
    uint32_t rampAt = 0;
    uint16_t rampMs = 0;
    bool lastEnabled = false;
    Status status{};
    static uint32_t Crc(const void* data, size_t length);
    static uint8_t ReadSavedConfig(Config& value, bool& current);
    void RefreshStorageProof();
    static float Smooth(float t);
    static float Phase(uint32_t elapsed, uint32_t period);
    static bool SameScene(const Scene& a, const Scene& b);
    static Scene SelectScene(const Config& value, State state);
    void Render(const Scene& value, uint32_t elapsed, float output[LED_COUNT][3]);
    void SendFrame(uint32_t nowMs);
};
#endif

