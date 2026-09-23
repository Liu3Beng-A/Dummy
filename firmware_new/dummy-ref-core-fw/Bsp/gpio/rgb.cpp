#include "rgb.hpp"

RGB::RGB(uint8_t m) {
    //init here
    dmaEvent = nullptr;
    dmaBusy = false;
    firstSend = true;
}

void RGB::InitSync()
{
    if (dmaEvent == nullptr)
    {
        dmaEvent = osEventFlagsNew(nullptr);
        dmaBusy = false;
        firstSend = true;
    }
}

bool RGB::UpdateLoop()
{
    if (dmaEvent == nullptr) return true;  // 未初始化（兼容旧调用）
    if (firstSend) { firstSend = false; return true; }

    // 非阻塞等待 0 ms，仅查询
    uint32_t flags = osEventFlagsWait(dmaEvent, 0x01, osFlagsNoClear, 0);
    if (flags & 0x01)
    {
        osEventFlagsClear(dmaEvent, 0x01);
        dmaBusy = false;
        return true;
    }
    // DMA 未完成：保持 dmaBusy = true，Run() 会跳过 WS2812_Send()
    return false;
}

void RGB::Run(RGB::Rgb_style_t mode) {
    switch (mode) {
        case PURE_COLOR_0: {
            ShowPureColor(0);
            break;
        }
        case PURE_COLOR_1: {
            ShowPureColor(1);
            break;
        }
        case PURE_COLOR_2: {
            ShowPureColor(2);
            break;
        }
        case RAINBOW: {
            SmoothRainbow();
            break;
        }
        case BLUE_TIDE: {
            BlueTide();
            break;
        }
        case WHITE_BREATH: {
            WhiteBreath();
            break;
        }
        case CYBER_BREATH: {
            CyberBreath();
            break;
        }
        case RED_HEARTBEAT: {
            RedHeartbeat();
            break;
        }
        case GREEN_SPIN: {
            GreenSpin();
            break;
        }
        case BLINK: {
            Blink();
            break;
        }
        case ALL_OFF: {
            for(uint16_t j=0; j<MAX_LED; j++) {
                Set_LED(j, 0, 0, 0);
            }
            WS2812_Send();
            break;
        }
        default:
            break;
    }
    FadeStep();
}

void RGB::FadeStep() {
    if (fabsf(brightness - targetBrightness) < 0.01f) {
        brightness = targetBrightness;
    } else {
        if (targetBrightness > brightness) {
            brightness += fadeSpeed;
            if (brightness > targetBrightness) brightness = targetBrightness;
        } else {
            brightness -= fadeSpeed;
            if (brightness < targetBrightness) brightness = targetBrightness;
        }
    }
}

void RGB::Interrupt(uint8_t flag) {
        data_sentflag = flag;
        // 由 DMA 完成中断回调：
        // 释放 dmaBusy，并通过事件标志唤醒等待 UpdateLoop() 的任务
        dmaBusy = false;
        if (dmaEvent != nullptr)
        {
            osEventFlagsSet(dmaEvent, 0x01);
        }
    }


