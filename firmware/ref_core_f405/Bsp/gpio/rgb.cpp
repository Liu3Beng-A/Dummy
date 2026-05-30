#include "rgb.hpp"

RGB::RGB(uint8_t m) {
    //init here
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
}

void RGB::Interrupt(uint8_t flag) {
    data_sentflag = flag;
}


