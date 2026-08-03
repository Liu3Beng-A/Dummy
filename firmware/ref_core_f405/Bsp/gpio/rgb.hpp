#ifndef REF_STM32F4_RGB_H
#define REF_STM32F4_RGB_H

#include <cstdint>
#include <tim.h>
#include <math.h>
#include "cmsis_os.h"

#define MAX_LED 24
#define USE_BRIGHTNESS 0
#define PI               3.14159265358979f

class RGB
{
private:
    uint8_t LED_Data[MAX_LED][4];
    uint8_t LED_Mod[MAX_LED][4];  // for brightness
    uint32_t pwmData[(24*MAX_LED)+50];
    uint16_t  effStep = 0;

    // FreeRTOS 事件标志：DMA 传输完成时由中断置位，UpdateLoop 消费后清零
    osEventFlagsId_t dmaEvent = nullptr;
    bool dmaBusy = false;        // 是否有 DMA 正在传输
    bool firstSend = true;       // 启动后首次发送标志（无需等待 DMA）

public:
    uint8_t  data_sentflag = 0;  // 兼容旧 Interrupt(1) 的回调接口

    enum Rgb_style_t
    {
        PURE_COLOR_0 = 0,  // <--- 0: 静态单色0
        PURE_COLOR_1 = 1,  // <--- 1: 静态单色1
        PURE_COLOR_2 = 2,  // <--- 2: 静态单色2
        RAINBOW = 3,       // <--- 3: 平滑流光彩虹
        BLUE_TIDE = 4,     // <--- 4: 蓝色潮汐
        WHITE_BREATH = 5,  // <--- 5: 白色呼吸
        CYBER_BREATH = 6,  // <--- 6: 赛博呼吸
        RED_HEARTBEAT = 7, // <--- 7: 红色心跳
        GREEN_SPIN = 8,    // <--- 8: 绿色旋转
        BLINK = 9,         // <--- 9: 闪烁特效
        ALL_OFF = 99       // <--- 99: 彻底关闭所有灯光
    };

    // 保存3个纯色模式下的RGB值
    uint8_t static_r[3] = {0, 0, 255};
    uint8_t static_g[3] = {0, 255, 255};
    uint8_t static_b[3] = {255, 0, 255};

    // 全局亮度渐变控制（不影响灯效模式本身）
    float brightness = 1.0f;        // 当前亮度 0.0~1.0
    float targetBrightness = 1.0f;  // 目标亮度，渐变到此值
    float fadeSpeed = 0.015f;      // 每步渐变步进量（约2秒完成全范围 0↔100%）
    float savedBrightness = 1.0f;  // 保存上次亮度，关灯后再开渐变到此值


    RGB(uint8_t mode=0);

    void InitSync();        // 初始化事件标志（必须在启动 RTOS 任务前调用）
    void Run(Rgb_style_t _mode = RAINBOW);

    void Interrupt(uint8_t flag);

    void FadeStep();

    // 非阻塞检查：DMA 是否已完成上一次发送。
    // 必须在调用 Run() 前先调用本函数，否则不会启动新的 WS2812_Send()。
    // 返回 true 表示 DMA 空闲可以发送新数据。
    bool UpdateLoop();

    //functions
    void Set_LED (uint8_t LEDs, uint8_t Red, uint8_t Green, uint8_t Blue)
    {
        LED_Data[LEDs][0] = LEDs;
        LED_Data[LEDs][1] = Green;
        LED_Data[LEDs][2] = Red;
        LED_Data[LEDs][3] = Blue;
    }

    void Set_Brightness (int brightness)  // 0-45
    {
#if USE_BRIGHTNESS

        if (brightness > 45) brightness = 45;
        for (int i=0; i<MAX_LED; i++)
        {
            LED_Mod[i][0] = LED_Data[i][0];
            for (int j=1; j<4; j++)
            {
                float angle = 105-brightness;  // in degrees
                angle = angle*PI / 180;  // in rad
                LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
            }
        }

#endif

    }

    void WS2812_Send (void)
    {
        uint32_t indx=0;
        uint32_t color;
        HAL_StatusTypeDef ret;

        // 保护：上一次 DMA 还没完成则跳过本帧，绝不阻塞
        if (dmaBusy)
        {
            return;
        }

        for (int i= 0; i<MAX_LED; i++)
        {
            uint8_t r = LED_Data[i][1];
            uint8_t g = LED_Data[i][2];
            uint8_t b = LED_Data[i][3];
            color = (((uint32_t)(r * brightness) << 16)
                  | ((uint32_t)(g * brightness) << 8)
                  |  (uint32_t)(b * brightness));
            for (int i=23; i>=0; i--)
            {
                //1 timing
                if (color&(1<<i))
                {
                    pwmData[indx] = 70;  // 2/3 of 105
                }
                //0 timing
                else pwmData[indx] = 30;  // 1/3 of 105
                indx++;
            }

        }

        //for reset between two data cycle
        for (int i=0; i<30; i++)
        {
            pwmData[indx] = 0;
            indx++;
        }

        ret = HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, pwmData, indx);
        if (ret != HAL_OK) {
            data_sentflag = 0;
            // 失败也视作空闲，唤醒 UpdateLoop
            if (dmaEvent != nullptr)
                osEventFlagsSet(dmaEvent, 0x01);
            return;
        }
        // 标记 DMA 正在传输
        dmaBusy = true;
        // 不再阻塞等待：DMA 完成中断会通过 Interrupt(1) 释放 dmaBusy 并置事件标志
    }

    // 辅助函数：色轮生成 (0-255 输入 -> R,G,B 输出)
    void Wheel(uint8_t WheelPos, uint8_t &r, uint8_t &g, uint8_t &b) {
        WheelPos = 255 - WheelPos;
        if(WheelPos < 85) {
            r = 255 - WheelPos * 3;
            g = 0;
            b = WheelPos * 3;
        } else if(WheelPos < 170) {
            WheelPos -= 85;
            r = 0;
            g = WheelPos * 3;
            b = 255 - WheelPos * 3;
        } else {
            WheelPos -= 170;
            r = WheelPos * 3;
            g = 255 - WheelPos * 3;
            b = 0;
        }
    }

// 主函数：平滑流光
    void SmoothRainbow() {
        uint8_t r, g, b;
        for(uint16_t i=0; i<MAX_LED; i++) {
            // 这里的 i*20 决定了彩虹在环上的密度
            // effStep * 2 决定了旋转速度
            Wheel(((i * 256 / MAX_LED) + effStep * 2) & 255, r, g, b);
            Set_LED(i, r, g, b);
        }
        WS2812_Send();
        effStep++;
    }

    void Sequence() {
        //define your color style here
    }

    void Blink()
    {
        uint8_t e,r,g,b;
        if(effStep < 44) {
            // 所有灯一起设黑，一次性发送（之前每灯一次共 24 次）
            for(uint16_t j=0;j<MAX_LED;j++)
            {
                Set_LED(j, 0, 0, 0);
            }
            WS2812_Send();
        }
        else if(effStep  < 46) {
            e = (effStep * 5) - 220;
            r = 255 * ( e / 10 ) + 0 * ( 1.0 - e / 10 );
            g = 255 * ( e / 10 ) + 0 * ( 1.0 - e / 10 );
            b = 255 * ( e / 10 ) + 0 * ( 1.0 - e / 10 );
            // (j%1)==0 永远为真，等效于全部灯点亮 → 一次发送即可
            for(uint16_t j=0;j<MAX_LED;j++)
            {
                Set_LED(j, r, g, b);
            }
            WS2812_Send();
        }
        else if(effStep < 53.6) {
            // 同上：所有灯全亮，批量发送
            for(uint16_t j=0;j<MAX_LED;j++)
            {
                Set_LED(j, 255, 255, 255);
            }
            WS2812_Send();
        }
        else if(effStep < 55.6) {
            e = (effStep * 5) - 268;
            r = 0 * ( e / 10 ) + 255 * ( 1.0 - e / 10 );
            g = 0 * ( e / 10 ) + 255 * ( 1.0 - e / 10 );
            b = 0 * ( e / 10 ) + 255 * ( 1.0 - e / 10 );
            for(uint16_t j=0;j<MAX_LED;j++)
            {
                Set_LED(j, r, g, b);
            }
            WS2812_Send();
        }
        else {
            for(uint16_t j=0;j<MAX_LED;j++)
            {
                Set_LED(j, 0, 0, 0);
            }
            WS2812_Send();
        }
        if(effStep >= 75.6) {effStep = 0; }
        else effStep++;
    }


    void WhiteBreath() {
        // 利用正弦波产生 0.0 到 1.0 的呼吸曲线
        float val = (exp(sin(effStep * 0.05f)) - 0.367879441f) * 108.0f;
        
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        uint8_t brightness = (uint8_t)val;

        for(int i=0; i<MAX_LED; i++) {
            // R=G=B 产生白色
            Set_LED(i, brightness, brightness, brightness); 
        }
        
        WS2812_Send();
        effStep++;
    }

    void ShowPureColor(uint8_t idx) {
        if(idx > 2) idx = 0;
        for(uint16_t j=0; j<MAX_LED; j++) {
            Set_LED(j, static_r[idx], static_g[idx], static_b[idx]);
        }
        WS2812_Send();
    }

    void CyberBreath() {
        // 利用 sin 函数产生 0.0 到 1.0 的呼吸曲线
        // 0.05f 调节呼吸速度
        float val = (exp(sin(effStep * 0.05f)) - 0.367879441f) * 108.0f; 
        
        // 限制范围防止溢出
        if (val < 0) val = 0;
        if (val > 255) val = 255;
        uint8_t brightness = (uint8_t)val;

        // 颜色混合：当亮度高时偏青色，亮度低时偏紫色
        // 你可以自己调整 R G B 的比例
        for(int i=0; i<MAX_LED; i++) {
            // R: 紫色分量, G: 青色分量, B: 蓝色基底
            // 这种混合会让颜色看起来在“流动”
            Set_LED(i, brightness/2, brightness, brightness); 
        }
        
        WS2812_Send();
        effStep++;
    }


    void RedHeartbeat() {
        // 模拟心跳的节奏：两次快速跳动，然后长停顿
        // 我们把周期设为 60 步
        int step = effStep % 60;
        float brightness = 0;

        // 第一跳 (0-10)
        if (step < 10) {
            brightness = sin(step * PI / 10.0f) * 255;
        }
        // 第二跳 (12-22) - 稍微小一点
        else if (step >= 12 && step < 22) {
            brightness = sin((step - 12) * PI / 10.0f) * 200;
        }
        // 其他时间 (休息)
        else {
            brightness = 10; // 保持微亮，不要全黑
        }

        if (brightness < 0) brightness = 0;
        
        // 设置红色
        for(int i=0; i<MAX_LED; i++) {
            Set_LED(i, (uint8_t)brightness, 0, 0);
        }
        
        WS2812_Send();
        // 如果觉得心跳太快，可以把 effStep++ 放到每隔几次循环才执行
        effStep++; 
    }


    void GreenSpin() {
        // 1. 设置背景为暗绿色，表示系统在线
        for(int i=0; i<MAX_LED; i++) {
             // R=0, G=20 (暗绿), B=0
            Set_LED(i, 0, 20, 0);
        }

        // 2. 计算高亮光标的位置 (旋转)
        // 0.5f 控制旋转速度
        int head = (int)(effStep * 0.5f) % MAX_LED;

        // 3. 绘制光标 (3个灯珠宽)
        for (int i = 0; i < 3; i++) {
            int pos = (head + i) % MAX_LED;
            // 越靠近头部越亮
            int b = 100 + i * 50; 
            Set_LED(pos, 0, b, 0); // 高亮绿
        }

        WS2812_Send();
        effStep++;
    }


    void BlueTide() {
        // 使用 cos 函数创造平滑的波浪
        // 0.05f 极慢的速度
        float val = (cos(effStep * 0.05f) + 1.0f) / 2.0f; // 结果 0.0 ~ 1.0
        
        // 亮度在 20 (微亮) 到 150 (高亮) 之间波动，不完全熄灭
        uint8_t b = 20 + (uint8_t)(val * 130);

        for(int i=0; i<MAX_LED; i++) {
            Set_LED(i, 0, 0, b);
        }

        WS2812_Send();
        effStep++;
    }



    void PureColor(Rgb_style_t color) {
        // 此函数不再使用，由 ShowPureColor() 替代，但保留空实现防报错
    }

};

#endif //REF_STM32F4_RGB_H

