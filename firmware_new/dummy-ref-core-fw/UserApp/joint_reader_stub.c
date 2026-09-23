// 占位实现：返回 6 个关节角（度）。请替换为真实的编码器读取实现。
#include <string.h>

void ReadJointAnglesDeg(float out[6])
{
    // TODO: 从你的编码器/电机控制器读取实际关节角（度），并填入 out[0..5]
    // 当前 stub 返回 0,0,0,0,0,0
    if (out)
    {
        for (int i = 0; i < 6; ++i) out[i] = 0.0f;
    }
}