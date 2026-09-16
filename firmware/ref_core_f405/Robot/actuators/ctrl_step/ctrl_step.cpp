#include "ctrl_step.hpp"
#include "communication.hpp"


CtrlStepMotor::CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
    nodeID(_id), hcan(_hcan), inverseDirection(_inverse), reduction(_reduction),
    angleLimitMin(_angleLimitMin), angleLimitMax(_angleLimitMax)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void CtrlStepMotor::SetEnable(bool _enable)
{
    // [F1 修复] enable 时不再同步 state=FINISH（避免与 A1 方向冲突）
    // 等待电机端首次 0x23 回包自动纠正 state。
    // disable 时同步 STOP，让 IsMoving() 在 disable 后能正确返回 false。
    if (!_enable) state = STOP;
    // [A3 修复] 不再清 targetAngle，jointsStateFlag 方案不再依赖 targetAngle 字段

    uint8_t mode = 0x01;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::SetEnableTemp(bool _enable)
{
    uint8_t mode = 0x7d;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x03;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x04;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    uint8_t mode = 0x05;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionWithMotorRps(float _pos, float _rps)
{
    uint8_t mode = 0x07;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_pos;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    b = (unsigned char*) &_rps;
    for (int i = 4; i < 8; i++)
        canBuf[i] = *(b + i - 4);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAngleWithMotorRps(float _angle, float _rps)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithMotorRps(stepMotorCnt, _rps);
}

void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    uint8_t mode = 0x11;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // 默认持久化（fiber 协议调用路径）

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::SetCurrentLimit_persist(float _val, bool persist)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = persist ? 1 : 0;  // B2 修复：按参数决定是否写 EEPROM

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    uint8_t mode = 0x13;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAcceleration(float _val)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // 默认不写 EEPROM（fiber 协议调用路径）

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::SetAcceleration_persist(float _val, bool persist)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = persist ? 1 : 0;  // 按参数决定是否写 EEPROM

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::QueryCurrentLimit()
{
    uint8_t mode = 0x2D;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::QueryAcceleration()
{
    uint8_t mode = 0x2C;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}


void CtrlStepMotor::ApplyPositionAsHome()
{
    uint8_t mode = 0x15;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
    uint8_t mode = 0x16;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    uint8_t mode = 0x1B;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::Reboot()
{
    uint8_t mode = 0x7f;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

uint32_t CtrlStepMotor::GetTemp()
{
    uint8_t mode = 0x25;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
    return temperature;
}

void CtrlStepMotor::EraseConfigs()
{
    uint8_t mode = 0x7e;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}




void CtrlStepMotor::UpdateAngle()
{
    uint8_t mode = 0x23;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}


void CtrlStepMotor::SetStallMode()
{
    state = STALL;
}


void CtrlStepMotor::SetDceKp(int32_t _val)
{
    uint8_t mode = 0x17;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKv(int32_t _val)
{
    uint8_t mode = 0x18;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKi(int32_t _val)
{
    uint8_t mode = 0x19;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKd(int32_t _val)
{
    uint8_t mode = 0x1A;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::QueryDceKp()
{
    uint8_t mode = 0x28;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::QueryDceKv()
{
    uint8_t mode = 0x29;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::QueryDceKi()
{
    uint8_t mode = 0x2A;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::QueryDceKd()
{
    uint8_t mode = 0x2B;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::BroadcastUnlock()
{
    // 广播 UNLOCKED: StdId = (0 << 7) | 0x5B = 0x5B (落入电机端广播区间 0x50~0x7F)
    txHeader.StdId = 0x5B;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::BroadcastEStop()
{
    // [v3.2] 广播 Emergency Stop: StdId = (nodeID << 7) | 0x09
    //   motorJ[ALL] (nodeID=0) 调用时 → StdId = (0<<7)|0x09 = 0x09（普通命令空间）
    //   由电机端 `id == 0` 路由（阶段 1）识别为 ALL 广播
    //   电机端收到后（case 0x09）：requestMode = MODE_STOP + SetBrake(true) + 清速度/电流
    //   物理上立即抱闸锁死在当前位置，不再继续梯形规划
    //   旧版用硬编码 0x89 会被电机端广播区间 0x80~0xBF 路由抢走，与 ALL 代理不兼容
    txHeader.StdId = (nodeID << 7) | 0x09;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

void CtrlStepMotor::QueryStallStatus(uint8_t queryType)
{
    // 单播查询 stall 状态: StdId = (nodeID << 7) | 0x5C (落入电机端广播区间 0x50~0x7F)
    txHeader.StdId = (nodeID << 7) | 0x5C;
    uint8_t buf[8] = { queryType, 0, 0, 0, 0, 0, 0, 0 };
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}
