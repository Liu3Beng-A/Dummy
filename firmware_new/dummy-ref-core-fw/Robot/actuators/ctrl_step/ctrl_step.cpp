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
    state = _enable ? FINISH : STOP;
    // STOP control traffic uses local frames, independent of periodic query buffers.
    CAN_TxHeaderTypeDef header = txHeader;
    header.StdId = (nodeID << 7) | 0x01;
    uint8_t data[8] = {};
    uint32_t val = _enable ? 1 : 0;
    memcpy(data, &val, sizeof(val));
    CanSendMessage(get_can_ctx(hcan), data, &header);
}

bool CtrlStepMotor::SendSoftStop(uint8_t action, uint16_t token)
{
    CAN_TxHeaderTypeDef header = txHeader;
    header.StdId = (nodeID << 7) | SoftStopProtocol::COMMAND;
    uint8_t data[8] = {action, 0, static_cast<uint8_t>(token),
                       static_cast<uint8_t>(token >> 8), 0, 0, 0, 0};
    return TryCanSendMessage(get_can_ctx(hcan), data, &header);
}

bool CtrlStepMotor::TrySendProtection(uint8_t command, const uint8_t* data)
{
    CAN_TxHeaderTypeDef header = txHeader;
    header.StdId = (nodeID << 7) | command;
    uint8_t frame[8]; memcpy(frame, data, 8);
    return TryCanSendMessage(get_can_ctx(hcan), frame, &header);
}

void CtrlStepMotor::SetEnableTemp(bool _enable)
{
    uint8_t mode = 0x7d;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::DoCalibration()
{
    uint8_t mode = 0x02;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x03;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
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

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    _val = inverseDirection ? -_val : _val;  // [FIX] inverseDirection 下放到最底层

    uint8_t mode = 0x05;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need ACK

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionWithVelocityLimit(float position, float velocity)
{
    position = inverseDirection ? -position : position;
    CAN_TxHeaderTypeDef header = {};
    header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8;
    header.StdId = (nodeID << 7) | 0x07;
    uint8_t data[8]; memcpy(data, &position, 4); memcpy(data + 4, &velocity, 4);
    CanSendMessage(get_can_ctx(hcan), data, &header);
}



// [Step 6] 与 ref 对齐：补 SetPositionWithMotorRps（去掉 inverse 取反，由 SetAngleWithMotorRps 在外层统一处理）
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


// [Step 6] 与 ref 对齐：补 SetAngleWithMotorRps（_angle 是度，内部做 inverseDirection 取反 + _angle/360*reduction 度→圈换算）
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

    // Int to Bytes
    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    uint8_t mode = 0x13;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAcceleration(float _val)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAcceleration_persist(float _val, bool _persist)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = _persist ? 1 : 0; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentLimit_persist(float _val, bool _persist)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = _persist ? 1 : 0; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
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

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

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
    canBuf[4] = 1; // Need save to EEPROM or not

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

void CtrlStepMotor::QueryAcceleration()
{
    uint8_t mode = 0x2C;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}

bool CtrlStepMotor::TryQueryAcceleration(uint32_t requestId)
{
    // The motor's existing 0x2C reply preserves request bytes 4..7.
    CAN_TxHeaderTypeDef header = {
        .StdId = static_cast<uint32_t>((nodeID << 7) | 0x2C),
        .ExtId = 0, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA,
        .DLC = 8, .TransmitGlobalTime = DISABLE
    };
    uint8_t data[8] = {};
    memcpy(data + 4, &requestId, sizeof(requestId));
    return TryCanSendMessage(get_can_ctx(hcan), data, &header);
}

void CtrlStepMotor::QueryCurrentLimit()
{
    uint8_t mode = 0x2D;
    txHeader.StdId = nodeID << 7 | mode;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
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
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);  // SetPositionSetPoint 已根据 inverseDirection 取反
}


void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);  // SetPositionWithVelocityLimit 已根据 inverseDirection 取反
}


void CtrlStepMotor::UpdateAngle()
{
    CAN_TxHeaderTypeDef header = txHeader;
    header.StdId = (nodeID << 7) | MotionCompletionProtocol::COMMAND;
    uint8_t data[8] = {}; // Do not accidentally reuse the extended-query marker.
    CanSendMessage(get_can_ctx(hcan), data, &header);
}

bool CtrlStepMotor::QueryPositionCompletion(float expectedMotorR, uint32_t nonce, uint32_t epoch, uint32_t generation)
{
    CAN_TxHeaderTypeDef header = {};
    header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8;
    header.StdId = (nodeID << 7) | MotionCompletionProtocol::COMMAND;
    uint8_t data[8] = {};
    memcpy(data, &expectedMotorR, 4); data[4] = MotionCompletionProtocol::QUERY_MARKER;
    MotionCompletionProtocol::PutNonce(data, nonce);
    return CanSendMotionMessage(get_can_ctx(hcan), data, &header, epoch, generation);
}



void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}


void CtrlStepMotor::SetDceKp(int32_t _val)
{
    uint8_t mode = 0x17;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKv(int32_t _val)
{
    uint8_t mode = 0x18;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKi(int32_t _val)
{
    uint8_t mode = 0x19;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKd(int32_t _val)
{
    uint8_t mode = 0x1A;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

bool CtrlStepMotor::SendMotionTarget(float signedMotorR, float velocity, uint32_t epoch, uint32_t generation)
{
    CAN_TxHeaderTypeDef header = {};
    header.IDE = CAN_ID_STD; header.RTR = CAN_RTR_DATA; header.DLC = 8;
    header.StdId = (nodeID << 7) | 0x07;
    uint8_t data[8]; memcpy(data, &signedMotorR, 4); memcpy(data + 4, &velocity, 4);
    return CanSendMotionMessage(get_can_ctx(hcan), data, &header, epoch, generation);
}
