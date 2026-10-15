#pragma once
#include <cstdint>
#include "stream_runtime.h"
#include "motion_completion_protocol.h"
#include <vector>
#define MOTOR_TYPE 35
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
struct CAN_TxHeaderTypeDef {uint32_t StdId{},IDE{},RTR{},DLC{};};
struct ReplyFrame {uint32_t id;uint8_t bytes[8];};
extern std::vector<ReplyFrame> replies;
bool CAN_TrySend(CAN_TxHeaderTypeDef*,const uint8_t*);
bool CAN_QueueStreamReply(CAN_TxHeaderTypeDef*,const uint8_t*);
void CAN_CancelStreamReplies();
bool CAN_StreamReplyFault();
uint32_t CAN_StreamReplyDiagnostic(uint8_t);
inline volatile uint32_t motorControlTiming[11]{};
extern uint32_t irqMask;
inline uint32_t __get_PRIMASK(){return irqMask;}
inline void __disable_irq(){irqMask=1;}
inline void (*irqReleaseHook)()=nullptr;
inline void __set_PRIMASK(uint32_t m){irqMask=m;if(!m && irqReleaseHook){auto hook=irqReleaseHook;irqReleaseHook=nullptr;hook();}}
inline uint32_t __get_MSP(){return 0x20004e00;}
struct DwtStub {uint32_t CYCCNT=0;};
inline DwtStub dwt;
#define DWT (&dwt)
inline bool calibrationBusy=false;
inline bool MotorStreamCalibrationBusy(){return calibrationBusy;}
namespace ProtectionProtocol {constexpr uint8_t IDLE=0,DR_COMPLETED=8,DG_NO_MOTION=8;}
namespace SoftStopProtocol {constexpr uint8_t STOP=1;}
struct BoardConfig {uint32_t canNodeId=1;int32_t velocityLimit=1536000,velocityAcc=1536000;};
extern BoardConfig boardConfig;
class Motor {
public:
 static constexpr int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS=51200;
 enum Mode { MODE_COMMAND_POSITION=1, MODE_STREAM_JOINT=9 };
 using Mode_t=Mode;
 enum State { STATE_RUNNING=2 };
 struct Config {struct Params{int32_t encoderHomeOffset{};}motionParams;}config;
 class Controller {
 public:
  Motor* context{};
  bool goalDisable{},softDisable{},goalBrake{},softBrake{},isStalled{},calibrationTask{},motionIntent{},positionSettled{},positionCompleted{},overloadFlag{};
  bool locked{},feedback=true;
  unsigned stopCalls=0;
  uint8_t protectionPhase{},protectionReason{},state{},detectionReset{},detectionResetDetail{},detectionGate{};
  uint32_t phaseTicks{};bool eventAcknowledged{};
  Mode modeRunning=MODE_COMMAND_POSITION,requestMode=MODE_COMMAND_POSITION;
  int32_t realPosition{},estVelocity{},softPosition{},goalPosition{},softVelocity{};
  int32_t realLapPosition{},realLapPositionLast{},realPositionLast{},estVelocityIntegral{},estPosition{},estError{};
  uint32_t transportMissingTicks{},invalidFeedbackTicks{};
  uint16_t positionStableTicks{};
  int32_t positionStableMin{},positionStableMax{},positionCompletionTarget{},positionCompletionHome{};
  int64_t positionVelocitySum{},positionEvidenceVelocitySum{};
  uint8_t positionCompletionMode{};bool positionEvidenceValid{};
  uint32_t positionEvidenceTick{},positionEvidenceRange{};
  uint32_t CaptureStreamOpen(uint16_t,bool=false);
  uint32_t StreamClock()const;uint32_t PositionEntryEvidenceAge()const;
  void InvalidatePositionEntry();void SetCtrlMode(Mode_t);void SetPositionSetPoint(int32_t);
  void SetDisable(bool);void SetBrake(bool);void ApplyPosAsHomeOffset();
  void SetProtectionPhase(uint8_t,uint8_t=0);
  bool StreamActive() const;
  bool StreamTerminal() const;
  void CancelStream(uint8_t why=StreamProtocol::ABORTED);
  bool ProcessStreamCan(uint8_t,const uint8_t*,uint32_t);
  void TickStreamClock();void TickStream();void PollStreamMain();
  bool IsFeedbackValid()const{return feedback&&invalidFeedbackTicks==0;}
  bool IsSoftStopLocked()const{return locked;}
  uint16_t GetSoftStopToken()const{return 0;}
  void OnMasterCommunication(){}
  void ResetDetection(bool=false){}
  void ResetPositionCompletion(bool=false);
  bool TickPositionCompletion();bool IsOrdinaryPositionArrival(bool=true)const;
  void RequestSoftStop(uint8_t,uint16_t){++stopCalls;InvalidatePositionEntry();CancelStream();locked=true;}
 };
};

#include "motor_position_completion_under_test.inc"
