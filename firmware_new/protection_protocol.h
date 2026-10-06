#ifndef DUMMY_PROTECTION_PROTOCOL_H
#define DUMMY_PROTECTION_PROTOCOL_H

#include <stdint.h>

// Every frame is exactly eight little-endian bytes. Existing 0x0A values are
// unchanged. Node zero retains its existing ALL meaning; protection ALL is
// expanded to nodes 1..7 by the main controller.
namespace ProtectionProtocol {
constexpr uint8_t VERSION = 1;
constexpr uint8_t EVENT = 0x0B, EVENT_ACK = 0x0C;
constexpr uint8_t PARAM_SET = 0x30, PARAM_GET = 0x31, PARAM_REPLY = 0x32;
constexpr uint8_t CONFIG = 0x33, CONTEXT = 0x34, CONTEXT_REPLY = 0x35;
// Coherent QUERY snapshot: coordinateVersion[0], flags[1], probe u16[2..3],
// real motor-r float32[4..7]. Storage VERSION stays 1: no EEPROM migration.
constexpr uint8_t PROBE_REPLY = 0x36;
// Extended 0x0A action: ramps the seven axes without cancelling their current
// protection episode. Plain STOP(1) always cancels internal retreat/calibration.
constexpr uint8_t STOP_PROTECTION = 5;
constexpr uint8_t QUERY_REPLY_FLAG = 0x80;
// CONTEXT_REPLY lease ACK: LEASE_REPLY_FLAG[0], current coordinateVersion[1],
// task u16[2..3], echoed renewal nonce u32[4..7]. Never a motion completion.
constexpr uint8_t LEASE_REPLY_FLAG = 0x40;

// EVENT: eventId u32[0..3], phase[4], reason[5], coordinateVersion[6], flags[7].
// EVENT_ACK: eventId u32[0..3], acknowledged phase[4], zero[5..7]. QUERY event
// uses CONTEXT/QUERY. An ACK never releases a protection lock.
enum Phase : uint8_t {
    IDLE = 0, LOCAL_STOPPING = 1, WAIT_GRANT = 2, RETREATING = 3,
    HOLD_SUCCESS = 4, HOLD_FAILED = 5, PEER_STOPPING = 6, PEER_HOLD = 7,
    CAL_READY = 8, CAL_SEEK = 9, CAL_BACKOFF = 10, CAL_RETOUCH = 11,
    CAL_RETURN = 12, CAL_DONE = 13, CAL_FAILED = 14,
    SAVE_PENDING = 15, SAVED = 16
};
enum Reason : uint8_t {
    NONE = 0, STALL = 1, NO_DIRECTION = 2, NO_SPACE = 3,
    NO_ENCODER = 4, INVALID_PROFILE = 5, GRANT_TIMEOUT = 6,
    LEASE_TIMEOUT = 7, RETREAT_TIMEOUT = 8, REBLOCKED = 9,
    CANCELLED_STOP = 10, CANCELLED_DISABLE = 11, SEARCH_TIMEOUT = 12,
    SEARCH_TRAVEL = 13, CONTACT_UNSTABLE = 14, EARLY_CONTACT = 15,
    RETURN_TIMEOUT = 16, SAVE_FAILED = 17, NOT_DISABLED = 18,
    BAD_TASK = 19, BAD_VALUE = 20, BUSY = 21, UNSUPPORTED = 22,
    COORDINATE_INVALID = 23, COMMUNICATION_TIMEOUT = 24, CAPTURE_UNSTABLE = 25
};
enum Flag : uint8_t {
    FEEDBACK_VALID = 1, PROFILE_VALID = 2, ASSUMED_BOOT = 4,
    COORD_DIRTY = 8, DISTANCE_CLIPPED = 16, LOCAL_STALL = 32,
    EXCLUDED = 64, DISABLED_FLAG = 128
};

// PARAM_SET/GET/REPLY: parameter[0], status[1], transaction u16[2..3],
// value float32[4..7]. All parameters use float32, including integral ms and
// booleans; integral-valued parameters reject fractional values.
enum Parameter : uint8_t {
    ENABLE = 1, CURRENT_A = 2, SPEED_RPS = 3, ERROR_R = 4,
    TRIGGER_MS = 5, STARTUP_MS = 6, CURRENT_FILTER_MS = 7,
    SPEED_FILTER_MS = 8, BACK_DISTANCE_R = 9, BACK_RPS = 10,
    BACK_ACCEL = 11, BACK_CURRENT_A = 12, HOLD_CURRENT_A = 13,
    BACK_POS_TOL_R = 14, BACK_TIMEOUT_MS = 15, REBLOCK_MS = 16,
    GRANT_WAIT_MS = 17, MIN_POSITION_R = 18, MAX_POSITION_R = 19,
    DIRECTION_MASK = 20, // bit 0 negative motor direction; bit 1 positive
    CAL_SEEK_RPS = 21, CAL_RETURN_RPS = 22, CAL_RETOUCH_RPS = 23,
    CAL_ACCEL = 24, CAL_CURRENT_A = 25, CAL_CONTACT_RATIO = 26,
    CAL_SPEED_RPS = 27, CAL_ERROR_R = 28, CAL_TRIGGER_MS = 29,
    CAL_STARTUP_MS = 30, CAL_MAX_TRAVEL_R = 31,
    CAL_SEEK_TIMEOUT_MS = 32, CAL_RETURN_TIMEOUT_MS = 33,
    CAL_RETOUCH_TIMEOUT_MS = 34, CAL_BACKOFF_R = 35,
    CAL_REPEAT_TOL_R = 36, CAL_POSITION_TOL_R = 37,
    CAL_CONTACT_WINDOW_R = 38, PARAMETER_COUNT = 38
};
enum Status : uint8_t { OK = 0, INVALID = 1, LOCKED = 2,
    TRANSACTION_MISMATCH = 3, WRITE_FAILED = 4, NOT_SUPPORTED = 5 };

// CONFIG: operation[0], zero[1], transaction u16[2..3], argument u32[4..7].
// BEGIN copies the live config into a staging group; SET only touches that
// group. COMMIT validates the entire group and atomically publishes it. A
// committed nonzero argument grants MIN/MAX/DIRECTION for the CURRENT
// coordinateVersion (argument low byte); zero commits detection settings only.
// Reply is PARAM_REPLY parameter=0, status[1], same transaction[2..3],
// echoed ConfigOperation[4], coordinateVersion[5], zero[6..7]. SAVE is
// permitted only while disabled. The operation echo rejects late BEGIN ACKs.
enum ConfigOperation : uint8_t { BEGIN = 1, COMMIT = 2, SAVE = 3, ABORT = 4 };

// CONTEXT: operation[0], coordinateVersion[1], task u16[2..3], value[4..7].
// QUERY returns PROBE_REPLY, plus an unacknowledged active EVENT if needed.
// Both endpoints must support PROBE_REPLY; legacy tagged snapshots cannot
// authorize START or refresh the verified protection status. CAL_BEGIN
// locks a disabled node under this task. SET_CAPTURE_COORD labels the current
// real encoder position with the signed motor-r float value (six joints use
// zero). CAL_RUN (J7/J8 only) runs seek/backoff/retouch/origin/return internally;
// value is an optional current command cap A (>0) bounded by CAL_CURRENT_A.
// RETREAT_GO value is the exact EVENT u32, rather than float; it requires the
// same valid coordinate version and is idempotent for task/event. RENEW is
// bound to the active task/version; value is a nonzero renewal nonce, echoed
// only after that lease was refreshed. Its lease remains 500ms. CANCEL invalidates the
// task and stops internal motion (including J8 ONLY in its CAL context).
// SAVE_COORD queues a disabled, completed task for main-loop flash save.
// CAL_END clears only a completed calibration context while remaining disabled.
enum ContextOperation : uint8_t {
    QUERY = 0, CAL_BEGIN = 1, SET_CAPTURE_COORD = 2, CAL_RUN = 3,
    CANCEL = 4, SAVE_COORD = 5, RETREAT_GO = 6, RENEW = 7, CAL_END = 8,
    PROTECT_PREPARE = 9 // value event u32, establishes task before seven-axis stop
};
// CONTEXT_REPLY: phase[0], reason[1], active task u16[2..3], real signed
// motor position r float32[4..7]. Coordinate version/feedback flags are in
// EVENT. Replies are snapshots, never proof that an unacknowledged task ran.
constexpr uint32_t LEASE_MS = 500;
constexpr uint32_t COMMUNICATION_WATCHDOG_MS = 1000;
}

#endif
