#include "stream_runtime.h"

namespace MotorStream {
using namespace StreamProtocol;
bool Runtime::Retired(uint16_t token) const {
    for (const auto value:retired) if (token && value==token) return true;
    return false;
}
void Runtime::Clear(uint8_t why) {
    // Bounded late-packet window, not an authentication/security mechanism.
    if (session && !Retired(session)) { retired[retiredWrite]=session;retiredWrite=(retiredWrite+1)%8; }
    for (auto& slot:slots) slot.state=0;
    stagedMask=0; seeded=seedCommitted=armed=terminal=settled=syncSeen=false;
    prepared=committed=completed=0; reason=why;
    state=why ? STATE_FAULT : STATE_IDLE; changed=true; stopRequired=false;
}
uint8_t Runtime::Free() const {
    uint8_t n=0;
    for (const auto& slot:slots) n += slot.state==0;
    return n;
}
void Runtime::Fault(uint8_t why) {
    if (state==STATE_FAULT) return;
    Clear(why); stopRequired=true;
}
void Runtime::Tick() {
    if (!armed || state==STATE_FAULT || state==STATE_COMPLETED) return;
    if (static_cast<int32_t>(ticks-anchor) <= 0) return;
    if (ticks-lastSyncTick > 2000) { Fault(SYNC_TIMEOUT); return; }
    if (state==STATE_STARTED) { state=STATE_RUNNING; changed=true; }
    if (terminal) return;
    Slot& slot=slots[(completed+1)%CAPACITY];
    if (slot.state!=2 || slot.sequence!=completed+1) { Fault(UNDERRUN); return; }
    slot.curve.Tick(position,velocity);
    if (slot.curve.Finished()) {
        completed=slot.sequence;
        terminal=(slot.flags&END)!=0;
        slot.state=0; changed=true;
    }
}
}
