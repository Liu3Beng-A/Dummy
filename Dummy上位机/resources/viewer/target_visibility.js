// Shared by rendering and camera framing. No pose, command or preference is
// changed when matching feedback temporarily hides the joint-target model.
export function jointTargetAllowed(state) {
  return state?.mode === 'preview' && state.actualOnly !== true
    && state.targetEnabled !== false
    && !state.marker && !(state.rgbPreview === true && !state.actualKnown);
}

export function jointTargetVisible(state) {
  return jointTargetAllowed(state) && state.targetMatched !== true;
}

export const TARGET_FADE_MS = 500;

export class TargetVisibilityFade {
  constructor() {
    this.value = 0;
    this.from = 0;
    this.to = 0;
    this.startedAt = 0;
  }

  sample(nowMs) {
    const progress = Math.min(1, Math.max(0, (nowMs - this.startedAt) / TARGET_FADE_MS));
    const eased = progress * progress * (3 - 2 * progress);
    this.value = this.from + (this.to - this.from) * eased;
    return this.value;
  }

  setState(state, nowMs) {
    const current = this.sample(nowMs);
    // Display-mode and master gates still take effect immediately. Only the
    // automatic match/difference decision fades while preview is permitted.
    if (!jointTargetAllowed(state)) {
      this.value = this.from = this.to = 0;
      this.startedAt = nowMs;
      return this.value;
    }
    const next = jointTargetVisible(state) ? 1 : 0;
    if (next !== this.to) {
      this.from = current;
      this.to = next;
      this.startedAt = nowMs;
    }
    return this.value;
  }
}
