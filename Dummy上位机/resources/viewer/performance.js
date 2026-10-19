// Count completed visible renders; input telemetry Hz is a separate measure.
export class FramePerformance {
  constructor(windowMs = 2000) { this.windowMs = windowMs; this.frames = []; }
  reset() { this.frames = []; }
  record(now) {
    if (!Number.isFinite(now)) return;
    if (this.frames.length && now <= this.frames.at(-1)) return;
    this.frames.push(now);
    this.frames = this.frames.filter(value => now - value <= this.windowMs).slice(-512);
  }
  snapshot(now) {
    const frames = this.frames.filter(value => value <= now && now - value <= this.windowMs);
    const gaps = frames.slice(1).map((value, index) => value - frames[index]).sort((a,b) => a-b);
    return { fps: frames.length > 1 ? (frames.length-1)*1000/(now-frames[0]) : null,
      frame_ms_p95: gaps.length ? gaps[Math.min(gaps.length-1, Math.floor(gaps.length*.95))] : null,
      rendered_frames: frames.length };
  }
}
