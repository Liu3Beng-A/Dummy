// Demand-driven drawing. Incoming data may change faster than the display.
export class RenderSchedule {
  constructor(draw, {
    frame = callback => requestAnimationFrame(callback),
    cancelFrame = id => cancelAnimationFrame(id),
    timer = (callback, delay) => setTimeout(callback, delay),
    cancelTimer = id => clearTimeout(id),
    now = () => performance.now(), backgroundHz = 15,
  } = {}) {
    Object.assign(this, { draw, frame, cancelFrame, timer, cancelTimer, now, backgroundHz });
    this.mode = 'active';
    this.frameId = this.timerId = this.lastDraw = null;
    this.dirty = this.drawing = this.closed = false;
    this.frames = 0;
  }

  request() {
    if (this.closed) return;
    this.dirty = true;
    this.queue();
  }

  setMode(mode) {
    if (!['active', 'background', 'hidden'].includes(mode) || this.closed) return;
    if (mode === this.mode) return;
    this.cancel();
    this.mode = mode;
    this.lastDraw = null;
    this.request();
  }

  cancel() {
    if (this.frameId !== null) this.cancelFrame(this.frameId);
    if (this.timerId !== null) this.cancelTimer(this.timerId);
    this.frameId = this.timerId = null;
  }

  queue() {
    if (this.closed || this.mode === 'hidden' || this.drawing || !this.dirty
        || this.frameId !== null || this.timerId !== null) return;
    const delay = this.mode === 'background' && this.lastDraw !== null
      ? Math.max(0, 1000 / this.backgroundHz - (this.now() - this.lastDraw)) : 0;
    if (delay > 0) {
      this.timerId = this.timer(() => {
        this.timerId = null;
        this.queue();
      }, Math.ceil(delay));
    } else {
      this.frameId = this.frame(time => this.tick(time));
    }
  }

  tick(time) {
    this.frameId = null;
    if (this.closed || this.mode === 'hidden' || !this.dirty) return;
    this.dirty = false;
    this.drawing = true;
    try {
      if (this.draw(time)) this.dirty = true;
      this.frames++;
      this.lastDraw = this.now();
    } finally {
      this.drawing = false;
    }
    this.queue();
  }

  diagnostics() {
    return { mode: this.mode, frames: this.frames,
      idle: !this.drawing && !this.dirty && this.frameId === null && this.timerId === null };
  }

  dispose() {
    this.cancel();
    this.closed = true;
  }
}
