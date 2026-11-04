// Shared by the footer and functional surfaces: identical input, damping and sampling.
(() => {
  class DistortionGrid {
    constructor() {
      this.size = 15;
      this.field = new Float32Array(this.size * this.size * 2);
      this.pointer = null;
    }
    clear() { this.field.fill(0); this.pointer = null; }
    leave() { this.pointer = null; }
    move(x, y) {
      if (!this.pointer) { this.pointer = {x, y}; return false; }
      const vx = Math.max(-.12, Math.min(.12, x - this.pointer.x));
      const vy = Math.max(-.12, Math.min(.12, y - this.pointer.y));
      this.pointer = {x, y};
      if (Math.abs(vx) + Math.abs(vy) < .0001) return false;
      for (let row = 0; row < this.size; row++) for (let col = 0; col < this.size; col++) {
        const distance = Math.hypot(x - (col + .5) / this.size, y - (row + .5) / this.size);
        if (distance >= .25) continue;
        const influence = Math.min(.25 / Math.max(distance, .025), 10);
        const i = (col + row * this.size) * 2;
        this.field[i] = Math.max(-8, Math.min(8, this.field[i] + 15 * vx * influence));
        this.field[i + 1] = Math.max(-8, Math.min(8, this.field[i + 1] + 15 * vy * influence));
      }
      return true;
    }
    advance(elapsed) {
      const damping = Math.pow(.9, Math.min(50, elapsed) / (1000 / 60));
      let energy = 0;
      for (let i = 0; i < this.field.length; i++) {
        this.field[i] *= damping;
        energy = Math.max(energy, Math.abs(this.field[i]));
      }
      return energy > .005;
    }
    draw(context, source, width, height) {
      context.clearRect(0, 0, width, height);
      const cw = width / this.size, ch = height / this.size;
      for (let row = 0; row < this.size; row++) for (let col = 0; col < this.size; col++) {
        const i = (col + row * this.size) * 2;
        const x = Math.round(col * cw), y = Math.round(row * ch);
        const w = Math.round((col + 1) * cw) - x, h = Math.round((row + 1) * ch) - y;
        context.drawImage(source, x - this.field[i] * .02 * width,
          y - this.field[i + 1] * .02 * height, w, h, x, y, w, h);
      }
    }
  }
  window.ControlDeskGrid = DistortionGrid;
})();
