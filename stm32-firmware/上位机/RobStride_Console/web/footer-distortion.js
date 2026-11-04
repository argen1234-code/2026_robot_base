// Pointer velocity -> local displacement grid -> resample the original wordmark.
// Independent Canvas 2D implementation; no external textures or rendering libraries.
(() => {
  const surface = document.querySelector('#footerWordmark');
  const canvas = document.querySelector('#footerDistortion');
  const fallback = document.querySelector('#footerWordmarkText');
  const toggle = document.querySelector('#footerMotionEnabled');
  const context = canvas?.getContext('2d');
  const source = document.createElement('canvas');
  const sourceContext = source.getContext('2d');
  if (!surface || !context || !sourceContext || !fallback) return;

  const grid = new ControlDeskGrid();
  const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
  const finePointer = matchMedia('(any-hover: hover) and (any-pointer: fine)');
  const preferenceKey = 'robstride.footer-motion.v1';
  let enabled = true, visible = false, frame = 0, lastFrame = 0;
  let width = 0, height = 0;
  try { enabled = localStorage.getItem(preferenceKey) !== 'false'; } catch {}

  const canAnimate = () => enabled && !reducedMotion.matches && finePointer.matches && !document.hidden && visible;

  function paint() {
    grid.draw(context, source, width, height);
  }

  function reset() {
    cancelAnimationFrame(frame);
    frame = 0;
    lastFrame = 0;
    grid.clear();
    if (width && height) paint();
  }

  function resize() {
    const rect = surface.getBoundingClientRect();
    const dpr = Math.min(devicePixelRatio || 1, 2);
    width = Math.max(1, Math.round(rect.width * dpr));
    height = Math.max(1, Math.round(rect.height * dpr));
    canvas.width = source.width = width;
    canvas.height = source.height = height;
    // Render the template's own text once. Only this cached image is sampled per frame.
    const text = (fallback.textContent.trim() || 'ControlDesk').slice(0, 40);
    sourceContext.font = '900 200px Arial, "Microsoft YaHei", sans-serif';
    const metrics = sourceContext.measureText(text);
    const left = metrics.actualBoundingBoxLeft;
    const inkWidth = left + metrics.actualBoundingBoxRight;
    const inkHeight = metrics.actualBoundingBoxAscent + metrics.actualBoundingBoxDescent;
    const scaleX = width * .94 / Math.max(1, inkWidth);
    const scaleY = height * .84 / Math.max(1, inkHeight);
    sourceContext.setTransform(scaleX, 0, 0, scaleY, width * .03 + left * scaleX,
      height * .08 + metrics.actualBoundingBoxAscent * scaleY);
    sourceContext.fillStyle = '#090b0a';
    sourceContext.fillText(text, 0, 0);
    sourceContext.resetTransform();
    surface.classList.add('is-ready');
    reset();
  }

  function animate(now) {
    frame = 0;
    if (!canAnimate()) { reset(); return; }
    const elapsed = lastFrame ? Math.min(50, now - lastFrame) : 1000 / 60;
    lastFrame = now;
    const moving = grid.advance(elapsed);
    paint();
    if (moving) frame = requestAnimationFrame(animate);
    else reset();
  }

  surface.addEventListener('pointermove', event => {
    if (event.pointerType === 'touch' || !canAnimate()) return;
    const rect = surface.getBoundingClientRect();
    const x = (event.clientX - rect.left) / rect.width;
    const y = (event.clientY - rect.top) / rect.height;
    if (grid.move(x, y) && !frame) { lastFrame = 0; frame = requestAnimationFrame(animate); }
  }, { passive: true });
  surface.addEventListener('pointerleave', () => grid.leave());
  surface.addEventListener('pointercancel', reset);
  addEventListener('blur', reset);
  addEventListener('scroll', () => grid.leave(), { passive: true });

  function syncPreference() {
    if (toggle) toggle.checked = enabled;
    surface.classList.toggle('is-static', !enabled || reducedMotion.matches || !finePointer.matches);
    reset();
  }
  toggle?.addEventListener('change', () => {
    enabled = toggle.checked;
    try { localStorage.setItem(preferenceKey, String(enabled)); } catch {}
    syncPreference();
  });
  document.querySelector('#open-settings')?.addEventListener('click', syncPreference);
  reducedMotion.addEventListener('change', syncPreference);
  finePointer.addEventListener('change', syncPreference);
  document.addEventListener('visibilitychange', reset);
  document.addEventListener('upper-computer:config', resize);
  new ResizeObserver(resize).observe(surface);
  new IntersectionObserver(entries => {
    visible = entries[0].isIntersecting;
    if (!visible) reset();
  }).observe(surface);
  resize();
  syncPreference();
})();
