// Cache changed content once; animate with the exact same sampler as the footer.
(() => {
  if (!window.ControlDeskGrid) return;
  const selector = '.session-intro h2, .panel-heading h2, .metrics article, .diagnostic-item, .chart-wrap, .logs, .guide-grid article, .slide-caption, .joint-position, .status-details > div, .led-live > div, .drawer-heading h2, .settings-drawer legend';
  const controls = 'button, input, select, textarea, a, label, summary, [contenteditable], [role="tab"], [role="switch"]';
  const properties = ('display position box-sizing width height min-width min-height max-width max-height top left right bottom margin padding border border-top border-right border-bottom border-left border-collapse border-spacing border-radius background background-color background-image background-size background-position color font font-family font-size font-weight font-style line-height letter-spacing text-align text-decoration text-transform text-overflow white-space word-break overflow overflow-x overflow-y vertical-align opacity transform transform-origin flex flex-direction flex-wrap flex-grow flex-shrink flex-basis align-items align-content align-self justify-content gap grid-template-columns grid-template-rows grid-column grid-row row-gap column-gap fill stroke stroke-width stroke-dasharray stop-color stop-opacity box-shadow object-fit').split(' ');
  const toggle = document.querySelector('#signalMotionEnabled');
  const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
  const finePointer = matchMedia('(any-hover: hover) and (any-pointer: fine)');
  const preferenceKey = 'robstride.signal-motion.v1';
  const cache = new Map(), active = new Map();
  let enabled = true, frame = 0, lastFrame = 0, hovered = null, pending = null, warmTimer = 0, stylesheet = null;
  try { enabled = localStorage.getItem(preferenceKey) !== 'false'; } catch {}
  const canAnimate = () => enabled && !reducedMotion.matches && finePointer.matches && !document.hidden;
  const visible = surface => {
    const r = surface.getBoundingClientRect();
    return surface.isConnected && !surface.closest('[inert]') && r.width > 0 && r.height > 0 && r.bottom > (surface.closest('.settings-drawer')?0:64) && r.top < innerHeight && r.right > 0 && r.left < innerWidth && !surface.closest('.monitor-slide:not(.is-active)');
  };
  const settled = surface => {
    for (let node=surface; node && node!==document.body; node=node.parentElement) {
      if (node.getAnimations({subtree:node===surface}).some(animation => animation.playState === 'running')) return false;
    }
    return true;
  };

  function remove(surface) {
    const state = active.get(surface);
    if (!state) return;
    surface.classList.remove('signal-distorting');
    state.canvas.remove();
    state.grid.clear();
    if (pending === state.grid) pending = null;
    active.delete(surface);
  }
  function reset() {
    cancelAnimationFrame(frame);
    frame = lastFrame = 0;
    hovered = pending = null;
    for (const surface of active.keys()) remove(surface);
  }
  function clearCache() { reset(); cache.clear(); stylesheet=null; }
  function snapshotStyles() {
    if (stylesheet!==null) return stylesheet;
    // Match media against the browser viewport, not the smaller snapshot image.
    // All descendant rules, inheritance and pseudo elements then remain native CSS.
    const flatten = rules => [...rules].map(rule => rule instanceof CSSMediaRule
      ? (matchMedia(rule.conditionText).matches ? flatten(rule.cssRules) : '')
      : rule.cssText).join('\n');
    stylesheet=[...document.styleSheets].map(sheet=>flatten(sheet.cssRules)).join('\n')+
      '\n*{animation:none!important;transition:none!important;scrollbar-width:none!important}::-webkit-scrollbar{display:none!important}';
    return stylesheet;
  }
  async function capture(surface) {
    if (cache.has(surface)) return cache.get(surface).ready;
    const rect = surface.getBoundingClientRect();
    if (!visible(surface) || !settled(surface)) return null;
    while (cache.size >= 16) {
      const victim = [...cache.keys()].find(node => !active.has(node) && node !== hovered);
      if (!victim) return null;
      cache.delete(victim);
    }
    const entry = {source:null, width:rect.width, height:rect.height, ready:null};
    cache.set(surface, entry);
    entry.ready = new Promise(resolve => {
      const prepare = async () => {
        try {
          if (cache.get(surface)!==entry || !surface.isConnected) { resolve(null); return; }
          const clone = surface.cloneNode(true);
          const style=getComputedStyle(surface);
          for (const key of [...properties, ...Array.from(style).filter(key=>key.startsWith('--'))]) clone.style.setProperty(key,style.getPropertyValue(key));
          // Preserve unitless inheritance for descendants with smaller font sizes.
          const lineHeight=parseFloat(style.lineHeight)/parseFloat(style.fontSize);
          if (Number.isFinite(lineHeight)) clone.style.lineHeight=String(lineHeight);
          const originals = [surface, ...surface.querySelectorAll('*')];
          const copies = [clone, ...clone.querySelectorAll('*')];
          originals.forEach((node, i) => {
            copies[i].classList.remove('signal-distorting');
            // Canvas pixels are not copied by cloneNode. Preserve real curve pixels.
            if (node instanceof HTMLCanvasElement) {
              const image=document.createElement('img');
              image.src=node.toDataURL();
              image.style.cssText=copies[i].style.cssText;
              image.className=node.className;image.id=node.id;
              image.width=node.width;image.height=node.height;
              copies[i].replaceWith(image);
            }
            if (node instanceof HTMLElement) {
              // Preserve viewport content for horizontally/vertically scrolled lists.
              if ((node.scrollTop || node.scrollLeft) && copies[i].firstElementChild) {
                const wrapper = document.createElement('div');
                wrapper.style.transform = `translate(${-node.scrollLeft}px, ${-node.scrollTop}px)`;
                wrapper.append(...copies[i].childNodes);
                copies[i].append(wrapper);
              }
            }
          });
          Object.assign(clone.style, {position:'relative', top:'auto', left:'auto', right:'auto', bottom:'auto', margin:'0', transform:'none', opacity:active.get(surface)?.opacity || style.opacity, width:`${rect.width}px`, height:`${rect.height}px`, overflow:'hidden'});
          const NS='http://www.w3.org/2000/svg';
          const svg=document.createElementNS(NS,'svg');
          svg.setAttribute('width',rect.width);svg.setAttribute('height',rect.height);
          const styles=document.createElementNS(NS,'style');styles.textContent=snapshotStyles();svg.append(styles);
          const object=document.createElementNS(NS,'foreignObject');object.setAttribute('width','100%');object.setAttribute('height','100%');svg.append(object);
          // Retain ancestor selectors (metrics, settings drawer, active slide) in snapshots.
          let wrapped=clone;
          for(let node=surface.parentElement;node;node=node.parentElement){
            const shell=document.createElement('div');shell.className=node.className;shell.id=node.id;
            Object.assign(shell.style,{display:'block',position:'static',width:'100%',height:'100%',margin:'0',padding:'0',border:'0',transform:'none',opacity:'1',background:'transparent'});
            shell.append(wrapped);wrapped=shell;
            if(node.matches('.signal-zone'))break;
          }
          object.append(wrapped);
          const image = new Image();
          image.src = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(new XMLSerializer().serializeToString(svg));
          await image.decode();
          if (cache.get(surface) !== entry || !surface.isConnected) { resolve(null); return; }
          const dpr = Math.min(devicePixelRatio || 1, 2);
          const source = document.createElement('canvas');
          source.width = Math.max(1, Math.round(rect.width*dpr));
          source.height = Math.max(1, Math.round(rect.height*dpr));
          const context = source.getContext('2d');
          if (!context) throw new Error('Canvas unavailable');
          context.drawImage(image, 0, 0, source.width, source.height);
          entry.source = source;
          resolve(entry);
        } catch {
          // Retain a failed entry until content/layout changes, avoiding idle retries.
          entry.failed = true;
          resolve(null); // Keep the native interface if snapshot rendering is unavailable.
        }
      };
      // Never clone or resolve styles inside the pointer event handler.
      if (window.requestIdleCallback) requestIdleCallback(prepare,{timeout:100}); else setTimeout(prepare,0);
    });
    return entry.ready;
  }
  function surfacePath(surface) {
    const parts=[];
    for(let node=surface;node;node=node.parentElement) {
      if(node.id){parts.unshift('#'+CSS.escape(node.id));break}
      parts.unshift(node.localName+':nth-child('+([...node.parentElement.children].indexOf(node)+1)+')');
    }
    return parts.join(' > ');
  }
  function updateSnapshot(surface,state,entry) {
    if(active.get(surface)!==state||!entry?.source)return;
    state.entry=entry;state.dirtyAt=0;
    if(state.canvas.width!==entry.source.width)state.canvas.width=entry.source.width;
    if(state.canvas.height!==entry.source.height)state.canvas.height=entry.source.height;
  }
  function refresh(surface,state) {
    capture(surface).then(entry=>{
      if(entry)updateSnapshot(surface,state,entry);
      else if(cache.get(surface)?.failed)remove(surface);
    });
  }
  function create(surface, entry, grid) {
    // Small cards may decay together; large tables share a bounded pixel budget.
    let pixels=entry.source.width*entry.source.height;
    for(const state of active.values())pixels+=state.canvas.width*state.canvas.height;
    while(active.size&&(active.size>=6||pixels>8_000_000)){
      const [oldSurface,oldState]=active.entries().next().value;
      pixels-=oldState.canvas.width*oldState.canvas.height;remove(oldSurface);
    }
    const canvas = document.createElement('canvas');
    canvas.className = 'signal-overlay';
    canvas.setAttribute('aria-hidden', 'true');
    canvas.width = entry.source.width;
    canvas.height = entry.source.height;
    if (surface.closest('.settings-drawer')) canvas.classList.add('signal-overlay-drawer');
    const context = canvas.getContext('2d');
    if (!context) return null;
    const state = {canvas, context, entry, grid, opacity:getComputedStyle(surface).opacity, path:surfacePath(surface), dirtyAt:0};
    active.set(surface, state);
    return state;
  }
  function animate(now) {
    frame = 0;
    if (!canAnimate()) { reset(); return; }
    const elapsed = lastFrame ? Math.min(50, now-lastFrame) : 1000/60;
    lastFrame = now;
    for (const [surface, state] of active) {
      const rect = surface.getBoundingClientRect();
      if (!visible(surface) || Math.abs(rect.width-state.entry.width)>.5 || Math.abs(rect.height-state.entry.height)>.5 || !state.grid.advance(elapsed)) { remove(surface); continue; }
      // Keep motion continuous while replacing content. Never hold an old image
      // beyond 80ms if a new snapshot is delayed or a CSS transition is running.
      if(state.dirtyAt&&performance.now()-state.dirtyAt>80){surface.classList.remove('signal-distorting');state.canvas.remove();continue}
      const {canvas, context, entry} = state;
      Object.assign(canvas.style, {left:`${rect.left}px`, top:`${rect.top}px`, width:`${rect.width}px`, height:`${rect.height}px`});
      state.grid.draw(context, entry.source, canvas.width, canvas.height);
      if (!canvas.isConnected) { document.body.append(canvas); surface.classList.add('signal-distorting'); }
    }
    if (active.size) frame = requestAnimationFrame(animate); else lastFrame = 0;
  }
  function changeHover(surface) {
    if (hovered === surface) return;
    active.get(hovered)?.grid.leave();
    hovered = surface;
    pending = null;
  }
  document.addEventListener('pointermove', event => {
    if (event.pointerType === 'touch' || event.buttons || !canAnimate()) { changeHover(null); return; }
    if (event.target.closest(controls)) {
      for (const surface of active.keys()) if (surface.contains(event.target)) remove(surface);
      changeHover(null); return;
    }
    const surface = event.target.closest(selector);
    if (!surface?.closest('.signal-zone') || !visible(surface)) { changeHover(null); return; }
    changeHover(surface);
    const rect = surface.getBoundingClientRect();
    const x = (event.clientX-rect.left)/rect.width, y = (event.clientY-rect.top)/rect.height;
    let entry = cache.get(surface);
    if (entry && (Math.abs(rect.width-entry.width)>.5 || Math.abs(rect.height-entry.height)>.5)) {
      remove(surface); cache.delete(surface); entry=null;
    }
    const grid = active.get(surface)?.grid || (pending ||= new ControlDeskGrid());
    const moved = grid.move(x,y);
    if (!entry?.source) {
      capture(surface).then(ready => {
        if(!ready||!canAnimate()||hovered!==surface)return;
        if(active.has(surface)){updateSnapshot(surface,active.get(surface),ready);return}
        if(pending!==grid)return;
        if(!grid.field.some(value=>Math.abs(value)>.005))return;
        create(surface, ready, grid);
        if (!frame) { lastFrame=0; frame=requestAnimationFrame(animate); }
      });
      return;
    }
    // A previous surface may still be animating. Keep the entry coordinate out
    // of that loop until this grid receives movement, or it gets cleared before
    // the next pointer event and the next card can never start.
    if(!moved&&!active.has(surface))return;
    const state = active.get(surface) || create(surface, entry, grid);
    if (state && moved && !frame) { lastFrame=0; frame=requestAnimationFrame(animate); }
  }, {passive:true});

  // Prewarm one visible surface per idle task, never serialize in the animation loop.
  function scheduleWarm() {
    if (warmTimer || !canAnimate()) return;
    warmTimer = setTimeout(() => {
      warmTimer = 0;
      if (!canAnimate()) return;
      for (const surface of cache.keys()) if (!surface.isConnected) cache.delete(surface);
      const next = [...document.querySelectorAll(selector)].find(surface => surface.closest('.signal-zone') && visible(surface) && settled(surface) && !cache.has(surface));
      if (!next || cache.size >= 16) return;
      capture(next).then(scheduleWarm);
    }, 32);
  }
  new MutationObserver(records => {
    let changed = false;
    const dirty=new Set();
    // The simulator replaces metric elements. Transfer the running field to the
    // same logical card rather than restarting its spring on every data update.
    for(const [surface,state] of active) {
      if(surface.isConnected)continue;
      const replacement=document.querySelector(state.path);
      if(!replacement){remove(surface);continue}
      surface.classList.remove('signal-distorting');active.delete(surface);cache.delete(surface);
      state.opacity=getComputedStyle(replacement).opacity;
      replacement.classList.add('signal-distorting');
      active.set(replacement,state);dirty.add(replacement);
      if(hovered===surface)hovered=replacement;
    }
    for (const record of records) {
      if (record.type === 'childList' && [...record.addedNodes, ...record.removedNodes].every(node => node.nodeType === Node.ELEMENT_NODE && node.classList.contains('signal-overlay'))) continue;
      // The controller republishes unchanged readouts on every poll.
      if(record.type==='childList' && [...record.addedNodes,...record.removedNodes].every(node=>node.nodeType===Node.TEXT_NODE) && [...record.addedNodes].map(node=>node.textContent).join('')===[...record.removedNodes].map(node=>node.textContent).join(''))continue;
      const node = record.target.nodeType === Node.ELEMENT_NODE ? record.target : record.target.parentElement;
      if (!node) continue;
      if (record.attributeName === 'class' && (record.oldValue || '').replace(/\bsignal-distorting\b/g,'').trim() === node.className?.toString().replace(/\bsignal-distorting\b/g,'').trim()) continue;
      if (!node.closest('.signal-zone') && !node.querySelector('.signal-zone')) continue;
      changed = true;
      for (const surface of new Set([...cache.keys(),...active.keys()])) if (!surface.isConnected || surface === node || surface.contains(node) || node.contains(surface)) dirty.add(surface);
    }
    for(const surface of dirty){
      cache.delete(surface);
      const state=active.get(surface);
      if(state){state.dirtyAt ||= performance.now();refresh(surface,state)}
    }
    if (changed) scheduleWarm();
  }).observe(document.body, {subtree:true, childList:true, characterData:true, attributes:true, attributeFilter:['class','style','hidden','width','height'], attributeOldValue:true});
  function syncPreference() { if (toggle) toggle.checked=enabled; clearCache(); scheduleWarm(); }
  toggle?.addEventListener('change', () => {
    enabled=toggle.checked;
    try { localStorage.setItem(preferenceKey,String(enabled)); } catch {}
    syncPreference();
  });
  document.querySelector('#open-settings')?.addEventListener('click', scheduleWarm);
  document.addEventListener('upper-computer:config', syncPreference);
  document.addEventListener('transitionend', () => {
    for(const [surface,state] of active)if(state.dirtyAt)refresh(surface,state);
    scheduleWarm();
  });
  document.fonts?.ready.then(() => { clearCache(); scheduleWarm(); });
  document.addEventListener('visibilitychange', () => { reset(); scheduleWarm(); });
  document.addEventListener('pointercancel', reset);
  document.addEventListener('pointerdown', reset, {passive:true});
  document.addEventListener('focusin', reset);
  document.addEventListener('pointerout', event => { if (!event.relatedTarget) changeHover(null); });
  addEventListener('scroll', () => { clearCache(); scheduleWarm(); }, {passive:true, capture:true});
  addEventListener('resize', () => { clearCache(); scheduleWarm(); });
  addEventListener('blur', reset);
  reducedMotion.addEventListener('change', syncPreference);
  finePointer.addEventListener('change', syncPreference);
  syncPreference();
})();
