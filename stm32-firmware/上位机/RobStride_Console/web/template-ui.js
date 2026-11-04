/* Presentation and navigation only. Hardware actions remain in app.js / led.js.
   Layout, carousel and pixel field adapted from upper-computer-template. */
(() => {
  'use strict';
  const byId = id => document.getElementById(id);
  const modules = [
    ['control', '电机控制', '独立控制，协同运转。', '选择一个关节，编辑它的速度与运行限制。准备、使能、运行，每一步都有明确的反馈。'],
    ['telemetry', '数据观测', '把每一刻，看得更清楚。', '观测当前所选电机最近 30 秒的真实反馈。切换指标、暂停曲线，或导出采集记录。'],
    ['diagnostics', '通信诊断', '连接的每个细节，有据可查。', '从固件身份到 CAN 收发、参数回执与健康状态，逐层了解你的设备。'],
    ['led', '灯带控制', '色彩与节奏，由你编排。', '选择灯效，调整颜色、亮度与速度。按自己的顺序编排循环，把方案保存到本机。'],
    ['guide', '使用指南', '从连接，到第一次转动。', '沿着熟悉的操作流程，完成设备连接、身份核对与第一次低速调试。'],
  ];
  const defaults = {accent:'#26ed89',motion:true,speed:1,spacing:8,compact:false,logs:true};
  const storageKey = 'robstride.workspace.v1';
  let preferences = {...defaults};
  try {
    const saved = JSON.parse(localStorage.getItem(storageKey) || '{}');
    if (/^#[0-9a-f]{6}$/i.test(saved.accent)) preferences.accent = saved.accent;
    for (const key of ['motion','compact','logs']) if (typeof saved[key] === 'boolean') preferences[key] = saved[key];
    if (Number.isFinite(saved.speed)) preferences.speed = Math.min(2, Math.max(.2, saved.speed));
    if (Number.isFinite(saved.spacing)) preferences.spacing = Math.min(14, Math.max(5, saved.spacing));
  } catch { /* Defaults also work when browser storage is unavailable. */ }
  const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
  const smooth = () => reducedMotion.matches ? 'instant' : 'smooth';
  let selected = 'control';

  const navigation = document.createElement('div');
  navigation.className = 'monitor-navigation';
  navigation.innerHTML = `<div class="monitor-tabs" role="tablist" aria-label="工作空间模块">${modules.map(([id,label],i)=>`<button class="monitor-tab" id="tab-${id}" role="tab" aria-controls="slide-${id}" aria-selected="${i===0}" tabindex="${i===0?0:-1}" data-module="${id}"><span>0${i+1}</span>${label}</button>`).join('')}</div><div class="monitor-arrows"><button class="square-arrow" id="module-previous" aria-label="上一个模块">←</button><button class="square-arrow next" id="module-next" aria-label="下一个模块">→</button></div>`;
  const viewport = document.createElement('div');
  viewport.className = 'monitor-viewport';
  viewport.id = 'monitor-viewport';
  viewport.tabIndex = 0;
  viewport.setAttribute('role','region');
  viewport.setAttribute('aria-label','模块卡片，使用左右方向键切换');
  const track = document.createElement('div');
  track.className = 'monitor-track';
  viewport.append(track);
  const chart = byId('chart-section');
  const telemetry = document.createElement('section');
  telemetry.id = 'view-telemetry';
  telemetry.innerHTML = '<p class="telemetry-intro">当前电机由上方卡片选择。暂停只冻结曲线，后台继续采集；导出保留两个电机的记录。</p>';
  chart.parentNode.insertBefore(telemetry,chart);
  telemetry.append(chart);
  const first = byId('view-control');
  first.before(navigation,viewport);
  for (const [id,label,title,description] of modules) {
    const content = id === 'telemetry' ? telemetry : byId('view-'+id);
    content.classList.remove('hidden');
    const slide = document.createElement('section');
    slide.className = 'monitor-slide';
    slide.id = 'slide-'+id;
    slide.dataset.module = id;
    slide.setAttribute('role','tabpanel');
    slide.setAttribute('aria-labelledby','tab-'+id);
    slide.innerHTML = `<div class="module-window"><div class="window-bar"><span class="window-mark">⌘</span><span>${label}</span><span class="window-meta">ROBSTRIDE / 2026 AIC</span></div><div class="module-content"></div></div><div class="slide-caption"><div><h3>${title}</h3><div class="module-step">0${modules.findIndex(m=>m[0]===id)+1} / 05</div></div><p>${description}</p></div><button class="preview-cover" aria-label="切换到${label}" tabindex="-1"></button>`;
    slide.querySelector('.module-content').append(content);
    slide.querySelector('.preview-cover').onclick = () => switchView(id);
    track.append(slide);
  }
  const hint = document.createElement('p');
  hint.className = 'module-nav-hint';
  hint.textContent = '← → 切换模块 · 触摸滑动标题栏或卡片说明 · Esc 两台全部失能';
  viewport.after(hint);

  function position() {
    const slide = byId('slide-'+selected);
    if (!slide) return;
    track.style.transform = `translateX(${(viewport.clientWidth-slide.offsetWidth)/2-slide.offsetLeft}px)`;
    viewport.style.height = `${slide.offsetHeight+28}px`;
  }
  function show(id) {
    if (!modules.some(m=>m[0]===id)) return;
    selected = id;
    for (const [key] of modules) {
      const active = id===key, slide = byId('slide-'+key), tab = byId('tab-'+key);
      slide.classList.toggle('is-active',active);
      slide.setAttribute('aria-hidden',String(!active));
      slide.querySelector('.module-window').inert = !active;
      slide.querySelector('.preview-cover').hidden = active;
      tab.setAttribute('aria-selected',String(active));
      tab.tabIndex = active?0:-1;
    }
    byId('module-previous').disabled = id===modules[0][0];
    byId('module-next').disabled = id===modules.at(-1)[0];
    byId('log-section').classList.toggle('hidden',!preferences.logs || id==='guide');
    position();
    requestAnimationFrame(()=>{chartNeedsDraw=true;drawChart();});
  }
  window.consoleUi = {show};
  function move(delta, focusTab=false) {
    const current = modules.findIndex(m=>m[0]===selected), next = Math.max(0,Math.min(modules.length-1,current+delta));
    switchView(modules[next][0]);
    if (focusTab) byId('tab-'+selected).focus({preventScroll:true});
  }
  navigation.querySelector('.monitor-tabs').onclick = event => {
    const tab = event.target.closest('[data-module]');
    if (tab) switchView(tab.dataset.module);
  };
  byId('module-previous').onclick = ()=>move(-1);
  byId('module-next').onclick = ()=>move(1);
  document.querySelectorAll('.primary-nav .nav-item').forEach(button=>button.addEventListener('click',()=>navigation.scrollIntoView({behavior:smooth(),block:'start'})));
  for (const region of [navigation,viewport]) region.addEventListener('keydown',event=>{
    if (event.altKey || event.ctrlKey || event.metaKey || event.shiftKey || event.target.closest('input,select,textarea,[contenteditable=true]')) return;
    const inTabs = !!event.target.closest('[role=tablist]');
    if (event.key==='ArrowLeft' || event.key==='ArrowRight') {
      event.preventDefault();move(event.key==='ArrowRight'?1:-1,inTabs);
    } else if (inTabs && ['Home','End'].includes(event.key)) {
      event.preventDefault();switchView(event.key==='Home'?modules[0][0]:modules.at(-1)[0]);
      byId('tab-'+selected).focus({preventScroll:true});
    }
  });
  let startTouch = null;
  viewport.addEventListener('touchstart',event=>{
    const touch = event.touches[0];
    startTouch = event.touches.length===1 && event.target.closest('.window-bar,.slide-caption') ? {x:touch.clientX,y:touch.clientY} : null;
  },{passive:true});
  viewport.addEventListener('touchend',event=>{
    if (!startTouch) return;
    const touch = event.changedTouches[0],dx=touch.clientX-startTouch.x,dy=touch.clientY-startTouch.y;
    startTouch=null;
    if (Math.abs(dx)>60 && Math.abs(dx)>Math.abs(dy)*1.5) move(dx<0?1:-1);
  },{passive:true});
  viewport.addEventListener('touchcancel',()=>{startTouch=null;},{passive:true});
  const observer = new ResizeObserver(position);
  observer.observe(viewport);
  track.querySelectorAll('.monitor-slide').forEach(slide=>observer.observe(slide));
  byId('enter-console').onclick = ()=>{
    byId('console').scrollIntoView({behavior:smooth(),block:'start'});
    byId('console').focus({preventScroll:true});
  };
  byId('dismiss-announcement').onclick = ()=>byId('announcement').remove();

  const drawer=byId('settings-drawer'),scrim=byId('settings-scrim');
  let lastFocus=null, previousOverflow='';
  const outside = ()=>document.querySelectorAll('body>.site-header,body>.announcement,body>.app-shell,body>.back-top,body>.skip-link');
  function closeSettings() {
    if (drawer.hidden) return;
    drawer.hidden=true;scrim.hidden=true;
    outside().forEach(el=>{el.inert=false;});
    document.body.style.overflow=previousOverflow;
    lastFocus?.focus({preventScroll:true});
  }
  byId('open-settings').onclick = ()=>{
    lastFocus=document.activeElement;previousOverflow=document.body.style.overflow;
    drawer.hidden=false;scrim.hidden=false;document.body.style.overflow='hidden';
    outside().forEach(el=>{el.inert=true;});byId('close-settings').focus();
  };
  byId('close-settings').onclick=closeSettings;scrim.onclick=closeSettings;
  document.addEventListener('keydown',event=>{
    if (drawer.hidden) return;
    // Do not suppress Escape: app.js also sends the existing stop-all request.
    if (event.key==='Escape') closeSettings();
    if (event.key!=='Tab') return;
    const focusable=[...drawer.querySelectorAll('button,input,select,[tabindex="0"]')].filter(el=>!el.disabled);
    const first=focusable[0],last=focusable.at(-1);
    if (event.shiftKey&&document.activeElement===first){event.preventDefault();last.focus();}
    else if(!event.shiftKey&&document.activeElement===last){event.preventDefault();first.focus();}
  });

  function syncSettings() {
    byId('accent-color').value=preferences.accent;
    byId('motion-enabled').checked=preferences.motion;
    byId('motion-speed').value=preferences.speed;
    byId('pixel-density').value=preferences.spacing;
    byId('compact-layout').checked=preferences.compact;
    byId('show-logs').checked=preferences.logs;
    byId('motion-speed-value').textContent=Number(preferences.speed).toFixed(1)+'×';
    byId('pixel-density-value').textContent=preferences.spacing+' px';
  }
  function applyPreferences(save=false) {
    document.documentElement.style.setProperty('--accent',preferences.accent);
    document.body.classList.toggle('is-compact',preferences.compact);
    byId('log-section').classList.toggle('hidden',!preferences.logs||selected==='guide');
    syncSettings();
    if(save)try{localStorage.setItem(storageKey,JSON.stringify(preferences));}catch{toast('浏览器未允许保存，外观仅在本页生效',true);}
    requestDraw();requestAnimationFrame(position);
    document.dispatchEvent(new Event('upper-computer:config'));
  }
  for(const [id,key,type] of [['accent-color','accent','value'],['motion-enabled','motion','checked'],['compact-layout','compact','checked'],['show-logs','logs','checked'],['motion-speed','speed','number'],['pixel-density','spacing','number']]){
    byId(id).addEventListener('input',()=>{
      preferences[key]=type==='number'?Number(byId(id).value):byId(id)[type];applyPreferences(true);
    });
  }
  byId('reset-settings').onclick=()=>{
    preferences={...defaults};applyPreferences(true);
    for(const id of ['signalMotionEnabled','footerMotionEnabled']){
      byId(id).checked=true;byId(id).dispatchEvent(new Event('change'));
    }
  };

  // The same warped point field as the template; never represents motor data.
  let canvas=byId('pixelField');
  const hero=byId('overview');
  let width=0,height=0,dpr=1,frame=0,lastDraw=0;
  let worker=null,renderer=null,lastWorkerState='',transferred=false;
  function fallback(){
    worker?.terminate();worker=null;
    const replacement=canvas.cloneNode(false);canvas.replaceWith(replacement);canvas=replacement;
    renderer=new ControlDeskPixels(canvas);resizeField();requestDraw();
  }
  try{
    if(!canvas.transferControlToOffscreen||!window.Worker)throw new Error('Worker canvas unavailable');
    worker=new Worker('/pixel-field.js');
    const offscreen=canvas.transferControlToOffscreen();transferred=true;
    worker.postMessage({canvas:offscreen},[offscreen]);
    worker.addEventListener('error',event=>{event.preventDefault();fallback();},{once:true});
  }catch{
    if(transferred)fallback();
    else{worker?.terminate();worker=null;renderer=new ControlDeskPixels(canvas);}
  }
  function resizeField(){
    const box=hero.getBoundingClientRect();width=box.width;height=box.height;dpr=Math.min(devicePixelRatio||1,1.5);
    canvas.style.width=width+'px';canvas.style.height=height+'px';renderer?.resize(width,height,dpr);requestDraw();
  }
  function draw(now){
    frame=0;
    if(!preferences.motion){renderer?.context?.clearRect(0,0,width,height);return;}
    const box=hero.getBoundingClientRect();
    if(document.hidden||box.bottom<0||box.top>innerHeight)return;
    if(now-lastDraw<32){frame=requestAnimationFrame(draw);return;}
    lastDraw=now;
    renderer?.paint(now,preferences,preferences.accent,reducedMotion.matches);
    if(!reducedMotion.matches)frame=requestAnimationFrame(draw);
  }
  function requestDraw(){
    if(worker){
      const box=hero.getBoundingClientRect();
      const state={width,height,dpr,preferences:{enabled:preferences.motion,speed:preferences.speed,spacing:preferences.spacing},accent:preferences.accent,reduced:reducedMotion.matches,visible:!document.hidden&&box.bottom>=0&&box.top<=innerHeight};
      const signature=JSON.stringify(state);
      if(signature!==lastWorkerState){lastWorkerState=signature;worker.postMessage(state);}
      return;
    }
    if(!frame)frame=requestAnimationFrame(draw);
  }
  new ResizeObserver(resizeField).observe(hero);
  function scrollUpdate(){
    const max=document.documentElement.scrollHeight-innerHeight;
    document.querySelector('.scroll-progress').style.width=(max>0?scrollY/max*100:0)+'%';
    byId('back-to-top').hidden=scrollY<500;requestDraw();
  }
  addEventListener('scroll',scrollUpdate,{passive:true});
  document.addEventListener('visibilitychange',requestDraw);
  reducedMotion.addEventListener('change',requestDraw);
  byId('back-to-top').onclick=()=>window.scrollTo({top:0,behavior:smooth()});
  resizeField();applyPreferences();show('control');scrollUpdate();
})();
