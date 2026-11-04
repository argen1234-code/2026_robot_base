'use strict';
(() => {
  const keys=new Set(), codes=['KeyW','KeyA','KeyS','KeyD'];
  let owner=null, starting=false, sending=false, dirty=false;
  const settings=['drive-speed','drive-left','drive-left-sign','drive-right-sign'];
  const editing=el=>!!el?.closest('input,select,textarea,[contenteditable=true],[role=dialog]');
  function targets() {
    const speed=Number($('drive-speed').value);
    if($('drive-speed').value==='' || !Number.isFinite(speed) || speed<0 || speed>MAX_SPEED_RAD_S)
      throw new Error('轮速需在现有范围 0–33 rad/s 内');
    const f=Number(keys.has('KeyW'))-Number(keys.has('KeyS'));
    const t=Number(keys.has('KeyD'))-Number(keys.has('KeyA'));
    const divisor=Math.max(1,Math.abs(f)+Math.abs(t));
    const left=(f+t)/divisor*speed*Number($('drive-left-sign').value);
    const right=(f-t)/divisor*speed*Number($('drive-right-sign').value);
    return $('drive-left').value==='0'?[left,right]:[right,left];
  }
  function drawKeys() {
    document.querySelectorAll('[data-drive-key]').forEach(el=>el.classList.toggle('pressed',keys.has(el.dataset.driveKey)));
  }
  function forget() {owner=null;keys.clear();dirty=false;drawKeys();renderControls();}
  async function leave() {
    if(starting){starting=false;}
    const previous=owner;
    forget();
    if(!previous)return;
    try {await request('drive_exit',{owner:previous});}
    catch(error){toast(error.message,true);}
    finally{await poll();}
  }
  async function send() {
    if(!owner)return;
    dirty=true;
    if(sending)return;
    sending=true;
    const session=owner;
    try {
      do {
        dirty=false;
        await request('drive_update',{owner:session,targets:targets()});
      } while(dirty && owner===session);
    } catch(error) {
      if(owner===session){toast(error.message,true);await leave();}
    } finally{sending=false;}
  }
  function zero(){keys.clear();drawKeys();void send();}
  function renderDrive() {
    const active=!!owner || !!data?.drive_active;
    const snap=data?.snapshot, motors=[snap?.motor,snap?.motor2];
    const ready=connected() && !data?.initializing && data?.drive_supported && snap?.coherent && snap?.coherent2 &&
      snap?.setup?.dual_ready && !data?.setup_pending && !data?.pending_motors?.some(Boolean) &&
      motors.every(m=>m && [2,3,4].includes(m.state) && m.enabled && m.enable_requested && !m.job_command && m.online);
    $('drive-enter').disabled=!!(active || starting || busy || !ready);
    $('drive-exit').disabled=!owner;
    $('drive-zero').disabled=!owner;
    settings.forEach(id=>$(id).disabled=active || starting);
    text('drive-status',owner?'键盘控制已开启 · 按住 WASD 行驶，松键归零':active?'小车模式由其他页面控制':
      connected() && !data?.drive_supported?'请下载本次新版 H7 固件后使用 WASD':
      ready?'可进入键盘控制 · 进入时接管两轮目标并归零，无需等待实测零速':
      data?.initializing?data.startup_message:'双电机尚未就绪 · 重新连接可自动配置并使能');
    if(active) {
      for(const id of ['run','speed-limit','current-limit','accel-limit'])$(id).disabled=true;
    }
    if(owner && !starting && (!connected() || !data?.drive_active)) {
      // Fresh status follows the acquire receipt; H7 can revoke the session.
      forget();
    }
  }
  $('drive-enter').onclick=async()=>{
    if(starting || owner)return;
    try {
      targets();
      starting=true;renderControls();
      if(data?.chassis_supported)await window.syncChassisMapping?.();
      const result=await request('drive_enter');
      // Losing focus while acquire was in flight cancels its resulting session.
      if(!starting){await request('drive_exit',{owner:result.owner});return;}
      owner=result.owner;keys.clear();
      // Finish any pre-acquire status read, then request post-acquire status.
      while(polling)await new Promise(resolve=>setTimeout(resolve,5));
      await poll();
      $('drive-enter').blur();
    } catch(error){toast(error.message,true);}
    finally{starting=false;renderControls();}
  };
  $('drive-exit').onclick=leave;
  $('drive-zero').onclick=zero;
  $('drive-panel').addEventListener('toggle',()=>{if(!$('drive-panel').open)void leave();});
  document.addEventListener('keydown',event=>{
    if(!owner)return;
    if(event.code==='Escape'){forget();return;}
    if(editing(event.target) || event.ctrlKey || event.altKey || event.metaKey || event.isComposing)return;
    if(event.repeat)return;
    if(event.code==='Space'){event.preventDefault();zero();return;}
    if(!codes.includes(event.code))return;
    event.preventDefault();
    if(!keys.has(event.code)){keys.add(event.code);drawKeys();void send();}
  });
  document.addEventListener('keyup',event=>{
    if(keys.delete(event.code)){event.preventDefault();drawKeys();void send();}
  });
  document.addEventListener('focusin',event=>{if(owner && editing(event.target))zero();});
  window.addEventListener('blur',()=>void leave());
  window.addEventListener('pagehide',()=>{
    if(owner)void fetch('/api/drive_exit',{method:'POST',keepalive:true,
      headers:{'Content-Type':'application/json','X-Console-Token':sessionToken},body:JSON.stringify({owner})});
    forget();
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden)void leave();});
  setInterval(()=>{if(owner)void send();},180);
  window.renderDrive=renderDrive;window.leaveDrive=leave;window.forgetDrive=forget;
  renderDrive();
})();
