'use strict';
(() => {
  let state={},session=null,direction='S',keys=new Set(),keyboard=false,seq=0,chain=Promise.resolve(),pending=0,polling=false;
  const directions=[...document.querySelectorAll('[data-bt-motion]')];
  const editing=target=>target?.closest('input,textarea,select,[contenteditable=true]');
  const codes=['KeyW','KeyA','KeyS','KeyD'];
  async function call(action,payload={}) {
    if(!sessionToken)sessionToken=(await(await fetch('/api/session')).json()).token;
    const response=await fetch('/api/bluetooth/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':sessionToken},body:JSON.stringify({session,...payload})});
    const value=await response.json();if(!response.ok)throw new Error(value.error||'蓝牙操作失败');return value.data;
  }
  function enqueue(action,payload={}) {
    pending++;
    const work=chain.then(()=>call(action,payload));
    chain=work.catch(error=>{toast(error.message,true);}).finally(()=>{pending--;});
    return work;
  }
  function display() {
    const connected=!!state.connected,armed=connected&&state.armed;
    $('bt-connect').disabled=connected;$('bt-disconnect').disabled=!connected;
    $('bt-port').disabled=connected;$('bt-arm').disabled=!connected||!!state.phase;
    $('bt-disable').disabled=!connected;$('bt-stop').disabled=!connected;
    $('bt-keyboard').disabled=!armed;$('bt-apply-speed').disabled=!connected;
    for(const button of directions){button.disabled=!armed;button.classList.toggle('pressed',direction===button.dataset.btMotion);}
    for(const id of ['bt-cap','bt-current','bt-accel','bt-left','bt-left-sign','bt-right-sign'])$(id).disabled=!!(state.armed||state.phase);
    $('bt-keyboard').setAttribute('aria-pressed',String(keyboard));
    $('bt-keyboard').textContent=keyboard?'关闭蓝牙 WASD':'开启蓝牙 WASD';
    $('bt-status').textContent=!connected?'蓝牙未连接':state.phase?'正在确认电机参数并零速使能…':state.error?state.error:armed?'蓝牙遥控已使能 · 按住方向行驶':'蓝牙已连接 · 请使能遥控';
    $('bt-live').textContent=connected?`${state.port} · ${state.identity||'等待身份'} · 轮速 ${state.speed??'—'} rad/s · M1 ${Number(state.v1??0).toFixed(3)} / M2 ${Number(state.v2??0).toFixed(3)} rad/s`:'等待固件回执';
    const log=$('bt-log');log.textContent=state.logs?.map(x=>`${x.time} ${x.text}`).join('\n')||'尚无蓝牙数据';log.scrollTop=log.scrollHeight;
  }
  async function pollBluetooth() {
    if(polling)return;polling=true;
    try {
      const response=await fetch('/api/bluetooth/status');if(!response.ok)return;
      const next=await response.json();if(typeof next.identity!=='string'||typeof next.port!=='string')return;state=next;
      if(!state.connected || (!state.armed&&!state.phase)){direction='S';keys.clear();keyboard=false;}
      display();
    } catch(_){} finally{polling=false;}
  }
  function motion(next) {
    direction=next;display();
    if(state.connected)void enqueue('motion',{direction:next,seq:++seq}).catch(()=>{direction='S';keys.clear();keyboard=false;display();});
  }
  function stop(disable=false) {
    direction='S';keys.clear();keyboard=false;display();
    if(state.connected)void enqueue('stop',{disable}).catch(()=>{});
  }
  function keyDirection() {
    const f=Number(keys.has('KeyW'))-Number(keys.has('KeyS'));
    const t=Number(keys.has('KeyA'))-Number(keys.has('KeyD'));
    if(f>0)return t>0?'FL':t<0?'FR':'F';
    if(f<0)return t>0?'BL':t<0?'BR':'B';
    return t>0?'L':t<0?'R':'S';
  }
  async function refresh() {
    const ports=await enqueue('ports'),selected=$('bt-port').value;
    $('bt-port').replaceChildren(new Option('请选择 COM 口',''));
    for(const port of ports)$('bt-port').add(new Option(`${port.device} · ${port.description}`,port.device));
    $('bt-port').value=selected;
  }
  $('bt-refresh').onclick=()=>void refresh().catch(()=>{});
  $('bt-connect').onclick=async()=>{
    try{const value=await enqueue('connect',{port:$('bt-port').value});session=value.session;seq=0;await pollBluetooth();}
    catch(_){}
  };
  $('bt-disconnect').onclick=async()=>{stop(true);try{await enqueue('disconnect');session=null;await pollBluetooth();}catch(_){}};
  $('bt-arm').onclick=async()=>{
    direction='S';keys.clear();keyboard=false;
    if(data?.drive_active || data?.snapshot?.chassis?.active){toast('请先退出 SWD 小车/底盘控制，再使能蓝牙遥控',true);return;}
    const number=id=>$(id).value.trim()===''?NaN:Number($(id).value);
    const payload={cap:number('bt-cap'),current:number('bt-current'),accel:number('bt-accel'),speed:number('bt-speed'),left_slot:number('bt-left'),left_sign:number('bt-left-sign'),right_sign:number('bt-right-sign')};
    if(Object.values(payload).some(x=>!Number.isFinite(x))){toast('请输入完整的有效蓝牙参数',true);return;}
    try{await enqueue('arm',payload);await pollBluetooth();}catch(_){}
  };
  $('bt-disable').onclick=()=>stop(true);$('bt-stop').onclick=()=>stop();
  $('bt-apply-speed').onclick=async()=>{stop();try{await enqueue('speed',{speed:$('bt-speed').value.trim()===''?null:Number($('bt-speed').value)});await pollBluetooth();}catch(_){}};
  $('bt-keyboard').onclick=async()=>{
    if(keyboard){stop();return;}
    if(window.leaveDrive)await window.leaveDrive();
    keyboard=true;keys.clear();$('bt-keyboard').blur();display();
  };
  for(const button of directions) {
    button.addEventListener('pointerdown',event=>{if(button.disabled)return;event.preventDefault();keys.clear();button.setPointerCapture(event.pointerId);motion(button.dataset.btMotion);});
    for(const name of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(name,()=>{if(direction!=='S')motion('S');});
  }
  document.addEventListener('keydown',event=>{
    if(event.code==='Escape'){stop(true);return;}
    if(!keyboard || editing(event.target) || event.altKey || event.ctrlKey || event.metaKey || event.isComposing)return;
    if(event.code==='Space'){event.preventDefault();keys.clear();motion('S');return;}
    if(codes.includes(event.code)){event.preventDefault();if(!event.repeat){keys.add(event.code);motion(keyDirection());}}
  });
  document.addEventListener('keyup',event=>{if(keys.delete(event.code)){event.preventDefault();motion(keyDirection());}});
  document.addEventListener('focusin',event=>{if(direction!=='S'&&editing(event.target))stop();});
  window.addEventListener('blur',()=>{if(direction!=='S'||keyboard)stop();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
  $('bluetooth-panel').addEventListener('toggle',()=>{if(!$('bluetooth-panel').open)stop();else void refresh().catch(()=>{});});
  window.addEventListener('pagehide',()=>{
    if(state.connected)void fetch('/api/bluetooth/stop',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Console-Token':sessionToken},body:JSON.stringify({disable:true})});
  });
  window.stopBluetooth=stop;
  setInterval(()=>{if((direction!=='S'||keyboard)&&activeView!=='control')stop();if(direction!=='S'&&!pending)motion(direction);},180);
  setInterval(pollBluetooth,300);void pollBluetooth();display();
})();
