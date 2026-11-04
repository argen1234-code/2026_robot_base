'use strict';
(() => {
  const groups=[['angle','航向角度 PID · 输出左右轮目标速度']];
  const gainNames=groups.flatMap(([name])=>['kp','ki','kd'].map(g=>name+'_'+g));
  const fields=['target_deg','left_rad_s','right_rad_s','turn_limit_rad_s','yaw_sign',...gainNames];
  const defaults={angle_kp:.04,angle_ki:0,angle_kd:.005};
  for(const [name,title] of groups){
    const group=document.createElement('div');group.className='chassis-gain-group';
    const h=document.createElement('h3');h.textContent=title;group.append(h);
    const row=document.createElement('div');row.className='chassis-gain-fields';
    for(const key of ['kp','ki','kd']){
      const label=document.createElement('label');label.textContent=key.toUpperCase();
      const input=document.createElement('input');input.id='chassis-'+name+'_'+key;input.type='number';input.step='any';input.value=defaults[name+'_'+key];label.append(input);row.append(label);
    }group.append(row);$('chassis-gains').append(group);
  }
  let points=[],lastTick=null,lastBoot=null,loaded=false,working=false;
  const current=()=>data?.snapshot?.chassis;
  function mapping(){return {left_slot:Number($('drive-left').value),left_sign:Number($('drive-left-sign').value),right_sign:Number($('drive-right-sign').value)};}
  function config(){
    const value=mapping();
    for(const name of fields){const el=$('chassis-'+name);if(el.value==='' || !Number.isFinite(Number(el.value)))throw new Error('请输入有效参数：'+name);value[name]=Number(el.value);}
    return value;
  }
  function load(){
    const c=current()?.config;if(!c)return;
    for(const name of fields)$('chassis-'+name).value=c[name];
    $('drive-left').value=String(c.left_slot);$('drive-left-sign').value=String(c.left_sign);$('drive-right-sign').value=String(c.right_sign);
  }
  async function command(code){await request('chassis',{command:code,config:code===4 || code===5?{}:config()});}
  async function operate(work){if(working)return;working=true;renderChassis();try{await work();await poll();}catch(e){toast(e.message,true);}finally{working=false;renderChassis();}}
  for(const [id,code] of [['apply',1],['speed',2],['angle',3],['stop',4],['zero',5]])$('chassis-'+id).onclick=()=>operate(()=>command(code));
  $('chassis-read').onclick=load;
  function draw(){
    const canvas=$('chassis-chart'),r=canvas.getBoundingClientRect();if(r.width<1)return;
    const dpr=devicePixelRatio||1;canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);
    const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);const w=r.width,h=r.height;
    const metric=$('chassis-metric').value;
    const keys=metric==='angle'?['target_deg','yaw_deg']:metric==='current'?['left_iq_a','right_iq_a']:[metric+'_target_rad_s',metric+'_speed_rad_s'];
    const samples=points.filter(p=>p.at>Date.now()-30000);const values=samples.flatMap(p=>keys.map(k=>p[k])).filter(Number.isFinite);
    $('chassis-empty').hidden=!(!values.length);
    if(!values.length)return;
    let min=Math.min(...values),max=Math.max(...values);const pad=Math.max((max-min)*.15,.2);min-=pad;max+=pad;
    const x=t=>50+(t-(Date.now()-30000))/30000*(w-65),y=v=>h-24-(v-min)/(max-min)*(h-45);
    ctx.font='11px Consolas';ctx.fillStyle='#82958a';ctx.strokeStyle='#23352a';
    for(let i=0;i<=4;i++){const v=min+(max-min)*i/4,py=y(v);ctx.beginPath();ctx.moveTo(48,py);ctx.lineTo(w-10,py);ctx.stroke();ctx.fillText(v.toFixed(1),4,py+4);}
    keys.forEach((key,i)=>{ctx.beginPath();ctx.lineWidth=2;ctx.strokeStyle=i?'#55c4dd':'#c2e55e';let started=false;for(const p of samples){if(!Number.isFinite(p[key]))continue;if(!started){ctx.moveTo(x(p.at),y(p[key]));started=true;}else ctx.lineTo(x(p.at),y(p[key]));}ctx.stroke();});
    ctx.fillStyle='#82958a';ctx.fillText('−30 s',50,h-6);ctx.fillText('现在',w-38,h-6);
  }
  function renderChassis(){
    const c=current(),live=connected() && !!data?.chassis_supported;
    const fresh=live && data?.snapshot?.chassis_coherent && data.sample_age_ms<600;
    const motors=[data?.snapshot?.motor,data?.snapshot?.motor2],pending=working || !!data?.pending_motors?.some(Boolean);
    const active=!!c?.active,drive=!!data?.drive_active;
    if(live && !loaded && fresh){load();loaded=true;}
    if(!live)loaded=false;
    const ready=live && !data?.initializing && !pending && !drive && motors.every(m=>m?.mode===2 && m.enabled && m.enable_requested && [2,3,4].includes(m.state));
    $('chassis-speed').disabled=!ready;$('chassis-angle').disabled=!ready || !c?.imu_online;
    $('chassis-stop').disabled=!live || working;$('chassis-apply').disabled=!live || working;
    $('chassis-zero').disabled=!live || working || active || drive;
    $('chassis-read').disabled=!live || working;
    $('chassis-yaw_sign').disabled=active || drive;
    if(active)for(const id of ['run','drive-enter','drive-left','drive-left-sign','drive-right-sign'])$(id).disabled=true;
    text('chassis-status',!live?'等待连接新版固件':!fresh?'底盘快照过期 / 无效':`${active?(c.mode===2?'绝对角度保持中':'双轮速度控制中'):drive?'WASD 控制中':'底盘待命'} · JY901S ${c.imu_online?'在线':'未收到新角度'} · 回执 ${c.executed_seq} / ${resultNames[c.result]??c.result}`);
    if(!fresh){for(const id of ['yaw','angle-output','left-live','right-live'])text('chassis-'+id,'—');draw();return;}
    text('chassis-yaw',`${format(c.yaw_deg,2)}° / ${format(c.target_deg,2)}°`);
    text('chassis-angle-output',`${format(c.angle_error_deg,2)}° / ${format(c.turn_rad_s,3)} rad/s`);
    for(const name of ['left','right'])text('chassis-'+name+'-live',`${format(c[name+'_target_rad_s'])} / ${format(c[name+'_speed_rad_s'])} rad/s / ${format(c[name+'_iq_a'])} A`);
    for(const side of ['left','right']){const slot=side==='left'?c.config.left_slot:1-c.config.left_slot,m=slot?data.snapshot.motor2:data.snapshot.motor;
      text('chassis-'+side+'-mount',`前进为正；电机 ${slot+1} · ID ${m.configured_id} · ${c.config[side+'_sign']===1?'正转':'反转'}前进`);}
    const age=(data.snapshot.system.tick_ms-c.imu.angle_tick_ms)>>>0;
    text('chassis-imu',`原始航向 ${format(c.imu.raw_yaw_deg,2)}° · Z 角速度 ${format(c.imu.gyro_z_deg_s,2)}°/s · 帧 ${c.imu.angle_frames} · ${age} ms · 校验错 ${c.imu.checksum_errors}`);
    if(lastBoot!==null && c.published_tick_ms<lastBoot){points=[];lastTick=null;}
    lastBoot=c.published_tick_ms;
    if(c.published_tick_ms!==lastTick){lastTick=c.published_tick_ms;points.push({...c,at:Date.now()});points=points.filter(p=>p.at>Date.now()-30000);}
    draw();
  }
  $('chassis-metric').onchange=draw;$('chassis-panel').addEventListener('toggle',draw);
  new ResizeObserver(draw).observe($('chassis-chart'));
  window.syncChassisMapping=()=>request('chassis',{command:1,config:mapping()});window.renderChassis=renderChassis;
  renderChassis();
})();
