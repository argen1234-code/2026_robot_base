'use strict';
const $ = id => document.getElementById(id);
const states = ['已失能', '准备就绪', '已使能', '运行中', '减速中', '故障', '准备中', '使能中'];
const resultNames = ['成功','未知命令','参数越界','状态不允许','确认键错误','电机离线','CAN 发送失败','参数读回不符 / 超时','电机故障','状态确认超时','CAN Bus-off','保护触发','保留结果 12','健康检查失败','超速','服务间断','正在执行','固件忙'];
let sessionToken = '', data = null, unit = 'rad', activeView = 'control', busy = false;
let samples = [], paused = false, lastTick = null, toastTimer, lastLogKey = '', bridgeAvailable = false;
let lastStatusAt = 0, chartNeedsDraw = true;
let exited = false, selectedMotor = 0;
const edits = [{target:1,cap:33,current:2.4,accel:20},{target:1,cap:33,current:2.4,accel:20}];
const selectedPending=()=>data?.pending_motors?.[selectedMotor] ?? (selectedMotor===0?data?.pending:null);
const continuous=()=>data?.continuous_motors?.[selectedMotor] ?? (selectedMotor===0?data?.continuous:false);
const selectedCoherence=()=>selectedMotor===0?data?.snapshot?.coherent:data?.snapshot?.coherent2;
const MAX_SPEED_RAD_S = 33;
const format = (n, places=3) => Number.isFinite(n) ? n.toFixed(places) : '—';
const hex = n => '0x' + (n >>> 0).toString(16).toUpperCase();
const text = (id, value) => { $(id).textContent = value; };
const esc = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

function toast(message, error=false) {
  clearTimeout(toastTimer);
  text('toast', message);
  $('toast').className = 'toast' + (error ? ' error' : '');
  toastTimer = setTimeout(() => $('toast').classList.add('hidden'), 5500);
}

async function request(action, payload={}) {
  const response = await fetch('/api/' + action, {method:'POST', headers:{'Content-Type':'application/json', 'X-Console-Token':sessionToken}, body:JSON.stringify(payload)});
  const body = await response.json();
  if (!response.ok) throw new Error(body.error || '请求失败');
  return body.data;
}

async function action(work) {
  if (busy) return;
  busy = true; renderControls();
  try { await work(); }
  catch (error) { toast(error.message, true); }
  finally { busy = false; await poll(); renderControls(); }
}

function connected() { return !!(bridgeAvailable && data?.connected && performance.now() - lastStatusAt < 800); }
function motor() { return (selectedMotor===0?data?.snapshot?.motor:data?.snapshot?.motor2) || {}; }
function coherent() { return connected() && data.sample_age_ms < 600 && !!selectedCoherence(); }
function validFeedback() { return coherent() && !!motor().online && motor().feedback_age_ms + (data?.sample_age_ms || 0) < 150; }
function badge(id, message, mode='') {
  const el = $(id);
  el.className = 'badge' + (id === 'motor-badge' ? ' small' : '') + (mode ? ' ' + mode : '');
  el.innerHTML = `<span class="dot ${mode === 'online' ? 'green' : mode === 'error' ? 'red' : ''}"></span>${esc(message)}`;
}

function renderControls() {
  if (window.renderLed) renderLed();
  const live = connected(), m = motor(), state = m.state;
  const idle = !busy && !selectedPending() && !m.job_command;
  text('run-status', continuous() ? '● 持续旋转中' : '等待开始');
  $('run-status').className = continuous() ? 'green-text' : '';
  $('run').innerHTML = continuous() ? '应用新速度 ↗' : '开始运行 ▶';
  $('connect').disabled = busy || (!live && !$('probe').value);
  $('refresh').disabled = busy || live;
  $('probe').disabled = busy || live;
  $('connect').innerHTML = busy ? '处理中…' : live ? '断开连接 <span>↗</span>' : '连接设备 <span>↗</span>';
  $('run').disabled = !(validFeedback() && idle && !data?.initializing && m.enabled && m.enable_requested && [2,3,4].includes(state));
  document.querySelector('.control-panel .chip').textContent='电机速度模式';
  $('stop').disabled = !(live && !busy && !m.job_command && (!selectedPending() || selectedPending().command===3) && [2,3,4].includes(state));
  // Urgent disable remains available while a normal request is in flight.
  $('disable').disabled = !live;
  $('clear-fault').disabled = !live || busy;
  for (const id of ['speed-limit','current-limit','accel-limit']) $(id).disabled = busy || live && ![0,1,5].includes(state);
  $('export').disabled = !(data?.history_count > 0);
  window.renderDrive?.();
  window.renderChassis?.();
}

function render() {
  renderDual();
  const m = motor(), live = connected(), fresh = validFeedback();
  badge('connection-badge', live ? 'H743 已连接' : '设备未连接', live ? 'online' : '');
  badge('motor-badge', !live ? '等待反馈' : !selectedCoherence() ? '快照不一致' : fresh ? '电机在线' : '电机离线', fresh ? 'online' : live ? 'error' : '');
  $('sidebar-dot').className = 'dot' + (fresh ? ' green' : '');
  text('sidebar-state', fresh ? states[m.state] || '未知状态' : live ? selectedCoherence() ? '已连接 · 反馈过期' : '已连接 · 快照无效' : '未连接设备');
  text('footer-status', live ? fresh ? '反馈正常' : '等待有效反馈' : '等待设备连接');
  $('footer-dot').className = 'dot' + (fresh ? ' green' : '');
  text('app-state', live ? states[m.state] || '未知' : '—');
  text('feedback-age', live && m.feedback_age_ms < 0xFFFFFFFF ? m.feedback_age_ms + data.sample_age_ms + ' ms' : '— ms');
  text('motor-id', live ? hex(m.configured_id) : '0x7F');
  text('coherent', live ? selectedCoherence() ? '一致' : '不一致' : '—');
  text('velocity', fresh ? format(m.velocity_rad_s) : '—');
  text('applied', coherent() ? format(m.applied_rad_s) : '—');
  text('torque', fresh ? format(m.torque_nm) : '—');
  text('temperature', fresh ? format(m.temperature_c,1) : '—');
  const iqFresh = coherent() && m.current_age_ms + data.sample_age_ms < 500, vFresh = coherent() && m.voltage_age_ms + data.sample_age_ms < 500;
  text('current', iqFresh ? format(m.current_a) : '—');
  text('voltage', vFresh ? format(m.bus_voltage_v,1) : '—');
  text('current-age', iqFresh ? `读回年龄 ${m.current_age_ms + data.sample_age_ms} ms` : '参数反馈无效 / 过期');
  $('position').innerHTML = `${fresh ? format(m.position_rad) : '—'}<small>rad</small>`;
  text('position-deg', fresh ? format(m.position_rad * 180 / Math.PI,1) + '° / 输出轴角度' : '等待有效反馈');
  if (fresh) $('joint-arm').setAttribute('transform', `rotate(${m.position_rad * 180 / Math.PI})`);
  let notice = '先在 Keil 中运行固件并退出 Debug，再连接调试器。连接后自动配置并零速使能两台电机。', error = false;
  if (live) {
    notice = continuous() ? '持续运行中：速度指令保持有效。修改速度后点击「应用新速度」；点击减速停止或立即失能结束。' : '双电机已就绪。可直接设置角度或速度并运行，也可进入 WASD 控制。';
    if (!selectedCoherence()) {notice='正在获取一致快照，本次测量无效；运动操作暂不可用。';error=true;}
    else if (m.fault || m.fault_bits || m.health_fault || m.state===5) {notice=`固件状态：${resultNames[m.reject_reason] || '故障'}；故障摘要 ${hex(m.fault_bits)}，详细故障 ${hex(m.fault)}。请到通信诊断查看。`;error=true;}
    else if (!fresh) {notice='H743 已连接，但尚无有效电机反馈。请检查电机供电、CAN 接线、ID 与终端电阻。';error=true;}
    if (m.reject_reason===7 && m.readback_index) notice+=` 参数 ${hex(m.readback_index)}：${['未记录','匹配','电机拒绝读取','数值不符','等待回复超时'][m.readback_status]||'未知'}，期望 ${hex(m.readback_expected)}，实际 ${hex(m.readback_actual)}。`;
    if (selectedPending() && !continuous()) notice=`序号 ${selectedPending().seq} 已提交，等待固件执行确认…`;
  }
  if (live && data?.initializing) notice=data.startup_message;
  else if (live && !data?.motors_ready && !error) notice=data?.startup_message || '双电机尚未就绪，重新连接可自动配置并使能';
  if (!bridgeAvailable) {notice=exited?'上位机已退出，调试器已释放。下次使用请双击启动程序。':'本地服务连接中断。当前数据无效，请检查上位机进程。';error=!exited;}
  if(live && activeView==='led') {
    notice=data?.led_supported?'灯带控制无需使能电机。编辑后点击应用；循环在开发板上运行，断开连接后仍继续播放。':'当前固件仅支持灯带状态读取，请下载新版 RTOS 固件以开放灯效控制。';
    error=false;
  }
  text('notice-text',notice);$('notice').className='notice'+(error?' error':'');
  text('firmware-path',data?.profile?.source || '—');
  text('firmware-hash','SHA256  '+(data?.profile?.axf_sha256 || '—'));
  renderDiagnostics(); renderLogs(); renderControls();
  const tick = data?.snapshot?.system?.tick_ms;
  if (live && tick !== lastTick) {
    lastTick = tick;
    const row = {time:Date.now(), valid:fresh, ...m};
    row.currentValid = iqFresh;row.voltageValid = vFresh;
    samples.push(row);
    samples = samples.filter(p=>p.time>Date.now()-31000);
    chartNeedsDraw = true;
  }
  text('sample-rate',data?.history_count || 0);
  text('chart-status',live ? `${selectedCoherence() ? '一致快照' : '快照无效'} · 采样 ${data.sample_age_ms} ms 前${paused?' · 显示已暂停':''}` : '尚未连接 · 不生成模拟数据');
  if (!paused && chartNeedsDraw) {drawChart();chartNeedsDraw=false;}
}

function renderLogs() {
  const rows = data?.logs || [], key = JSON.stringify(rows);
  if (key === lastLogKey) return;
  lastLogKey = key;
  text('log-count', rows.length + ' 条事件');
  $('logs').innerHTML = rows.map(r=>`<div class="log-row ${['拒绝','超时','连接中断'].includes(r.kind)?'error':''}"><span class="log-time">${esc(r.time)}</span><span class="log-kind">${esc(r.kind)}</span><span class="log-message">${esc(r.message)}</span></div>`).join('');
  $('logs').scrollTop = $('logs').scrollHeight;
}

function renderDiagnostics() {
  const m=motor(), c=m.can||{}, s=data?.snapshot?.system||{}, live=connected();
  const items = [['CAN RX',c.rx],['CAN TX 成功',c.tx_ok],['CAN TX 失败',c.tx_fail],['RX 溢出',c.rx_drop],
    ['CAN 错误',c.errors],['Bus-off 计数',c.bus_off],['TEC / REC',`${c.tec??'—'} / ${c.rec??'—'}`],['恢复次数',c.recoveries],
    ['有效电机反馈',m.feedback_count],['参数读回次数',m.parameter_count],['未匹配帧',m.unmatched_frames],['最近 CAN ID',hex(c.last_rx_id||0)],
    ['故障摘要',hex(m.fault_bits||0)],['详细故障',hex(m.fault||0)],['预警',hex(m.warning||0)],['最近执行结果',resultNames[m.result]||m.result],
    ['接受 / 执行序号',`${m.accepted_seq??'—'} / ${m.executed_seq??'—'}`],['电机侧超时',m.timeout_readback===5000?'250 ms（读回）':m.timeout_readback],
    ['H743 主循环',s.loop_heartbeat],['最大服务耗时',s.max_service_us===undefined?'—':s.max_service_us+' µs'],
    ['系统启动 / Fatal',`${s.boot_stage??'—'} / ${s.fatal??'—'}`],['健康故障',hex(m.health_fault||0)],
    ['电机身份 ID',m.identity_valid?hex(m.identity_id):'未收到'],['OLED 完成帧',data?.snapshot?.oled?.refresh_ok],
    ['最近核对参数',hex(m.readback_index||0)],['参数核对结果',['未核对','匹配','电机拒绝','数值不符','超时'][m.readback_status||0]],
    ['期望 / 实际原始值',`${hex(m.readback_expected||0)} / ${hex(m.readback_actual||0)}`],
    ['加速度执行',m.accel_motor_supported?'H743 斜坡 + 电机参数':'H743 速度斜坡（兼容）'],['持续旋转',continuous()?'运行中':'未运行']];
  items.push(['电机超时参数',m.timeout_motor_supported?`${hex(m.timeout_parameter)} · 已读回`:'电机拒绝 · 未验证断线自停']);
  $('diagnostic-grid').innerHTML=items.map(([label,value])=>`<div class="diagnostic-item"><span>${esc(label)}</span><b>${esc(live ? value??'—' : '—')}</b></div>`).join('');
}

function targetRad() {
  const n=Number($('target').value), cap=Number($('speed-limit').value);
  const rad=unit==='rpm'?n*Math.PI/30:n;
  // Exact range endpoints must survive a rad/s -> rpm -> rad/s roundtrip.
  return Math.abs(Math.abs(rad)-cap)<1e-12 ? Math.sign(rad)*cap : rad;
}
function setTarget(rad) {
  const cap=Number($('speed-limit').value)||MAX_SPEED_RAD_S;
  rad=Math.max(-cap,Math.min(cap,rad));
  $('target').value=String(unit==='rpm'?rad*30/Math.PI:rad);
  $('speed-slider').value=rad;
}
function updateRange() {
  const cap=Number($('speed-limit').value);
  if (!(cap>0&&cap<=MAX_SPEED_RAD_S)) return;
  $('speed-slider').min=-cap;$('speed-slider').max=cap;
  const scale=unit==='rpm'?30/Math.PI:1;
  $('target').min=-cap*scale;$('target').max=cap*scale;$('target').step='any';
  text('min-speed','−'+format(cap*scale,2));text('max-speed','+'+format(cap*scale,2));
  setTarget(targetRad());
  text('speed-range-note',`速度模式范围：±${format(cap,2)} rad/s / ±${format(cap*30/Math.PI,2)} rpm；加速度决定提速时间。`);
}

async function command(number) {
  const payload = {motor:selectedMotor, command:number, target_rad_s:number===3?targetRad():0,
    speed_limit_rad_s:Number($('speed-limit').value), current_limit_a:Number($('current-limit').value), accel_rad_s2:Number($('accel-limit').value)};
  if (number<=3) {
    if (!$('target').value || !['speed-limit','current-limit','accel-limit'].every(id=>$(id).value!=='')) throw new Error('请填写完整数值');
    if (!Object.values(payload).every(Number.isFinite)) throw new Error('请输入有效数值');
  }
  const result=await request('command',payload);
  toast(`序号 ${result.seq} 已提交，等待固件确认`);
}

function switchView(name) {
  if(name!=='control')window.leaveDrive?.();
  activeView=name;
  document.querySelectorAll('.nav-item').forEach(el=>el.classList.toggle('active',el.dataset.view===name));
  window.consoleUi?.show(name);
  const info=name==='led'?['灯带控制','让每一束光，','随你而动。','选择灯效、调整外观，编排属于你的循环播放列表。']:{control:['电机控制','让每一次转动，','尽在掌握。','从连接到控制，实时了解你的每一个关节。'],telemetry:['数据观测','每一组数据，','清晰可见。','同步观测关节运动与电气状态，导出真实采集记录。'],diagnostics:['通信诊断','让连接状态，','有据可查。','从固件心跳到 CAN 收发，保留每一条诊断证据。'],guide:['使用指南','从第一次连接，','开始掌控。','沿用当前工程的控制流程，完成一次低速联调。']}[name];
  text('crumb',info[0]);$('page-title').innerHTML=esc(info[1])+'<em>'+esc(info[2])+'</em>';text('page-description',info[3]);
  chartNeedsDraw=true;if(!paused)drawChart();
}

function drawChart() {
  if (activeView !== 'telemetry') return;
  const canvas=$('chart'),r=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
  if(!r.width)return;
  canvas.width=r.width*dpr;canvas.height=r.height*dpr;
  const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);
  const w=r.width,h=r.height,pad={left:49,right:16,top:14,bottom:28},pw=w-pad.left-pad.right,ph=h-pad.top-pad.bottom;
  const key=$('chart-metric').value,now=Date.now();
  const pts=samples.filter(p=>p.time>=now-30000);
  const valid=p=>key==='current_a'?p.currentValid:key==='bus_voltage_v'?p.voltageValid:p.valid;
  const values=pts.filter(valid).map(p=>p[key]).filter(Number.isFinite);
  if(key==='velocity_rad_s'&&$('show-target').checked)values.push(...pts.filter(p=>p.valid).map(p=>p.target_rad_s));
  let low=Math.min(0,...values),high=Math.max(key==='velocity_rad_s'?.1:.01,...values),range=high-low;
  low-=range*.15;high+=range*.15;range=high-low;
  const y=v=>pad.top+(high-v)/range*ph,x=t=>pad.left+(t-(now-30000))/30000*pw;
  ctx.strokeStyle='#223029';ctx.lineWidth=1;ctx.font='9px Consolas, monospace';ctx.fillStyle='#536d5d';
  for(let i=0;i<=4;i++){const yy=pad.top+ph*i/4;ctx.beginPath();ctx.moveTo(pad.left,yy);ctx.lineTo(w-pad.right,yy);ctx.stroke();ctx.textAlign='right';ctx.fillText((high-range*i/4).toFixed(key==='temperature_c'||key==='bus_voltage_v'?1:3),pad.left-9,yy+3);}
  for(let i=0;i<=6;i++){const xx=pad.left+pw*i/6;ctx.beginPath();ctx.moveTo(xx,pad.top);ctx.lineTo(xx,h-pad.bottom);ctx.stroke();ctx.textAlign='center';ctx.fillText(i===6?'现在':`−${30-i*5}s`,xx,h-9);}
  const line=(field,color,dashed)=>{ctx.beginPath();let drawing=false;for(let i=0;i<pts.length;i++){const p=pts[i];if(!valid(p)||!Number.isFinite(p[field])){drawing=false;continue;}if(i&&p.time-pts[i-1].time>450)drawing=false;const xx=x(p.time),yy=y(p[field]);if(!drawing)ctx.moveTo(xx,yy);else ctx.lineTo(xx,yy);drawing=true;}ctx.strokeStyle=color;ctx.lineWidth=dashed?1:1.7;ctx.setLineDash(dashed?[5,5]:[]);ctx.stroke();ctx.setLineDash([]);};
  if($('show-target').checked&&key==='velocity_rad_s')line('target_rad_s','#9caaa1',true);
  if($('show-actual').checked)line(key,'#29ee91',false);
  $('chart-empty').classList.toggle('hidden',values.length>0);
  const spark=samples.filter(p=>p.valid).slice(-35);
  if(spark.length>1){const lo=Math.min(...spark.map(p=>p.velocity_rad_s)),hi=Math.max(lo+.001,...spark.map(p=>p.velocity_rad_s));$('spark-speed').innerHTML=`<polyline fill="none" stroke="#29ee91" stroke-width="1.5" points="${spark.map((p,i)=>`${i/(spark.length-1)*110},${26-(p.velocity_rad_s-lo)/(hi-lo)*23}`).join(' ')}"/>`;}
}

async function refreshProbes() {
  const probes=await request('probes');
  const previous=$('probe').value;
  $('probe').innerHTML='<option value="">'+(probes.length?'选择 SWD 调试器':'未检测到调试器')+'</option>'+probes.map(p=>`<option value="${esc(p.id)}">${esc(p.name)} · ${esc(p.id)}</option>`).join('');
  $('probe').value=probes.some(p=>p.id===previous)?previous:probes.length===1?probes[0].id:'';
  renderControls();
}

let polling=false;
async function poll() {
  if(polling||exited)return;
  polling=true;
  try {
    const response=await fetch('/api/status',{cache:'no-store'});
    if(!response.ok)throw new Error('服务不可用');
    data=await response.json();bridgeAvailable=true;lastStatusAt=performance.now();render();
  } catch(error) {bridgeAvailable=false;render();}
  finally {polling=false;}
}

document.querySelectorAll('.nav-item').forEach(el=>el.addEventListener('click',()=>switchView(el.dataset.view)));
document.querySelectorAll('[data-unit]').forEach(el=>el.addEventListener('click',()=>{const rad=targetRad();unit=el.dataset.unit;document.querySelectorAll('[data-unit]').forEach(b=>b.classList.toggle('selected',b===el));text('target-unit',unit==='rad'?'rad/s':'rpm');setTarget(rad);updateRange();}));
document.querySelectorAll('[data-speed]').forEach(el=>el.addEventListener('click',()=>setTarget(Number(el.dataset.speed))));
$('plus').onclick=()=>setTarget(targetRad()+(unit==='rpm'?Math.PI/30:.1));$('minus').onclick=()=>setTarget(targetRad()-(unit==='rpm'?Math.PI/30:.1));
$('speed-slider').oninput=()=>setTarget(Number($('speed-slider').value));
$('target').oninput=()=>{const v=targetRad();if(Number.isFinite(v))$('speed-slider').value=v;};
$('speed-limit').onchange=updateRange;$('probe').onchange=renderControls;
$('refresh').onclick=()=>action(refreshProbes);
$('exit').onclick=()=>action(async()=>{await request('shutdown');exited=true;bridgeAvailable=false;render();toast('上位机已退出，调试器已释放');});
$('connect').onclick=()=>action(async()=>{if(connected())await request('disconnect');else{saveDraft();await request('connect',{probe_id:$('probe').value,motor_params:edits.map(e=>({speed_limit_rad_s:e.cap,current_limit_a:e.current,accel_rad_s2:e.accel}))});samples=[];lastTick=null;toast('已连接，正在自动配置并零速使能双电机');}});
for(const [id,n] of Object.entries({run:3,stop:4,'clear-fault':6})) $(id).onclick=()=>action(()=>command(n));
let disabling=false;
async function disable(){if(!connected()||disabling)return;disabling=true;try{await command(5);}catch(error){toast(error.message,true);}finally{disabling=false;await poll();}}
$('disable').onclick=disable;
$('stop-all').onclick=stopAll;
document.querySelectorAll('[data-motor]').forEach(el=>el.onclick=()=>selectMotor(Number(el.dataset.motor)));
document.addEventListener('keydown',event=>{if(event.key==='Escape'){event.preventDefault();stopAll();}});
$('pause-chart').onclick=()=>{paused=!paused;const el=$('pause-chart');el.textContent=paused?'▶':'Ⅱ';el.title=paused?'继续曲线':'暂停曲线';el.setAttribute('aria-label',el.title);if(!paused)drawChart();};
for(const id of ['show-target','show-actual','chart-metric']) $(id).onchange=()=>{chartNeedsDraw=true;drawChart();};
$('export').onclick=async()=>{try{const r=await fetch('/api/export');if(!r.ok)throw new Error((await r.json()).error);const blob=await r.blob(),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='RobStride05_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);toast('真实采集数据已导出');}catch(error){toast(error.message,true);}};
$('copy-log').onclick=async()=>{try{await navigator.clipboard.writeText((data?.logs||[]).map(r=>`${r.time} [${r.kind}] ${r.message}`).join('\n'));toast('事件日志已复制');}catch{toast('浏览器未允许复制，请手动选择日志文本',true);}};
window.addEventListener('resize',()=>{chartNeedsDraw=true;drawChart();});
setInterval(()=>{text('clock',new Intl.DateTimeFormat('zh-CN',{hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false}).format(new Date()));if(performance.now()-lastStatusAt>800&&data?.connected){bridgeAvailable=false;render();}},500);
(async()=>{try{updateRange();sessionToken=(await(await fetch('/api/session')).json()).token;await poll();await action(refreshProbes);}catch(error){toast(error.message,true);}setInterval(poll,200);drawChart();})();

function renderDual() {
  const live=connected(),snap=data?.snapshot||{},dual=data?.motor_count===2;
  for(let i=0;i<2;i++) {
    const m=(i===0?snap.motor:snap.motor2)||{};
    const valid=live && data.sample_age_ms<600 && (i===0?snap.coherent:snap.coherent2);
    const fresh=valid && m.online && m.feedback_age_ms+data.sample_age_ms<150;
    text('summary-'+i,!live?'等待连接':!dual&&i===1?'当前固件仅支持单电机':`${fresh?'在线':'离线 / 过期'} · ${states[m.state]||'未知'} · 实测 ${fresh?format(m.velocity_rad_s):'—'} / 目标 ${valid?format(m.target_rad_s):'—'} rad/s`);
    $('select-motor-'+i).classList.toggle('selected',selectedMotor===i);
    $('select-motor-'+i).disabled=busy || (live && i===1 && !dual);
  }
  text('selected-label',`当前控制：电机 ${selectedMotor+1} · ID ${selectedMotor===0?127:126}。下方参数、命令和曲线均对应此电机。`);
  $('stop-all').disabled=!live;
}
function saveDraft() {
  edits[selectedMotor]={target:targetRad(),cap:Number($('speed-limit').value),current:Number($('current-limit').value),accel:Number($('accel-limit').value)};
}
function selectMotor(index) {
  if(index===selectedMotor || busy)return;
  saveDraft();
  selectedMotor=index;
  const e=edits[index];
  $('speed-limit').value=e.cap;$('current-limit').value=e.current;$('accel-limit').value=e.accel;
  setTarget(e.target);updateRange();samples=[];lastTick=null;chartNeedsDraw=true;render();
}
let allStopping=false;
async function stopAll() {
  window.forgetDrive?.();
  if(!connected()||allStopping)return;
  allStopping=true;
  try {await request('stop_all');toast('两台失能请求已提交，请查看两台反馈确认');}
  catch(error){toast(error.message,true);}
  finally {allStopping=false;await poll();}
}
