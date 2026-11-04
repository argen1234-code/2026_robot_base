'use strict';
const ledNames=['自定义循环','流动彩虹','双向彗星','极光波浪','火焰','流动等离子','往返扫描','纯色常亮'];
const ledDefaults={enabled:1,mode:0,brightness:64,speed_percent:100,color_order:0,red:32,green:180,blue:255,cycle_interval_ms:8000,cycle_modes:[1,2,3,4,5,6]};
let ledPlaylist=[...ledDefaults.cycle_modes],ledLoaded=false;
$('log-section').insertAdjacentHTML('beforebegin',`
<section id="view-led" class="panel led-panel view hidden">
 <div class="panel-heading"><div><span class="section-index">04 /</span><h2>WS2812 灯带控制</h2></div><span id="led-status" class="badge">等待连接</span></div>
 <div class="led-live"><div><span>板上当前显示</span><strong id="led-live-mode">—</strong><p id="led-live-detail">连接设备后读取灯带状态</p></div><div><span>应用状态</span><strong id="led-sync">尚未读取</strong><p id="led-errors">—</p></div></div>
 <div class="led-grid"><div class="led-settings">
  <h3>当前灯效与外观</h3><p class="led-hint">编辑后点击“应用到灯带”。灯带控制无需准备或使能电机。</p>
  <label class="led-check"><input id="led-enabled" type="checkbox" checked> 开启灯带</label>
  <label>显示方式<select id="led-mode">${ledNames.map((n,i)=>`<option value="${i}">${n}</option>`).join('')}</select></label>
  <div class="led-pair"><label>亮度 · 0–255<input id="led-brightness" type="number" min="0" max="255" step="1" value="64"></label><label>动画速度 · %<input id="led-speed" type="number" min="1" max="400" step="1" value="100"></label></div>
  <label>自定义颜色<input id="led-color" type="color" value="#20b4ff"></label>
  <div class="led-rgb">${['red','green','blue'].map((n,i)=>`<label>${['R 红','G 绿','B 蓝'][i]}<input id="led-${n}" type="number" min="0" max="255" step="1" value="${[32,180,255][i]}"></label>`).join('')}</div>
  <p class="led-hint">自定义颜色作用于“往返扫描”和“纯色常亮”；其它灯效使用自身配色。动画速度与循环切换间隔独立。</p>
  <details><summary>高级设置</summary><label>灯带颜色顺序<select id="led-order"><option value="0">GRB · 默认 WS2812</option><option value="1">RGB</option></select></label></details>
 </div><div class="led-cycle">
  <h3>自定义循环列表</h3><p class="led-hint">按列表从上到下循环，可重复添加；最多 8 项。显示方式选择“自定义循环”后生效。</p>
  <label>每项播放时长 · 秒<input id="led-interval" type="number" min="1" max="600" step="0.1" value="8"></label>
  <ol id="led-playlist" class="led-playlist"></ol>
  <div class="led-add"><select id="led-add-mode" aria-label="要添加的灯效">${ledNames.slice(1).map((n,i)=>`<option value="${i+1}">${n}</option>`).join('')}</select><button id="led-add" class="button secondary">添加灯效 ＋</button></div>
  <div class="led-presets"><button id="led-all" class="button secondary">循环全部 7 种</button><button id="led-default" class="button secondary">恢复默认编辑值</button></div>
  <p class="led-hint">循环由开发板运行，关闭页面或断开上位机后继续。板上设置在 MCU 重启后恢复默认；可保存到本机，重新读取后点击应用。</p>
 </div></div>
 <div class="led-actions"><button id="led-apply" class="button primary" disabled>应用到灯带 ↗</button><button id="led-off" class="button secondary" disabled>关闭灯带</button><button id="led-read" class="button secondary" disabled>读取板上设置</button><button id="led-save" class="button secondary">保存到本机</button><button id="led-load" class="button secondary">读取本机方案</button></div>
</section>`);

function ledBoardConfig() {
  const l=data?.snapshot?.led;
  if(!l || !Array.isArray(l.cycle_modes))return null;
  return Object.fromEntries(Object.keys(ledDefaults).map(k=>[k,k==='cycle_modes'?l.cycle_modes.slice(0,l.cycle_count):l[k]]));
}
function ledDraft() {
  const c={enabled:$('led-enabled').checked?1:0,mode:Number($('led-mode').value),
    brightness:Number($('led-brightness').value),speed_percent:Number($('led-speed').value),
    color_order:Number($('led-order').value),red:Number($('led-red').value),green:Number($('led-green').value),blue:Number($('led-blue').value),
    cycle_interval_ms:Math.round(Number($('led-interval').value)*1000),cycle_modes:[...ledPlaylist]};
  for(const id of ['led-brightness','led-speed','led-red','led-green','led-blue','led-interval'])
    if(!$(id).checkValidity() || $(id).value==='')throw new Error('请填写范围内的灯效参数');
  if(!c.cycle_modes.length)throw new Error('请至少添加一项循环灯效');
  return c;
}
function ledFill(c) {
  $('led-enabled').checked=!!c.enabled;$('led-mode').value=c.mode;
  for(const [id,k] of [['led-brightness','brightness'],['led-speed','speed_percent'],['led-order','color_order'],['led-red','red'],['led-green','green'],['led-blue','blue']])$(id).value=c[k];
  $('led-interval').value=c.cycle_interval_ms/1000;ledPlaylist=[...c.cycle_modes];ledUpdateColor();ledRenderPlaylist();
}
function ledUpdateColor() {
  $('led-color').value='#'+['red','green','blue'].map(k=>Math.max(0,Math.min(255,Number($('led-'+k).value)||0)).toString(16).padStart(2,'0')).join('');
}
function ledRenderPlaylist() {
  $('led-playlist').innerHTML=ledPlaylist.map((mode,i)=>`<li><span>${i+1}. ${esc(ledNames[mode])}</span><button data-at="${i}" data-op="up" aria-label="上移第 ${i+1} 项" ${i===0?'disabled':''}>↑</button><button data-at="${i}" data-op="down" aria-label="下移第 ${i+1} 项" ${i===ledPlaylist.length-1?'disabled':''}>↓</button><button data-at="${i}" data-op="remove" aria-label="移除第 ${i+1} 项" ${ledPlaylist.length===1?'disabled':''}>×</button></li>`).join('');
  $('led-add').disabled=ledPlaylist.length>=8;
}
function renderLed() {
  const live=connected(),supported=data?.led_supported,l=data?.snapshot?.led;
  const usable=live&&supported&&!!l;
  if(usable&&!ledLoaded){ledFill(ledBoardConfig());ledLoaded=true;}
  for(const id of ['led-apply','led-off','led-read'])$(id).disabled=!usable||busy;
  badge('led-status',!live?'等待连接':!supported?'请下载新版灯效固件':!l?.ready?'灯带发送异常':`${l.led_count} 颗 · 已连接`,usable&&l.ready?'online':'');
  text('led-live-mode',usable?l.enabled?ledNames[l.active_mode]||'等待发送':'已关闭':'—');
  text('led-live-detail',usable?`${l.mode===0?`循环第 ${l.cycle_index+1}/${l.cycle_count} 项 · `:'固定灯效 · '}亮度 ${l.brightness}/255 · 动画 ${l.speed_percent}% · RGB ${l.red}, ${l.green}, ${l.blue}`:'连接设备后读取灯带状态');
  text('led-errors',usable?`已发送 ${l.frames_done} 帧 · 错误 ${l.errors} · 最近错误码 ${l.last_error}`:'—');
  let same=false;try {const draft=ledDraft(),board=ledBoardConfig();same=!!board&&Object.keys(draft).every(k=>JSON.stringify(draft[k])===JSON.stringify(board[k]));}catch{}
  text('led-sync',!usable?'等待连接':same?'与板上设置一致':'编辑值尚未应用');
}
$('view-led').addEventListener('input',renderLed);
$('led-color').oninput=()=>{const h=$('led-color').value;['red','green','blue'].forEach((k,i)=>$('led-'+k).value=parseInt(h.slice(1+i*2,3+i*2),16));renderLed();};
for(const k of ['red','green','blue'])$('led-'+k).oninput=()=>{ledUpdateColor();renderLed();};
$('led-playlist').onclick=event=>{const b=event.target.closest('button');if(!b)return;const i=Number(b.dataset.at),op=b.dataset.op;if(op==='remove')ledPlaylist.splice(i,1);else{const j=i+(op==='up'?-1:1);[ledPlaylist[i],ledPlaylist[j]]=[ledPlaylist[j],ledPlaylist[i]];}ledRenderPlaylist();renderLed();};
$('led-add').onclick=()=>{if(ledPlaylist.length<8)ledPlaylist.push(Number($('led-add-mode').value));ledRenderPlaylist();renderLed();};
$('led-all').onclick=()=>{ledPlaylist=[1,2,3,4,5,6,7];$('led-mode').value=0;ledRenderPlaylist();renderLed();};
$('led-default').onclick=()=>{ledFill(ledDefaults);renderLed();};
$('led-read').onclick=()=>{ledFill(ledBoardConfig());renderLed();toast('已读取板上设置到编辑区');};
$('led-apply').onclick=()=>action(async()=>{const result=await request('led',ledDraft());toast(`灯效已获固件确认 · 序号 ${result.seq}`);});
$('led-off').onclick=()=>action(async()=>{const c=ledBoardConfig();c.enabled=0;await request('led',c);ledFill(c);toast('灯带已关闭，重新勾选开启并应用即可恢复');});
$('led-save').onclick=()=>{try{localStorage.setItem('robstride.led.v1',JSON.stringify(ledDraft()));toast('灯效方案已保存到本机浏览器');}catch(e){toast(e.message,true);}};
$('led-load').onclick=()=>{try{const c=JSON.parse(localStorage.getItem('robstride.led.v1'));if(!c || Object.keys(ledDefaults).some(k=>!(k in c)) || !Array.isArray(c.cycle_modes) || !c.cycle_modes.length || c.cycle_modes.length>8 || c.cycle_modes.some(m=>!Number.isInteger(m)||m<1||m>7))throw new Error('没有有效的本机灯效方案');ledFill(c);ledDraft();renderLed();toast('已读取本机方案，点击应用后生效');}catch(e){toast(e.message,true);}};
ledRenderPlaylist();renderLed();
