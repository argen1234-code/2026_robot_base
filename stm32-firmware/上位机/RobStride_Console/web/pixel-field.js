// The same point field runs on an OffscreenCanvas worker, with a main-thread fallback.
(() => {
  class PixelField {
    constructor(canvas) { this.canvas=canvas; this.context=canvas.getContext('2d',{alpha:true}); }
    resize(width,height,dpr) {
      this.width=width;this.height=height;
      this.dpr=dpr;
      this.canvas.width=Math.round(width*dpr);this.canvas.height=Math.round(height*dpr);
      this.context?.setTransform(dpr,0,0,dpr,0,0);
    }
    paint(now,preferences,accent,reduced) {
      const {width,height,context}=this;
      if (!context || !width || !height) return;
      const time=reduced?0:now*.0003*preferences.speed;
      const rgb=/^#[a-f0-9]{6}$/i.test(accent)?[1,3,5].map(i=>parseInt(accent.slice(i,i+2),16)):[50,240,140];
      const cell=(Number(preferences.spacing)||7)*(width>1000?1:1.1);
      context.clearRect(0,0,width,height);
      for(let y=12;y<height;y+=cell) for(let x=3;x<width;x+=cell) {
        const u=x/width,v=y/height;
        const warp=.1*Math.sin(u*8+v*5+time*.3)+.06*Math.sin(v*14-u*7-time*.24);
        const ridge=.64+.17*Math.sin(u*6-.9+time*.35)+warp;
        const band=Math.exp(-(((v-ridge)/.19)**2));
        const left=Math.exp(-(((u+.03+.06*Math.sin(v*8+time*.2))/.32)**2)-((v-.38)/.43)**2);
        const right=.6*Math.exp(-(((u-1.02)/.16)**2)-((v-.08-.07*Math.sin(time*.3))/.18)**2);
        const grain=.62+.38*Math.sin(x*.019+y*.025+Math.sin(y*.013-time*.4)*2);
        const hollow=1-.9*Math.exp(-(((u-.64)/.29)**2)-((v-.28)/.24)**2);
        const amount=Math.max(band,left,right)*(.6+.4*grain)*hollow;
        const flicker=.65+.35*Math.sin(x*.021+y*.057+time*2.1);
        const seed=Math.sin(x*12.9898+y*78.233)*43758.5453,scatter=seed-Math.floor(seed);
        const opacity=Math.min(.82,Math.max(0,(amount-.16-scatter*.25)*1.3*flicker));
        if(opacity<.035)continue;
        const green=Math.sin(x*.014+y*.023+time*.4)>.35&&scatter>.35;
        context.fillStyle=green?`rgba(${rgb.map(c=>Math.round(c*.65)).join(',')},${opacity*.7})`:`rgba(183,190,186,${opacity*.8})`;
        context.fillRect(Math.round(x),Math.round(y),Math.max(2,cell-2),Math.max(2,cell-2));
      }
    }
  }
  globalThis.ControlDeskPixels=PixelField;
  if(typeof document!=='undefined')return;
  let field,state,timer=0;
  const draw=()=>{
    timer=0;
    if(!state?.preferences.enabled){field?.context?.clearRect(0,0,field.width,field.height);return;}
    if(!state.visible)return;
    field.paint(performance.now(),state.preferences,state.accent,state.reduced);
    if(!state.reduced)timer=setTimeout(draw,32);
  };
  onmessage=event=>{
    const next=event.data;
    if(next.canvas)field=new PixelField(next.canvas);
    if(!field || !next.preferences)return;
    if(field.width!==next.width||field.height!==next.height||field.dpr!==next.dpr)field.resize(next.width,next.height,next.dpr);
    state=next;
    clearTimeout(timer);timer=setTimeout(draw,0);
  };
})();
