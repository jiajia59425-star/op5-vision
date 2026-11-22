/* OP5_8090_FUSION_TOGGLE_FINAL_V1
 * Reuses the existing 8090 tracking slide and stop button.
 * Fusion ON routes slide -> 8101 ARM; the 8090 slider handles V2 when OFF.
 * Never auto-arms on page open or toggle; preserves 8101 fail-closed gate.
 */
(function(){
'use strict';
const BASE='/op5-v3-toggle/';
let mode=false, token='', fusionSession=false, busy=false, lastStatus=null, lastPreflight=null, polling=false;
const $=id=>document.getElementById(id);
function txt(s){const el=$('op5-v3-mini-status');if(el)el.textContent=s;}
function info(s, bad=false){txt(s);const el=$('op5-v3-mini-status');if(el)el.classList.toggle('bad',bad);}
async function req(route,body,auth=false){
  const headers={Accept:'application/json'};
  const opt={method:body===undefined?'GET':'POST',cache:'no-store',credentials:'same-origin',headers};
  if(body!==undefined){headers['Content-Type']='application/json';opt.body=JSON.stringify(body);}
  if(auth && token)headers['X-Operator-Token']=token;
  const response=await fetch(BASE+route,opt);
  let data;
  try{data=await response.json();}catch(_){throw Error('V3接口响应不是JSON');}
  if(!response.ok||data.ok===false)throw Error(data.error||data.reason||('HTTP '+response.status));
  return data;
}
function refreshSwitch(){
  const b=$('op5-v3-switch'), label=$('op5-v3-switch-mode');
  if(!b)return;
  b.setAttribute('aria-checked',String(mode));b.classList.toggle('on',mode);
  b.classList.toggle('busy',busy);b.disabled=busy;
  if(label)label.textContent=mode?'已加入 · V3雷达监护':'未加入 · V2纯视觉';
  const hint=$('op5-v3-why');if(hint)hint.textContent=mode?'使用上方原跟踪滑块启动；8101融合许可不通过时不会启动云台。':'使用上方原跟踪滑块；原V2预测跟踪模式保持不变。';
  const heading=document.querySelector('#op5-8090-follow .op5-slide-label');
  if(heading)heading.textContent=mode?'V3雷达＋视觉融合跟踪 · 在RGB画面点击目标后启动':'云台视觉跟踪 · 在RGB画面点击目标后启动';
  const slide=document.querySelector('#op5-8090-slide .op5-slide-text');
  if(slide && !slide.closest('.done'))slide.textContent=mode?'滑动到最右端，开启V3融合跟踪':'滑动到最右端，开启云台跟踪';
}
async function inspectActive(){
  const [v2, v3]=await Promise.all([
    fetch('/op5-slide/status',{cache:'no-store'}).then(r=>r.json()),
    req('status')
  ]);
  lastStatus=v3;
  return Boolean(v2.active||v3.armed||v3.stop_pending||fusionSession);
}
async function chooseMode(){
  if(busy)return;busy=true;refreshSwitch();
  try{
    if(await inspectActive())throw Error('请先点击“停止跟踪”，再切换融合模式');
    mode=!mode;
    if(mode){
      const h=await req('health');
      if(h.version!=='V3_STAGE1_FUSED_CAMERA_GUARD')throw Error('8101版本不匹配');
      info('融合已选中 · 等待目标与雷达就绪');
    }else info('已切换V2纯视觉模式');
  }catch(e){
    // On failed enable, revert to V2. Never change mode while another session is running.
    if(mode && (!lastStatus||!lastStatus.armed))mode=false;
    info('切换失败：'+e.message,true);
  }finally{busy=false;refreshSwitch();}
}
function showPinModal(){
  return new Promise((resolve,reject)=>{
    let closed=false;
    const outer=document.createElement('div');outer.id='op5-v3-pin-modal';
    outer.innerHTML=`<div class="op5-v3-dialog" role="dialog" aria-modal="true" aria-label="V3雷达融合授权">
      <div class="op5-v3-dialog-head">V3融合授权 <button type="button" class="op5-v3-close">✕</button></div>
      <div class="op5-v3-prompt">首次启动输入8101操作PIN（仅本次网页会话保留授权）</div>
      <input class="op5-v3-pin" type="password" autocomplete="off" spellcheck="false" placeholder="操作PIN" aria-label="操作PIN" maxlength="128" />
      <div class="op5-v3-keys" aria-label="触摸字母数字键盘"></div>
      <div class="op5-v3-actions"><button type="button" class="op5-v3-cancel">取消</button><button type="button" class="op5-v3-login">授权并启动</button></div>
      <div class="op5-v3-pin-error" role="status"></div>
    </div>`;
    document.body.appendChild(outer);
    const inp=outer.querySelector('input'),grid=outer.querySelector('.op5-v3-keys');
    let upper=false;
    function finish(ok,v){if(closed)return;closed=true;outer.remove();if(ok)resolve(v);else reject(Error(v||'取消融合授权'));}
    for(const k of [...'1234567890abcdefghijklmnopqrstuvwxyz-_','⇧','⌫','清空']){
      const key=document.createElement('button');key.type='button';key.textContent=k;
      key.addEventListener('click',()=>{
        if(k==='⇧'){upper=!upper;grid.querySelectorAll('[data-letter]').forEach(el=>{el.textContent=upper?el.dataset.letter.toUpperCase():el.dataset.letter;});return;}
        if(k==='⌫')inp.value=inp.value.slice(0,-1);
        else if(k==='清空')inp.value='';
        else if(inp.value.length<128)inp.value+=upper?k.toUpperCase():k;
      });
      if(/^[a-z]$/.test(k))key.dataset.letter=k;
      grid.appendChild(key);
    }
    outer.querySelector('.op5-v3-close').onclick=()=>finish(false,'已取消');
    outer.querySelector('.op5-v3-cancel').onclick=()=>finish(false,'已取消');
    outer.querySelector('.op5-v3-login').onclick=async()=>{
      const pin=inp.value;inp.value='';
      if(!pin){outer.querySelector('.op5-v3-pin-error').textContent='请输入PIN';return;}
      const submit=outer.querySelector('.op5-v3-login');submit.disabled=true;
      try{const result=await req('login',{pin});finish(true,result.token);}
      catch(e){outer.querySelector('.op5-v3-pin-error').textContent=e.message;submit.disabled=false;}
    };
  });
}
async function arm(){
  if(!mode)throw Error('雷达融合开关未开启');
  if(busy||fusionSession)throw Error('融合启动忙或已启动');
  if(window.__op5HudV11Source==='thermal')throw Error('当前融合监护只支持RGB目标，请切换到RGB');
  busy=true;refreshSwitch();
  try{
    const p=await req('preflight');
    lastPreflight=p;
    if(p.ready!==true)throw Error('雷达融合未就绪：'+(p.reason||'检查目标及雷达'));
    if(!token)token=await showPinModal();
    // Server repeats all checks immediately before commanding V2.
    await req('arm',{},true);
    fusionSession=true;info('● V3雷达＋视觉融合跟踪中');
    return {ok:true};
  }catch(e){
    if(/PIN|授权|session|令牌/i.test(e.message))token='';
    info('融合启动未获许可：'+e.message,true);
    throw e;
  }finally{busy=false;refreshSwitch();}
}
async function heartbeat(){
  if(!fusionSession)throw Error('融合会话未启动');
  const s=await req('status');lastStatus=s;
  if(!s.armed||s.stop_pending){fusionSession=false;throw Error('8101已停止或正在停机：'+(s.reason||''));}
  await req('heartbeat',{},true);
}
async function stop(){
  // Even when the bridge has expired, a stop must always reach 8101.
  try{await req('stop',{});info('已请求停止V3融合跟踪');}
  finally{fusionSession=false;}
}
async function poll(){
  if(polling)return;polling=true;
  try{
    const [p,s]=await Promise.all([req('preflight'),req('status')]);
    lastPreflight=p;lastStatus=s;
    if(mode){
      const distance=(typeof p.distance_m==='number')?' · '+p.distance_m.toFixed(2)+'m':'';
      info(s.armed?'● V3融合跟踪中'+distance:(p.ready?'● 融合就绪'+distance:'融合等待：'+(p.reason||'目标/雷达未就绪')),false);
      if(s.stop_pending)info('8101正在确认停止',true);
    }
    // The 8090 original slider keepalive owns heartbeat while fusionSession.
  }catch(e){if(mode)info('8101状态读取失败：'+e.message,true);}
  finally{polling=false;}
}
function mount(){
  if($('op5-v3-switch'))return;
  const slide=$('op5-center-v2-slide'), infoNode=$('op5-center-v2-message');
  const root=$('op5-8090-follow');
  if(!root || !slide){setTimeout(mount,300);return;}
  const ui=document.createElement('div');ui.id='op5-v3-fusion-row';
  ui.innerHTML=`<div class="op5-v3-top"><div class="op5-v3-left"><span class="op5-v3-title">是否加入雷达融合</span><span class="op5-v3-mode" id="op5-v3-switch-mode">未加入 · V2纯视觉</span></div>
    <button type="button" id="op5-v3-switch" role="switch" aria-label="加入雷达融合" aria-checked="false"><span class="op5-v3-knob"></span></button></div>
    <div id="op5-v3-mini-status" class="op5-v3-status" role="status">默认纯视觉；打开开关再滑动原跟踪条</div>
    <div id="op5-v3-why" class="op5-v3-small">开机只启动服务，不会自动转动云台。</div>`;
  (infoNode||slide).insertAdjacentElement('afterend',ui);
  $('op5-v3-switch').addEventListener('click',chooseMode);
  refreshSwitch();poll();setInterval(poll,1100);
}
window.OP5FusionMode={enabled:()=>mode,isFusedSession:()=>fusionSession,arm,heartbeat,stop};
window.addEventListener('pagehide',()=>{
  if(fusionSession){
    try{fetch(BASE+'stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',credentials:'same-origin',keepalive:true});}catch(_){ }
  }
});
if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',mount,{once:true});else mount();
})();
