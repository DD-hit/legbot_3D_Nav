const pad=document.querySelector('#pad'), knob=document.querySelector('#knob'), status=document.querySelector('output');
let active, owner='', heartbeat;
const connect=document.querySelector('#connect'), disconnect=document.querySelector('#release');
function refresh(){
  connect.disabled=!!owner; disconnect.disabled=!owner;
  for(const b of document.querySelectorAll('[data-turn],[data-pose]'))b.disabled=!owner;
  pad.style.opacity=owner?'1':'.45';
}
function drop(){
  if(!owner)return;
  release();
  send({action:'release'},true).catch(()=>{});
  owner=''; clearInterval(heartbeat); refresh(); status.textContent='已释放本页控制权';
}
connect.onclick=async()=>{
  connect.disabled=true;
  try{
    owner=await send({action:'connect'});
    status.textContent='已取得手动控制权';
    heartbeat=setInterval(async()=>{const current=owner;try{await send({action:'heartbeat'})}catch(e){if(current===owner){drop();status.textContent=e.message}}},1300);
  }catch(e){status.textContent=e.message}finally{refresh()}
};
disconnect.onclick=drop;
for(const b of document.querySelectorAll('[data-pose]'))b.onclick=async()=>{
  if(!owner || !confirm('确认执行'+b.textContent+'？'))return;
  try{status.textContent=await send({action:b.dataset.pose})}catch(e){status.textContent=e.message}
};
window.addEventListener('pagehide',drop);
refresh();
async function send(body,keepalive=false){
  const r=await fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...body,owner}),keepalive,signal:AbortSignal.timeout(1200)});
  const data=await r.json();
  if(!r.ok) throw Error(data.error || '控制连接中断');
  return data.message;
}
function release(){
  const g=active;
  if(!g)return;
  active=null;
  clearTimeout(g.timer);
  knob.style.transform='';
  status.textContent='已松开';
  if(g.token)send({action:'end',token:g.token},true).catch(()=>{});
}
function position(e){
  if(!active || active.id!==e.pointerId || active.target!==pad)return;
  const r=pad.getBoundingClientRect(), radius=r.width*.34;
  let x=(r.top+r.height/2-e.clientY)/radius,y=(r.left+r.width/2-e.clientX)/radius;
  const norm=Math.max(1,Math.hypot(x,y)); x/=norm; y/=norm;
  if(Math.hypot(x,y)<.12)x=y=0;
  Object.assign(active,{x,y});
  knob.style.transform='translate('+(-y*radius)+'px,'+(-x*radius)+'px)';
}
// The bridge scales normalized translation by 0.12 m/s. D1 rejects
// nonzero components below 0.05 m/s forward or 0.10 m/s sideways.
function driveVector(x,y){
  const magnitude=Math.min(1,Math.hypot(x,y));
  if(magnitude<.12)return {x:0,y:0};
  const forward=Math.abs(x)>=.38*magnitude;
  const sideways=Math.abs(y)>=.38*magnitude;
  const fraction=(magnitude-.12)/.88;
  const speed=(min,max)=>min+(max-min)*fraction;
  return {
    x:forward?Math.sign(x)*speed(.051,sideways?.058:.12)/.12:0,
    y:sideways?Math.sign(y)*speed(.101,forward?.104:.12)/.12:0
  };
}
async function pump(g){
  if(active!==g)return;
  try {
    await send({action:'drive',token:g.token,seq:++g.seq,...driveVector(g.x,g.y),turn:g.turn});
    if(active===g)g.timer=setTimeout(()=>pump(g),70);
  } catch(e){if(active===g){release();status.textContent=e.message}}
}
for(const target of [pad,...document.querySelectorAll('[data-turn]')]){
  target.onpointerdown=async e=>{
    if(!owner){status.textContent='请先点击「连接控制权」';return}
    if(active || e.button!==0)return;
    e.preventDefault();
    const g=active={id:e.pointerId,target,turn:Number(target.dataset.turn||0),x:0,y:0,seq:0};
    target.setPointerCapture(e.pointerId);
    position(e);
    try{
      g.token=await send({action:'begin'});
      if(active!==g){await send({action:'end',token:g.token});return}
      status.textContent='手动控制中';await pump(g);
    }catch(err){if(active===g){release();status.textContent=err.message}}
  };
  target.onpointermove=position;
  for(const event of ['pointerup','pointercancel','lostpointercapture'])
    target.addEventListener(event,e=>{if(active?.id===e.pointerId)release()});
}
window.addEventListener('blur',release);
window.addEventListener('pagehide',release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});
