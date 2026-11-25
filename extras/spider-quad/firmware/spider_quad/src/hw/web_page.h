// Control page served at "/". Hold an arrow to walk: the page repeats the command every 300 ms
// and the robot stops by itself one second after the last one, so a dropped phone never leaves it walking.
#pragma once
#include <pgmspace.h>

static const char kPage[] PROGMEM = R"HTML(<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Паучок</title>
<style>
:root{color-scheme:dark}
body{margin:0;padding:16px;font:16px system-ui,sans-serif;background:#15171a;color:#e8e8e8;
  -webkit-user-select:none;user-select:none;touch-action:manipulation}
h2{font-size:15px;font-weight:600;color:#9aa3ad;margin:20px 0 8px}
button{font:inherit;color:inherit;background:#262a30;border:1px solid #3a4048;border-radius:10px;padding:12px}
button:active,button.on{background:#3d6ea8;border-color:#3d6ea8}
.pad{display:grid;grid-template-columns:repeat(3,80px);grid-template-rows:repeat(3,80px);gap:8px;justify-content:center}
.pad button{font-size:28px}
.row{display:flex;flex-wrap:wrap;gap:8px}
.row button{flex:1 1 30%}
input[type=range]{width:100%}
table{width:100%;border-collapse:collapse}
td{padding:4px 2px}
td.v{width:40px;text-align:center;font-variant-numeric:tabular-nums}
td button{padding:8px 12px}
details{margin-top:20px}
summary{color:#9aa3ad}
p.hint{color:#9aa3ad;font-size:14px;line-height:1.4}
#st{min-height:20px;color:#9aa3ad;font-size:14px;margin-top:12px}
</style></head><body>
<h2>Ходьба</h2>
<div class="pad">
  <span></span><button data-d="fwd">▲</button><span></span>
  <button data-d="left">⟲</button><button id="stop">■</button><button data-d="right">⟳</button>
  <span></span><button data-d="back">▼</button><span></span>
</div>
<h2>Позы</h2>
<div class="row">
  <button data-c="stand">Стоять</button><button data-c="sit">Сесть</button><button data-c="lie">Лечь</button>
  <button data-c="hello">Привет</button><button data-c="dance">Танец</button>
</div>
<h2>Скорость шага: <span id="cyc">1200</span> мс на цикл</h2>
<input id="speed" type="range" min="600" max="2000" step="100" value="1200">
<details><summary>Калибровка</summary>
<p class="hint">1. «Все в 90°» и надеть качалки как можно ровнее.<br>
2. «Стойка», затем подогнать каждую серву кнопками −/+: бёдра поперёк корпуса, лапы отвесно.<br>
3. Если сустав крутится не в ту сторону — «⇄».<br>4. «Сохранить».</p>
<div class="row"><button data-c="center">Все в 90°</button><button data-c="stand">Стойка</button></div>
<table id="cal"></table>
<div class="row" style="margin-top:8px"><button id="save">Сохранить</button></div>
</details>
<div id="st"></div>
<script>
const names=['ПЛ бедро','ПЛ колено','ПП бедро','ПП колено','ЗЛ бедро','ЗЛ колено','ЗП бедро','ЗП колено'];
let cal={trim:[0,0,0,0,0,0,0,0],dir:[1,1,1,1,1,1,1,1]};
const st=document.getElementById('st');
async function go(url){
  try{const r=await fetch(url);const t=await r.text();
    if(!r.ok){st.textContent='Ошибка: '+t;return null}
    st.textContent='';return t}
  catch(e){st.textContent='Нет связи с роботом';return null}
}
async function goCal(url){const t=await go(url);if(t){cal=JSON.parse(t);draw()}}
function draw(){
  const tb=document.getElementById('cal');tb.innerHTML='';
  names.forEach((n,j)=>{
    const tr=document.createElement('tr');
    tr.innerHTML='<td>'+n+'</td><td><button>−</button></td><td class="v">'+cal.trim[j]+
      '</td><td><button>+</button></td><td><button>⇄ '+(cal.dir[j]>0?'+':'−')+'</button></td>';
    const b=tr.querySelectorAll('button');
    b[0].onclick=()=>goCal('/trim?j='+j+'&v='+Math.max(-30,cal.trim[j]-1));
    b[1].onclick=()=>goCal('/trim?j='+j+'&v='+Math.min(30,cal.trim[j]+1));
    b[2].onclick=()=>goCal('/dir?j='+j);
    tb.appendChild(tr);
  });
  if(cal.cycle){document.getElementById('speed').value=cal.cycle;document.getElementById('cyc').textContent=cal.cycle}
}
let timer=null,held=null;
function walkStart(btn){
  walkEnd(false);held=btn;btn.classList.add('on');
  const send=()=>go('/cmd?c=walk&d='+btn.dataset.d);
  send();timer=setInterval(send,300);
}
function walkEnd(stop){
  if(timer){clearInterval(timer);timer=null}
  if(held){held.classList.remove('on');held=null;if(stop)go('/cmd?c=stop')}
}
document.querySelectorAll('[data-d]').forEach(b=>{
  b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);walkStart(b)});
  ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>walkEnd(true)));
});
document.getElementById('stop').onclick=()=>{walkEnd(false);go('/cmd?c=stop')};
document.querySelectorAll('[data-c]').forEach(b=>b.onclick=()=>go('/cmd?c='+b.dataset.c));
const sp=document.getElementById('speed');
sp.oninput=()=>document.getElementById('cyc').textContent=sp.value;
sp.onchange=()=>goCal('/speed?ms='+sp.value);
document.getElementById('save').onclick=async()=>{if(await go('/save'))st.textContent='Сохранено'};
document.addEventListener('visibilitychange',()=>{if(document.hidden)walkEnd(true)});
goCal('/cal');
</script></body></html>)HTML";
