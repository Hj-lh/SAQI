"""Runtime settings routes + a tiny self-contained settings UI.

GET  /settings        -> JSON: sections, fields, current + default values
POST /settings        -> JSON body {KEY: value, ...}; persists editable values
POST /settings/reset  -> clears all overrides (back to config.py defaults)
GET  /settings/ui     -> a plain HTML page to view/edit everything
"""

from fastapi import APIRouter, Body
from fastapi.responses import HTMLResponse

from components import settings

router = APIRouter(prefix="/settings", tags=["settings"])


@router.get("")
def get_settings():
    return {"sections": settings.schema()}


@router.post("")
def update_settings(values: dict = Body(...)):
    return {"status": "ok", "values": settings.update(values)}


@router.post("/reset")
def reset_settings():
    return {"status": "ok", "values": settings.reset()}


@router.get("/ui", response_class=HTMLResponse)
def settings_ui():
    return _PAGE


_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SAQI Settings</title>
<style>
  :root { color-scheme: light dark; }
  * { box-sizing: border-box; }
  body { font-family: system-ui, sans-serif; margin: 0; padding: 1rem;
         max-width: 820px; margin-inline: auto; line-height: 1.4; }
  h1 { font-size: 1.4rem; margin: .2rem 0; }
  .bar { position: sticky; top: 0; background: Canvas; padding: .6rem 0;
         display: flex; gap: .5rem; align-items: center; border-bottom: 1px solid #8884; z-index: 5; }
  .bar .grow { flex: 1; }
  button { font: inherit; padding: .5rem .9rem; border-radius: .5rem;
           border: 1px solid #8886; cursor: pointer; background: #6b8afd; color: #fff; }
  button.secondary { background: transparent; color: inherit; }
  details { border: 1px solid #8884; border-radius: .6rem; margin: .8rem 0; padding: 0 .9rem; }
  summary { font-weight: 600; cursor: pointer; padding: .7rem 0; }
  .sdesc { font-size: .85rem; opacity: .75; margin: 0 0 .6rem; }
  .field { padding: .55rem 0; border-top: 1px solid #8882; display: grid;
           grid-template-columns: 1fr 9rem; gap: .3rem .8rem; align-items: center; }
  .field:first-of-type { border-top: none; }
  .field label { font-weight: 500; }
  .field .help { grid-column: 1 / -1; font-size: .8rem; opacity: .7; margin: 0; }
  .field input, .field select { font: inherit; padding: .4rem .5rem; border-radius: .4rem;
                 border: 1px solid #8886; width: 100%; background: Field; color: FieldText; }
  .field input:disabled, .field select:disabled { opacity: .6; }
  .field .def { grid-column: 1 / -1; font-size: .72rem; opacity: .55; }
  .ro { font-size: .72rem; opacity: .6; font-weight: 400; }
  #msg { font-size: .9rem; }
  .ok { color: #2e9e44; } .err { color: #d23; }
</style>
</head>
<body>
  <div class="bar">
    <h1>SAQI Settings</h1>
    <span class="grow"></span>
    <span id="msg"></span>
    <button class="secondary" onclick="restore()">Restore defaults</button>
    <button onclick="save()">Save</button>
  </div>
  <p class="sdesc">Navigation values apply the next time you press <b>Start</b>
  on an auto run. Buzzer settings apply immediately. The hardware section is
  read-only.</p>
  <div id="form"></div>

<script>
let SECTIONS = [];

function esc(s){ return String(s).replace(/[&<>"]/g, c =>
  ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])); }

async function load(){
  const r = await fetch('/settings');
  const data = await r.json();
  SECTIONS = data.sections;
  render();
}

function render(){
  const root = document.getElementById('form');
  root.innerHTML = '';
  SECTIONS.forEach((sec, si) => {
    const d = document.createElement('details');
    if (si < 3) d.open = true;
    d.innerHTML = `<summary>${esc(sec.title)}</summary>
      <p class="sdesc">${esc(sec.desc)}</p>`;
    sec.fields.forEach(f => {
      const ro = f.scope === 'readonly';
      const div = document.createElement('div');
      div.className = 'field';
      const inputType = (f.type === 'int' || f.type === 'float') ? 'number'
                        : (f.type === 'bool' ? 'checkbox' : 'text');
      const step = f.type === 'int' ? '1' : (f.step || 'any');
      const checked = (f.type === 'bool' && f.value) ? 'checked' : '';
      const val = f.type === 'bool' ? '' : `value="${esc(f.value)}"`;
      const rng = (f.min!==undefined?`min="${f.min}" `:'') + (f.max!==undefined?`max="${f.max}" `:'');
      const control = f.options
        ? `<select id="f_${f.key}" data-key="${f.key}" data-type="${f.type}" ${ro?'disabled':''}>
             ${f.options.map(o => `<option value="${esc(o.value)}" ${o.value===f.value?'selected':''}>${esc(o.label)}</option>`).join('')}
           </select>`
        : `<input id="f_${f.key}" data-key="${f.key}" data-type="${f.type}"
               type="${inputType}" step="${step}" ${rng} ${val} ${checked} ${ro?'disabled':''}>`;
      div.innerHTML = `
        <label for="f_${f.key}">${esc(f.label)} ${ro?'<span class="ro">(read-only)</span>':''}</label>
        ${control}
        <p class="help">${esc(f.help)}</p>
        <p class="def">default: ${esc(f.default)}</p>`;
      d.appendChild(div);
    });
    root.appendChild(d);
  });
}

function collect(){
  const out = {};
  document.querySelectorAll('#form input:not(:disabled), #form select:not(:disabled)').forEach(inp => {
    const t = inp.dataset.type;
    out[inp.dataset.key] = (t === 'bool') ? inp.checked
                          : (t === 'int' || t === 'float') ? Number(inp.value)
                          : inp.value;
  });
  return out;
}

function msg(text, cls){
  const m = document.getElementById('msg');
  m.textContent = text; m.className = cls || '';
  if (text) setTimeout(() => { if (m.textContent === text) m.textContent=''; }, 3000);
}

async function save(){
  try {
    const r = await fetch('/settings', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify(collect())});
    if(!r.ok) throw new Error(await r.text());
    await load();
    msg('Saved ✓ applies on next Start', 'ok');
  } catch(e){ msg('Save failed: ' + e.message, 'err'); }
}

async function restore(){
  if(!confirm('Restore all settings to their defaults?')) return;
  try {
    const r = await fetch('/settings/reset', {method:'POST'});
    if(!r.ok) throw new Error(await r.text());
    await load();
    msg('Restored defaults ✓', 'ok');
  } catch(e){ msg('Reset failed: ' + e.message, 'err'); }
}

load();
</script>
</body>
</html>"""
