import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { api, fmtClock, FrameStream, Telemetry, type AgentInfo, type EntityDetail, type SimEvent, type SimStatus, type TaskOverlay, type WsMessage } from "./api";
import { rpyToQuat, WorldScene, type Overlays } from "./scene";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;
const $$ = <T extends HTMLElement>(sel: string) => [...document.querySelectorAll<T>(sel)];

// ---- renderer ----------------------------------------------------------------
const viewport = $("#viewport");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.05;
viewport.appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(52, 1, 0.1, 3000);
camera.up.set(0, 0, 1);
camera.position.set(-60, -140, 70);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, -50, 5); controls.enableDamping = true; controls.maxPolarAngle = Math.PI / 2 - 0.02;
const world = new WorldScene();

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false); renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport); resize();

// ---- state -------------------------------------------------------------------
type CamMode = "orbit" | "follow" | "top" | "fpv";
type ViewMode = "operations" | "development" | "replay";
let selected: string | null = null;
let camMode: CamMode = "orbit";
let viewMode: ViewMode = "operations";
let status: SimStatus | null = null;
let agents = new Map<string, AgentInfo>();
let lastEventSeq = 0;
let streams: FrameStream[] = [];
let overlay: TaskOverlay | null = null;
let framedOnce = false;
let fpvCam: string | null = null;

function toast(msg: string, info = false) {
  const t = $("#toast"); t.textContent = msg; t.className = info ? "info" : ""; t.style.display = "block";
  setTimeout(() => (t.style.display = "none"), 3500);
}

// ---- smooth camera transitions ------------------------------------------------
let tween: { p0: THREE.Vector3; p1: THREE.Vector3; t0: THREE.Vector3; t1: THREE.Vector3; start: number; dur: number } | null = null;
function flyTo(pos: THREE.Vector3, target: THREE.Vector3, dur = 700) {
  tween = { p0: camera.position.clone(), p1: pos.clone(), t0: controls.target.clone(), t1: target.clone(), start: performance.now(), dur };
}
function frameAll() {
  const box = world.focusBox(); if (!box) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = Math.max(box.getSize(new THREE.Vector3()).length(), 8);
  const dist = size * 1.3 + 6;
  flyTo(new THREE.Vector3(center.x - dist * 0.55, center.y - dist * 0.65, center.z + dist * 0.45), center);
}
function frameEntity(id: string) {
  const g = world.models.get(id); if (!g) return;
  const box = new THREE.Box3().setFromObject(g); const size = Math.max(box.getSize(new THREE.Vector3()).length(), 3);
  const c = box.getCenter(new THREE.Vector3());
  flyTo(new THREE.Vector3(c.x - size * 1.2, c.y - size * 1.4, c.z + size * 0.9), c, 600);
}

// ---- hierarchy ----------------------------------------------------------------
const CATEGORY_ORDER = ["drone", "vehicle", "target", "dynamic", "building", "infrastructure", "obstacle", "terrain"];
const CATEGORY_LABEL: Record<string, string> = { drone: "Drones", vehicle: "Vehicles", target: "Targets", dynamic: "Dynamic objects", building: "Buildings", infrastructure: "Infrastructure", obstacle: "Obstacles", terrain: "Terrain" };
const openGroups = new Set(["drone", "vehicle", "target", "dynamic"]);
function renderHierarchy() {
  const filter = ($("#filter") as HTMLInputElement).value.trim().toLowerCase();
  const groups = new Map<string, string[]>();
  for (const m of world.meta.values()) {
    if (filter && !m.entity_id.toLowerCase().includes(filter)) continue;
    const c = CATEGORY_ORDER.includes(m.category) ? m.category : "obstacle";
    (groups.get(c) ?? groups.set(c, []).get(c)!).push(m.entity_id);
  }
  const root = $("#hierarchy"); root.innerHTML = "";
  let total = 0;
  for (const cat of CATEGORY_ORDER) {
    const ids = groups.get(cat); if (!ids) continue;
    ids.sort(); total += ids.length;
    const det = document.createElement("details"); det.open = openGroups.has(cat) || !!filter;
    det.addEventListener("toggle", () => det.open ? openGroups.add(cat) : openGroups.delete(cat));
    det.innerHTML = `<summary>${CATEGORY_LABEL[cat]}<span class="badge">${ids.length}</span></summary>`;
    const ul = document.createElement("ul");
    for (const id of ids) {
      const li = document.createElement("li"); li.className = cat + (id === selected ? " sel" : "");
      li.innerHTML = `<span class="sw"></span><span>${id}</span><span class="kind">${world.meta.get(id)?.kind ?? ""}</span>`;
      li.onclick = () => { select(id); if (camMode === "orbit") frameEntity(id); };
      ul.appendChild(li);
    }
    det.appendChild(ul); root.appendChild(det);
  }
  $("#ent-count").textContent = String(total);
}
$("#filter").oninput = renderHierarchy;

// ---- selection & inspector -------------------------------------------------------
function select(id: string | null) {
  selected = id; world.select(id); renderHierarchy();
  const m = id ? world.meta.get(id) : undefined;
  const info = id ? agents.get(id) : undefined;
  $("#sel-title").textContent = id ?? "Inspector";
  $("#sel-sub").textContent = m ? `${CATEGORY_LABEL[m.category] ?? m.category} · ${m.kind}` : "";
  if (!id) $("#inspect-body").textContent = "select an entity";
  $("#manual").classList.toggle("hidden", !info);
  ($("#manual-toggle") as HTMLInputElement).checked = false;
  openSensorStreams(info);
  refreshInspector();
}

function openSensorStreams(info?: AgentInfo) {
  for (const s of streams) s.close(); streams = [];
  const grid = $("#sensor-grid"); grid.innerHTML = "";
  const frames = info?.observation_space.frames ?? [];
  $("#sensors").classList.toggle("hidden", frames.length === 0);
  const fpvSel = $("#fpv-cam") as HTMLSelectElement;
  fpvSel.innerHTML = frames.filter(f => f.type === "rgb").map(f => `<option>${f.name}</option>`).join("");
  fpvCam = frames.find(f => f.type === "rgb")?.name ?? null;
  if (!info) return;
  for (const f of frames) {
    const box = document.createElement("div"); box.className = "cam";
    const img = document.createElement("img"); img.alt = f.name;
    const cap = document.createElement("div"); cap.className = "cap";
    cap.innerHTML = `<span>${f.name} · ${f.type} ${f.width}×${f.height}</span><span class="fps">— fps</span>`;
    box.appendChild(img); box.appendChild(cap); grid.appendChild(box);
    const st = new FrameStream(info.agent_id, f.name, f.type === "depth" ? "color" : "jpeg", img, Math.min(f.rate_hz, 12));
    (st as any).capEl = cap.querySelector(".fps");
    streams.push(st);
  }
}

const f3 = (n: number | undefined | null) => (n ?? 0).toFixed(3).padStart(9);
const row = (k: string, v: string) => `<tr><td class="k">${k}</td><td>${v}</td></tr>`;
function quatToEuler(x: number, y: number, z: number, w: number) {
  return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))), Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))];
}
async function refreshInspector() {
  if (!selected) return;
  try {
    const d: EntityDetail = await api.detail(selected);
    const st = d.state;
    let html = "";
    if (d.type_label) html += row("type", `${d.type_label}${d.template ? ` <span class="k">(${d.template})</span>` : ""}`);
    if (st) {
      const p = st.pose.position, v = st.linear_velocity, q = st.pose.orientation;
      const [r, pi, y] = quatToEuler(q.x, q.y, q.z, q.w);
      html += row("position", `${f3(p.x)} ${f3(p.y)} ${f3(p.z)}`);
      if (d.is_agent) html += row("velocity", `${f3(v.x)} ${f3(v.y)} ${f3(v.z)}  <span class="k">|v|</span> ${Math.hypot(v.x, v.y, v.z).toFixed(2)}`);
      else if (d.trajectory) html += row("motion", `kinematic · ${d.trajectory}`);
      html += row("orientation °", `${(r * 57.2958).toFixed(1).padStart(7)} ${(pi * 57.2958).toFixed(1).padStart(7)} ${(y * 57.2958).toFixed(1).padStart(7)}`);
    }
    if (d.dimensions) html += row("dimensions", `${d.dimensions.x.toFixed(1)} × ${d.dimensions.y.toFixed(1)} × ${d.dimensions.z.toFixed(1)} m`);
    html += row("collision", d.collision ? `<span class="ok">enabled</span>` : `<span class="k">none</span>`);
    if (d.trajectory && d.is_agent) html += row("trajectory", d.trajectory);
    if (d.is_agent) {
      const info = agents.get(d.entity_id);
      html += row("control", `${d.control ?? "idle"} <span class="k">· level ${info?.action_space.level ?? "?"} · ${d.physical.armed ? "armed" : "disarmed"}</span>`);
      html += row("observation", `${d.physical.observation_profile}${info ? ` <span class="k">(${info.observation_space.components.join(", ")})</span>` : ""}`);
      const mounts = d.sensor_mounts.length ? d.sensor_mounts.map((m: any) => `${m.name} <span class="k">${m.type}</span>`).join(", ") : "";
      html += row("sensors", d.sensors.map(s => `<span class="sensor-ok">✓</span> ${s}`).join("  ") + (mounts ? `<br/><span class="k">mounts:</span> ${mounts}` : ""));
      if (d.physical.limits) { const l = d.physical.limits; html += row("limits", `xy ${l.max_speed_xy} m/s · z ${l.max_speed_z} m/s · yaw ${l.max_yaw_rate} rad/s`); }
      if (d.physical.mass_kg) html += row("mass", `${d.physical.mass_kg} kg <span class="k">(${d.physical.drone_type})</span>`);
      html += row("status", st && st.pose.position.z > 0.3 ? "airborne" : "grounded");
    } else if (Object.keys(d.physical).length) {
      html += row("params", Object.entries(d.physical).map(([k, v]) => `${k}=${v}`).join(" "));
    }
    $("#inspect-body").innerHTML = `<table>${html}</table>`;
    for (const s of streams) { const el = (s as any).capEl as HTMLElement | undefined; if (el) el.textContent = `${s.fps.toFixed(1)} fps`; }
  } catch { /* transient */ }
}
setInterval(refreshInspector, 250);

// ---- scene loading -----------------------------------------------------------
async function loadScene() {
  try {
    const [desc, ag] = await Promise.all([api.scene(), api.agents()]);
    agents = new Map(ag.map(a => [a.agent_id, a]));
    world.rebuild(desc, ag);
    renderHierarchy();
    if (!selected || !world.meta.has(selected)) { const a = desc.models.find(m => m.is_agent); select(a ? a.entity_id : null); }
    else select(selected);
    const evs = await api.events(0); $("#event-list").innerHTML = ""; lastEventSeq = 0;
    for (const e of evs) addEvent(e);
    if (!framedOnce) { framedOnce = true; frameAll(); }
    refreshReplayPanel();
  } catch (e) { console.warn("scene not available yet", e); }
}
async function refreshScenarios() {
  const sel = $("#scenario-select") as HTMLSelectElement;
  const names = await api.scenarios();
  sel.innerHTML = names.map(n => `<option>${n}</option>`).join("");
  const st = await api.status(); if (st.scenario) sel.value = st.scenario;
}

// ---- events --------------------------------------------------------------------
function addEvent(e: SimEvent) {
  if (e.seq <= lastEventSeq) return;
  lastEventSeq = e.seq;
  const ul = $("#event-list");
  const li = document.createElement("li"); li.className = e.event;
  li.textContent = `${fmtClock(e.sim_time)} ${e.event} ${e.entities.join(" ↔ ")}`; li.title = JSON.stringify(e.data);
  ul.prepend(li); while (ul.children.length > 60) ul.removeChild(ul.lastChild!);
  $("#ev-count").textContent = String(lastEventSeq);
  if (e.event === "collision") { world.markCollision(e.data.position); toast(`collision: ${e.entities.join(" ↔ ")}`); }
  if (e.event === "simulator_crashed") toast("simulator process died — reset to relaunch");
}

// ---- status / websocket --------------------------------------------------------
function applyStatus(s: Partial<SimStatus> & { rtf?: number }) {
  const st: SimStatus = { ...(status ?? ({} as SimStatus)), ...s } as SimStatus;
  status = st;
  $("#sim-time").textContent = fmtClock(s.sim_time ?? 0);
  $("#rtf").textContent = (s.real_time_factor ?? s.rtf ?? 0).toFixed(2);
  $("#iters").textContent = String(s.iterations ?? 0);
  const stepped = st.mode === "stepped";
  $("#btn-play").textContent = st.paused ? "▶" : "⏸";
  ($("#btn-play") as HTMLButtonElement).disabled = stepped;
  ($("#speed-select") as HTMLSelectElement).disabled = stepped;
  if (st.speed !== undefined) ($("#speed-select") as HTMLSelectElement).value = String(st.speed);
  $$("#mode-seg button").forEach(b => b.classList.toggle("active", b.dataset.mode === st.mode));
  const rs = $("#run-state");
  const run = !st.running ? "idle" : stepped ? "stepped" : st.paused ? "paused" : "running";
  rs.textContent = run; rs.className = "chip " + run;
  const ep = $("#ep-status");
  if (st.episode) { ep.textContent = `${st.episode.status} · seed ${st.episode.seed}${stepped ? ` · step ${st.episode.step_count}` : ""}`; ep.className = "chip dim " + st.episode.status; }
  else { ep.textContent = "no episode"; ep.className = "chip dim"; }
}
const tele = new Telemetry((m: WsMessage) => {
  switch (m.type) {
    case "hello": applyStatus(m.status); loadScene(); break;
    case "state": applyStatus({ sim_time: m.sim_time, paused: m.paused, mode: m.mode as any, real_time_factor: m.rtf, iterations: m.iterations }); world.updatePoses(m.poses, m.vel, m.wp);
      if (overlay) world.setTaskOverlay(overlay.agent, overlay.target, overlay.start ?? null, overlay.success_radius ?? 1, overlay.status); break;
    case "event": addEvent(m); break;
    case "scenario_loaded": framedOnce = false; case "scene_changed": loadScene(); refreshScenarios(); api.status().then(applyStatus); break;
    case "reset": world.clearTrails(); loadScene(); api.status().then(applyStatus); break;
    case "restored": world.clearTrails(); loadScene(); toast(`restored ${m.snapshot} (divergence ${m.divergence?.max_position_m?.toExponential(1)} m)`, true); break;
    case "overlay": applyOverlay(m.data); break;
    case "ack": if (m.status) applyStatus(m.status); break;
    case "error": toast(m.message); break;
  }
}, ok => ($("#conn").className = "dot" + (ok ? " ok" : "")));

// ---- transport controls -------------------------------------------------------------
$("#btn-play").onclick = () => (status?.paused ? api.resume() : api.pause()).catch(e => toast(String(e.message)));
$("#btn-step").onclick = () => tele.send({ type: "step", steps: Number(($("#step-input") as HTMLInputElement).value) || 1 });
$("#btn-reset").onclick = () => { const v = ($("#seed-input") as HTMLInputElement).value; tele.send({ type: "reset", ...(v ? { seed: Number(v) } : {}) }); };
$("#btn-load").onclick = () => api.load(($("#scenario-select") as HTMLSelectElement).value).catch(e => toast(String(e.message)));
$("#speed-select").onchange = () => api.speed(Number(($("#speed-select") as HTMLSelectElement).value)).catch(e => toast(String(e.message)));
$$("#mode-seg button").forEach(b => (b.onclick = () => tele.send({ type: "mode", mode: b.dataset.mode })));
$$("#view-seg button").forEach(b => (b.onclick = () => setViewMode(b.dataset.view as ViewMode)));
function setViewMode(v: ViewMode) {
  viewMode = v; document.body.className = `mode-${v}`;
  $$("#view-seg button").forEach(x => x.classList.toggle("active", x.dataset.view === v));
  if (v === "replay") refreshReplayPanel();
}
$$("#overlay-chips button").forEach(b => (b.onclick = () => {
  const k = b.dataset.ov as keyof Overlays; world.overlays[k] = !world.overlays[k];
  b.classList.toggle("on", world.overlays[k]); world.applyOverlays();
}));
$$("#cam-modes button[data-cam]").forEach(b => (b.onclick = () => setCamMode(b.dataset.cam as CamMode)));
function setCamMode(mode: CamMode) {
  camMode = mode;
  $$("#cam-modes button[data-cam]").forEach(x => x.classList.toggle("active", x.dataset.cam === mode));
  $("#fpv-cam").classList.toggle("hidden", mode !== "fpv" || streams.length === 0);
  controls.enabled = mode === "orbit" || mode === "follow";
  camera.up.set(0, 0, 1);
  const g = selected ? world.models.get(selected) : undefined;
  if (mode === "top" && g) flyTo(new THREE.Vector3(g.position.x, g.position.y + 0.01, g.position.z + 70), g.position.clone());
  if (mode === "follow" && g) flyTo(g.position.clone().add(new THREE.Vector3(-8, -8, 4)), g.position.clone(), 500);
  if (mode === "orbit" && g) frameEntity(selected!);
}
$("#fpv-cam").onchange = () => (fpvCam = ($("#fpv-cam") as HTMLSelectElement).value);
$("#btn-fit").onclick = () => { setCamMode("orbit"); frameAll(); };

// keyboard: space = play/pause (realtime) or step (stepped); manual flight keys when enabled
const keys = new Set<string>();
document.addEventListener("keydown", e => {
  const tag = (e.target as HTMLElement).tagName;
  if (tag === "INPUT" || tag === "SELECT") return;
  if (e.code === "Space") { e.preventDefault(); status?.mode === "stepped" ? $("#btn-step").click() : $("#btn-play").click(); return; }
  if (e.code === "KeyF" && !manualOn()) { setCamMode(camMode === "follow" ? "orbit" : "follow"); return; }
  if (manualOn()) { keys.add(e.code); e.preventDefault(); if (e.code === "KeyZ") toggleArm(); }
});
document.addEventListener("keyup", e => keys.delete(e.code));
const manualOn = () => ($("#manual-toggle") as HTMLInputElement).checked && !!selected && agents.has(selected);
let armed = true;
function toggleArm() { if (!selected) return; armed = !armed; tele.send({ type: "action", agent_id: selected, action: { type: "arm", armed } }); }
let lastManualSent = 0, wasMoving = false;
setInterval(() => {
  if (!manualOn() || !selected) return;
  const lim = agents.get(selected)?.action_space.limits ?? { max_speed_xy: 4, max_speed_z: 2, max_yaw_rate: 1 };
  const sp = Math.min(3.0, lim.max_speed_xy * 0.6), vz = Math.min(1.5, lim.max_speed_z * 0.7), yr = Math.min(1.0, lim.max_yaw_rate * 0.7);
  const v = { vx: 0, vy: 0, vz: 0, yaw_rate: 0 };
  if (keys.has("KeyW")) v.vx += sp; if (keys.has("KeyS")) v.vx -= sp;
  if (keys.has("KeyA")) v.vy += sp; if (keys.has("KeyD")) v.vy -= sp;
  if (keys.has("KeyR")) v.vz += vz; if (keys.has("KeyF")) v.vz -= vz;
  if (keys.has("KeyQ")) v.yaw_rate += yr; if (keys.has("KeyE")) v.yaw_rate -= yr;
  const moving = v.vx || v.vy || v.vz || v.yaw_rate;
  const now = performance.now();
  if (keys.has("KeyX")) { tele.send({ type: "action", agent_id: selected, action: { type: "hold" } }); keys.delete("KeyX"); wasMoving = false; return; }
  if (moving && now - lastManualSent > 100) { lastManualSent = now; wasMoving = true; tele.send({ type: "action", agent_id: selected, action: { type: "velocity", ...v, frame: "body" } }); }
  else if (!moving && wasMoving) { wasMoving = false; tele.send({ type: "action", agent_id: selected, action: { type: "hold" } }); }
}, 50);

// picking
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
renderer.domElement.addEventListener("pointerdown", e => {
  const down = { x: e.clientX, y: e.clientY };
  const up = (ev: PointerEvent) => {
    renderer.domElement.removeEventListener("pointerup", up);
    if (Math.hypot(ev.clientX - down.x, ev.clientY - down.y) > 4) return;
    const r = renderer.domElement.getBoundingClientRect();
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const id = world.pick(ray); if (id) select(id);
  };
  renderer.domElement.addEventListener("pointerup", up);
});

// ---- task overlay (external task/tool annotations; see POST /overlay) --------------------
function applyOverlay(o: TaskOverlay | null) {
  overlay = o;
  $("#taskpanel").classList.toggle("hidden", !o);
  if (!o) { world.setTaskOverlay(null, null, null); return; }
  const st = $("#task-status"); st.textContent = o.status; st.className = "badge " + (o.status === "reached" ? "ok" : o.status === "running" ? "" : "bad");
  $("#task-body").innerHTML =
    `<span class="k">${o.task}</span> ${o.level ?? ""} · ${o.observation_mode ?? ""} · agent ${o.agent}\n` +
    `<span class="k">distance</span> ${o.distance.toFixed(2).padStart(7)} m   <span class="k">radius</span> ${(o.success_radius ?? 0).toFixed(1)} m\n` +
    `<span class="k">step</span> ${String(o.step).padStart(4)} / ${o.max_steps ?? "?"}   <span class="k">reward</span> ${o.reward.toFixed(3).padStart(8)}   <span class="k">return</span> ${o.return.toFixed(1).padStart(7)}\n` +
    (o.observation ? `<span class="k">obs</span> [${o.observation.slice(0, 6).map(v => v.toFixed(2)).join(", ")}, …]` : "");
  const bars = $("#task-action"); bars.innerHTML = "";
  for (const [i, lbl] of ["vx", "vy", "vz"].entries()) {
    const v = o.action ? o.action[i] : 0;
    const bar = document.createElement("div"); bar.className = "bar";
    const fill = document.createElement("span"); const pct = Math.min(1, Math.abs(v)) * 50;
    fill.style.width = pct + "%"; fill.style.left = v >= 0 ? "50%" : (50 - pct) + "%";
    const l = document.createElement("label"); l.textContent = `${lbl} ${v.toFixed(2)}`;
    bar.appendChild(fill); bar.appendChild(l); bars.appendChild(bar);
  }
  world.setTaskOverlay(o.agent, o.target, o.start ?? null, o.success_radius ?? 1, o.status);
}
api.overlay().then(o => applyOverlay(o && o.task ? o : null)).catch(() => {});

// ---- recording / snapshots / replay ------------------------------------------------------
let recording = false;
$("#btn-rec").onclick = async () => {
  try {
    if (!recording) { await api.recordingStart(); recording = true; $("#btn-rec").textContent = "■ Stop rows"; toast("recording rows (replay buffer) for this episode", true); }
    else { await api.recordingStop(); recording = false; $("#btn-rec").textContent = "● Record rows"; }
    refreshReplayPanel();
  } catch (e: any) { toast(e.message); }
};
$("#btn-snap").onclick = async () => {
  try { const s = await api.snapshot(prompt("snapshot name", "snapshot") ?? "snapshot"); toast(`snapshot ${s.snapshot_id} @ ${fmtClock(s.sim_time)}`, true); refreshReplayPanel(); }
  catch (e: any) { toast(e.message); }
};
async function refreshReplayPanel() {
  if (viewMode !== "replay") return;
  try {
    const [ws, snaps, recs] = await Promise.all([api.worldState().catch(() => null), api.snapshots(), api.recordings()]);
    const r = ws?.recording;
    $("#rec-status").textContent = r ? `episode ${r.episode_id} · ${r.actions} actions logged · ${r.rows} rows${r.observations ? " (+obs)" : ""}` : "no episode";
    const sl = $("#snapshot-list"); sl.innerHTML = "";
    for (const s of snaps.slice().reverse().slice(0, 12)) {
      const li = document.createElement("li");
      li.innerHTML = `<span class="grow" title="${s.snapshot_id}">${s.name} · ${s.scenario} · seed ${s.seed} · t ${fmtClock(s.sim_time)}</span>`;
      const b = document.createElement("button"); b.textContent = "Restore";
      b.onclick = async () => { b.disabled = true; try { const res = await api.restore(s.snapshot_id); toast(`restored (divergence ${res.divergence.max_position_m.toExponential(1)} m)`, true); } catch (e: any) { toast(e.message); } b.disabled = false; };
      li.appendChild(b); sl.appendChild(li);
    }
    const rl = $("#recording-list"); rl.innerHTML = "";
    for (const m of recs.slice().reverse().slice(0, 12)) {
      const li = document.createElement("li");
      li.innerHTML = `<span class="grow" title="${m.recording_id}">${m.scenario} · seed ${m.seed} · ${m.actions} act · ${m.rows} rows · ${m.status}</span>`;
      const b = document.createElement("button"); b.textContent = "Replay";
      b.onclick = async () => { b.disabled = true; try { await api.replay(m.recording_id); toast("replayed", true); } catch (e: any) { toast(e.message); } b.disabled = false; };
      li.appendChild(b); rl.appendChild(li);
    }
  } catch { /* offline */ }
}
setInterval(() => { if (viewMode === "replay") refreshReplayPanel(); }, 3000);

// ---- environment status (development mode) ---------------------------------------------------
async function refreshStatusPanel() {
  if (viewMode !== "development") return;
  try {
    const m = await api.metrics(); const st = status;
    const run = !st?.running ? "IDLE" : st.mode === "stepped" ? "STEP-CONTROLLED" : st.paused ? "PAUSED" : "RUNNING";
    const physics = st?.running && (st.paused || st.mode === "stepped" || m.sim_hz > 20) ? `<span class="ok">OK</span>` : st?.running ? `<span class="warn">SLOW (${m.sim_hz.toFixed(0)} Hz)</span>` : `<span class="k">—</span>`;
    const fps = Object.entries(m.sensor_fps).map(([k, v]) => `${k.split("/")[1]} ${v}`).join(", ");
    $("#status-body").innerHTML =
      `<span class="k">simulation</span>       ${run}\n<span class="k">real-time factor</span> ${m.real_time_factor.toFixed(2)}  <span class="k">target</span> ${st?.speed === 0 ? "max" : (st?.speed ?? 1) + "×"}\n` +
      `<span class="k">physics</span>          ${physics}  <span class="k">dt</span> ${((m.step_size ?? 0) * 1000).toFixed(0)} ms  <span class="k">${m.sim_hz.toFixed(0)} Hz</span>\n` +
      `<span class="k">entities</span>         ${m.entity_count}   <span class="k">agents</span> ${m.agent_count}\n<span class="k">sensors</span>          ${Object.keys(m.sensor_fps).length} cameras active${fps ? ` (${fps} fps)` : ""}\n` +
      `<span class="k">API</span>              <span class="${$("#conn").classList.contains("ok") ? "ok" : "bad"}">${$("#conn").classList.contains("ok") ? "CONNECTED" : "DISCONNECTED"}</span>  <span class="k">clients</span> ${m.telemetry_clients}+${m.sensor_clients}\n` +
      `<span class="k">step latency</span>     ${m.api_step_latency_ms != null ? m.api_step_latency_ms.toFixed(0) + " ms" : "—"}   <span class="k">events</span> ${m.events_total}\n<span class="k">simulation time</span>  ${fmtClock(st?.sim_time ?? 0)}`;
  } catch { /* offline */ }
}
setInterval(refreshStatusPanel, 1000);

// ---- render loop -----------------------------------------------------------------------
const tmp = new THREE.Vector3(), fwd = new THREE.Vector3(), upv = new THREE.Vector3();
const ease = (t: number) => 1 - Math.pow(1 - t, 3);
function animate() {
  requestAnimationFrame(animate);
  if (tween) {
    const k = Math.min(1, (performance.now() - tween.start) / tween.dur), e = ease(k);
    camera.position.lerpVectors(tween.p0, tween.p1, e); controls.target.lerpVectors(tween.t0, tween.t1, e);
    if (k >= 1) tween = null;
  }
  const g = selected ? world.models.get(selected) : undefined;
  if (g && camMode !== "orbit" && !tween) {
    if (camMode === "follow") {
      controls.target.lerp(g.position, 0.12);
      camera.position.lerp(tmp.copy(g.position).add(new THREE.Vector3(-8, -8, 4)), 0.04);
    } else if (camMode === "top") {
      controls.target.copy(g.position); camera.position.set(g.position.x, g.position.y + 0.01, g.position.z + 70); camera.lookAt(g.position);
    } else if (camMode === "fpv") {
      const info = selected ? agents.get(selected) : undefined;
      const cam = info?.observation_space.frames.find(f => f.name === fpvCam) ?? info?.observation_space.frames.find(f => f.type === "rgb");
      const pose = cam?.pose ?? [0.12, 0, -0.02, 0, 0.35, 0];
      const q = g.quaternion.clone().multiply(rpyToQuat(pose[3], pose[4], pose[5]));
      camera.position.copy(tmp.set(pose[0], pose[1], pose[2]).applyQuaternion(g.quaternion).add(g.position));
      fwd.set(1, 0, 0).applyQuaternion(q); upv.set(0, 0, 1).applyQuaternion(q);
      camera.up.copy(upv); camera.lookAt(tmp.clone().add(fwd));
    }
  }
  if (controls.enabled) controls.update();
  renderer.render(world.scene, camera);
}
animate();
refreshScenarios();

// deep links: #view=development&select=drone_01&cam=follow
{
  const h = new URLSearchParams(location.hash.replace(/^#/, ""));
  const v = h.get("view") as ViewMode | null; if (v && ["operations", "development", "replay"].includes(v)) setViewMode(v);
  const sel = h.get("select"); const cam = h.get("cam") as CamMode | null;
  if (sel || cam) setTimeout(() => { if (sel && world.meta.has(sel)) select(sel); if (cam) setCamMode(cam); }, 2500);
}
