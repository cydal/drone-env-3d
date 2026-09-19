import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { api, FrameStream, Telemetry, type AgentInfo, type SimEvent, type SimStatus, type WsMessage } from "./api";
import { WorldScene, type Overlays } from "./scene";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

// ---- renderer ----------------------------------------------------------------
const viewport = $("#viewport");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.shadowMap.enabled = true;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
viewport.appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 2000);
camera.up.set(0, 0, 1);
camera.position.set(-16, -14, 9);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 2);
controls.enableDamping = true;
const world = new WorldScene();

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport); resize();

// ---- state -------------------------------------------------------------------
let selected: string | null = null;
let camMode: "orbit" | "follow" | "top" | "fpv" = "orbit";
let status: SimStatus | null = null;
let agents = new Map<string, AgentInfo>();
let lastEventSeq = 0;
let streams: FrameStream[] = [];

function toast(msg: string) {
  const t = $("#toast"); t.textContent = msg; t.style.display = "block";
  setTimeout(() => (t.style.display = "none"), 3500);
}

// ---- entities / telemetry ------------------------------------------------------
function renderEntityList() {
  const ul = $("#entity-list"); ul.innerHTML = "";
  const rank = (m: any) => (m.is_agent ? 0 : m.kind === "target" || m.kind === "vehicle" ? 1 : 2);
  const models = [...world.meta.values()].sort((a, b) => rank(a) - rank(b) || a.entity_id.localeCompare(b.entity_id));
  for (const m of models) {
    if (m.entity_id === "ground") continue;
    const li = document.createElement("li");
    li.className = (m.is_agent ? "agent " : rank(m) === 1 ? "dyn " : "") + (m.entity_id === selected ? "sel" : "");
    li.innerHTML = `<span>${m.entity_id}</span><span class="kind">${m.kind}</span>`;
    li.onclick = () => select(m.entity_id);
    ul.appendChild(li);
  }
}

function select(id: string | null) {
  selected = id; world.select(id); renderEntityList();
  $("#sel-title").textContent = id ?? "Telemetry";
  if (!id) $("#telemetry").textContent = "select an entity";
  const info = id ? agents.get(id) : undefined;
  $("#manual").classList.toggle("hidden", !info);
  $("#sensors").classList.toggle("hidden", !info || info.observation_space.frames.length === 0);
  ($("#manual-toggle") as HTMLInputElement).checked = false;
  openSensorStreams(info);
}

function openSensorStreams(info?: AgentInfo) {
  for (const s of streams) s.close();
  streams = [];
  if (!info) return;
  for (const f of info.observation_space.frames) {
    const img = $(`#img-${f.name}`) as HTMLImageElement | null;
    if (!img) continue;
    streams.push(new FrameStream(info.agent_id, f.name, f.type === "depth" ? "color" : "jpeg", img, 10));
  }
}

const f = (n: number | undefined | null) => (n ?? 0).toFixed(3).padStart(9);
async function refreshTelemetry() {
  if (!selected) return;
  try {
    const info = agents.get(selected);
    const s = info ? await api.observation(selected) : await api.entity(selected);
    const st = info ? (s.state ?? (await api.entity(selected))) : s;
    const q = st.pose.orientation;
    const [roll, pitch, yaw] = quatToEuler(q.x, q.y, q.z, q.w);
    let txt = `<span class="k">Position</span>\n  x ${f(st.pose.position.x)}  y ${f(st.pose.position.y)}  z ${f(st.pose.position.z)}\n`;
    txt += `<span class="k">Velocity (world)</span>\n  x ${f(st.linear_velocity?.x)}  y ${f(st.linear_velocity?.y)}  z ${f(st.linear_velocity?.z)}\n`;
    txt += `<span class="k">Orientation (rpy °)</span>\n  ${f(roll * 57.2958)} ${f(pitch * 57.2958)} ${f(yaw * 57.2958)}\n`;
    if (info) {
      txt += `<span class="k">Observation profile</span>\n  ${s.profile}: ${info.observation_space.components.join(", ")}\n`;
      if (s.imu) txt += `<span class="k">IMU accel (body)</span>\n  x ${f(s.imu.linear_acceleration.x)}  y ${f(s.imu.linear_acceleration.y)}  z ${f(s.imu.linear_acceleration.z)}\n`;
      if (s.gps) txt += `<span class="k">GPS</span>\n  ${s.gps.latitude_deg.toFixed(6)} ${s.gps.longitude_deg.toFixed(6)} alt ${s.gps.altitude.toFixed(2)}\n`;
      if (s.nearby_agents) txt += `<span class="k">Nearby</span>\n  ${s.nearby_agents.map((n: any) => `${n.agent_id} ${n.distance.toFixed(1)}m`).join(", ") || "—"}\n`;
      txt += `<span class="k">Control</span>\n  level ${info.action_space.level} · mode ${s.control_mode} · ${s.armed ? "armed" : "disarmed"}\n`;
      txt += `<span class="k">Sensors</span>\n  ${info.sensors.join(", ")}\n`;
      txt += `<span class="k">Status</span>\n  ${s.grounded === false ? "airborne" : s.grounded ? "grounded" : "unknown"}`;
    } else {
      txt += `<span class="k">Kind</span>\n  ${world.meta.get(selected)?.kind}`;
    }
    $("#telemetry").innerHTML = txt;
    const fps = streams.map(s => s.fps.toFixed(1)).join(" / ");
    $("#sensor-fps").textContent = fps ? fps + " fps" : "";
  } catch { /* transient */ }
}
setInterval(refreshTelemetry, 250);

function quatToEuler(x: number, y: number, z: number, w: number) {
  return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))];
}

// ---- scene loading -----------------------------------------------------------
async function loadScene() {
  try {
    const [desc, ag] = await Promise.all([api.scene(), api.agents()]);
    agents = new Map(ag.map(a => [a.agent_id, a]));
    world.rebuild(desc, ag);
    renderEntityList();
    if (!selected || !world.meta.has(selected)) { const a = desc.models.find(m => m.is_agent); select(a ? a.entity_id : null); }
    else select(selected);
    const evs = await api.events(0); $("#event-list").innerHTML = ""; lastEventSeq = 0;
    for (const e of evs) addEvent(e);
  } catch (e) { console.warn("scene not available yet", e); }
}
async function refreshScenarios() {
  const sel = $("#scenario-select") as HTMLSelectElement;
  const names = await api.scenarios();
  sel.innerHTML = names.map(n => `<option>${n}</option>`).join("");
  const st = await api.status();
  if (st.scenario) sel.value = st.scenario;
}

// ---- events --------------------------------------------------------------------
function addEvent(e: SimEvent) {
  if (e.seq <= lastEventSeq) return;
  lastEventSeq = e.seq;
  const ul = $("#event-list");
  const li = document.createElement("li"); li.className = e.event;
  li.textContent = `${e.sim_time.toFixed(2)}s ${e.event} ${e.entities.join(" ↔ ")}`;
  li.title = JSON.stringify(e.data);
  ul.prepend(li);
  while (ul.children.length > 40) ul.removeChild(ul.lastChild!);
  $("#ev-count").textContent = String(lastEventSeq);
  if (e.event === "collision") { world.markCollision(e.data.position); toast(`collision: ${e.entities.join(" ↔ ")}`); }
}

// ---- status / websocket --------------------------------------------------------
function applyStatus(s: Partial<SimStatus> & { rtf?: number }) {
  const st: SimStatus = { ...(status ?? ({} as SimStatus)), ...s } as SimStatus;
  status = st;
  $("#sim-time").textContent = (s.sim_time ?? 0).toFixed(3);
  $("#rtf").textContent = (s.real_time_factor ?? s.rtf ?? 0).toFixed(2);
  $("#iters").textContent = String(s.iterations ?? 0);
  const stepped = st.mode === "stepped";
  $("#btn-play").textContent = st.paused ? "▶" : "⏸";
  ($("#btn-play") as HTMLButtonElement).disabled = stepped;
  document.querySelectorAll<HTMLButtonElement>("#mode-seg button").forEach(b => b.classList.toggle("active", b.dataset.mode === st.mode));
  if (st.episode) $("#ep-status").textContent = `${st.episode.status} · seed ${st.episode.seed}${stepped ? ` · step ${st.episode.step_count}` : ""}`;
}
const tele = new Telemetry((m: WsMessage) => {
  switch (m.type) {
    case "hello": applyStatus(m.status); loadScene(); break;
    case "state": applyStatus({ sim_time: m.sim_time, paused: m.paused, mode: m.mode as any, real_time_factor: m.rtf, iterations: m.iterations }); world.updatePoses(m.poses, m.vel); break;
    case "event": addEvent(m); break;
    case "scenario_loaded": case "scene_changed": loadScene(); refreshScenarios(); api.status().then(applyStatus); break;
    case "reset": world.clearTrails(); loadScene(); api.status().then(applyStatus); break;
    case "ack": if (m.status) applyStatus(m.status); break;
    case "error": toast(m.message); break;
  }
}, ok => ($("#conn").className = "dot" + (ok ? " ok" : "")));

// ---- controls -----------------------------------------------------------------
$("#btn-play").onclick = () => (status?.paused ? api.resume() : api.pause()).catch(e => toast(String(e.message)));
$("#btn-step").onclick = () => tele.send({ type: "step", steps: Number(($("#step-input") as HTMLInputElement).value) || 1 });
$("#step-input").oninput = () => ($("#step-n").textContent = ($("#step-input") as HTMLInputElement).value);
$("#btn-reset").onclick = () => {
  const v = ($("#seed-input") as HTMLInputElement).value;
  tele.send({ type: "reset", ...(v ? { seed: Number(v) } : {}) });
};
$("#btn-load").onclick = () => api.load(($("#scenario-select") as HTMLSelectElement).value, status?.mode).catch(e => toast(String(e.message)));
document.querySelectorAll<HTMLButtonElement>("#mode-seg button").forEach(b => (b.onclick = () => tele.send({ type: "mode", mode: b.dataset.mode })));
document.querySelectorAll<HTMLButtonElement>("#overlay-chips button").forEach(b => (b.onclick = () => {
  const k = b.dataset.ov as keyof Overlays; world.overlays[k] = !world.overlays[k];
  b.classList.toggle("on", world.overlays[k]); world.applyOverlays();
}));
document.querySelectorAll<HTMLButtonElement>("#cam-modes button").forEach(b => (b.onclick = () => {
  camMode = b.dataset.cam as any;
  document.querySelectorAll("#cam-modes button").forEach(x => x.classList.toggle("active", x === b));
  controls.enabled = camMode === "orbit" || camMode === "follow";
  camera.up.set(0, 0, 1);
  if (camMode === "top") camera.up.set(0, 1, 0);
}));

// keyboard: space = play/pause (realtime) or step (stepped); manual flight keys when enabled
const keys = new Set<string>();
document.addEventListener("keydown", e => {
  const tag = (e.target as HTMLElement).tagName;
  if (tag === "INPUT" || tag === "SELECT") return;
  if (e.code === "Space") { e.preventDefault(); status?.mode === "stepped" ? $("#btn-step").click() : $("#btn-play").click(); return; }
  if (manualOn()) { keys.add(e.code); e.preventDefault(); if (e.code === "KeyZ") toggleArm(); }
});
document.addEventListener("keyup", e => keys.delete(e.code));
const manualOn = () => ($("#manual-toggle") as HTMLInputElement).checked && !!selected && agents.has(selected);
let armed = true;
function toggleArm() { if (!selected) return; armed = !armed; tele.send({ type: "action", agent_id: selected, action: { type: "arm", armed } }); }
let lastManualSent = 0, wasMoving = false;
setInterval(() => {
  if (!manualOn() || !selected) return;
  const sp = 2.0, v = { vx: 0, vy: 0, vz: 0, yaw_rate: 0 };
  if (keys.has("KeyW")) v.vx += sp; if (keys.has("KeyS")) v.vx -= sp;
  if (keys.has("KeyA")) v.vy += sp; if (keys.has("KeyD")) v.vy -= sp;
  if (keys.has("KeyR")) v.vz += 1.5; if (keys.has("KeyF")) v.vz -= 1.5;
  if (keys.has("KeyQ")) v.yaw_rate += 1.0; if (keys.has("KeyE")) v.yaw_rate -= 1.0;
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
    const id = world.pick(ray);
    if (id) select(id);
  };
  renderer.domElement.addEventListener("pointerup", up);
});

// ---- health strip ----------------------------------------------------------------
async function refreshHealth() {
  try {
    const m = await api.metrics();
    const fps = Object.entries(m.sensor_fps).map(([k, v]) => `${k.split("/")[1]} ${v}`).join(" ");
    $("#health").innerHTML =
      `rtf <b>${m.real_time_factor.toFixed(2)}</b>  sim <b>${m.sim_hz.toFixed(0)} Hz</b>  dt <b>${((m.step_size ?? 0) * 1000).toFixed(0)} ms</b>\n` +
      `entities <b>${m.entity_count}</b>  agents <b>${m.agent_count}</b>  clients <b>${m.telemetry_clients}+${m.sensor_clients}</b>\n` +
      `step latency <b>${m.api_step_latency_ms != null ? m.api_step_latency_ms.toFixed(0) + " ms" : "—"}</b>  events <b>${m.events_total}</b>` +
      (fps ? `\nsensor fps <b>${fps}</b>` : "");
  } catch { /* offline */ }
}
setInterval(refreshHealth, 1000);

// ---- render loop -------------------------------------------------------------------
const tmp = new THREE.Vector3(), fwd = new THREE.Vector3(), upv = new THREE.Vector3();
function animate() {
  requestAnimationFrame(animate);
  const g = selected ? world.models.get(selected) : undefined;
  if (g && camMode !== "orbit") {
    if (camMode === "follow") {
      controls.target.lerp(g.position, 0.15);
      camera.position.lerp(tmp.copy(g.position).add(new THREE.Vector3(-6, -6, 3)), 0.05);
    } else if (camMode === "top") {
      controls.target.copy(g.position);
      camera.position.set(g.position.x, g.position.y, g.position.z + 60);
      camera.lookAt(g.position);
    } else if (camMode === "fpv") {
      // camera mounted at (0.12, 0, -0.02) on the body, pitched 0.35 rad down (matches the SDF)
      tmp.set(0.12, 0, -0.02).applyQuaternion(g.quaternion).add(g.position);
      camera.position.copy(tmp);
      fwd.set(Math.cos(0.35), 0, -Math.sin(0.35)).applyQuaternion(g.quaternion);
      upv.set(0, 0, 1).applyQuaternion(g.quaternion);
      camera.up.copy(upv);
      camera.lookAt(tmp.clone().add(fwd));
    }
  }
  if (controls.enabled) controls.update();
  renderer.render(world.scene, camera);
}
animate();
refreshScenarios();
