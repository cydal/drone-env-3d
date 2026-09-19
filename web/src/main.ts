import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { api, connectWs, type WsMessage } from "./api";
import { WorldScene } from "./scene";

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
let camMode: "orbit" | "follow" | "top" = "orbit";
let paused = true;

// ---- UI: entities & telemetry -----------------------------------------------
function renderEntityList() {
  const ul = $("#entity-list"); ul.innerHTML = "";
  const models = [...world.meta.values()].sort((a, b) => Number(b.is_agent) - Number(a.is_agent) || a.entity_id.localeCompare(b.entity_id));
  for (const m of models) {
    if (m.entity_id === "ground") continue;
    const li = document.createElement("li");
    li.className = (m.is_agent ? "agent " : "") + (m.entity_id === selected ? "sel" : "");
    li.innerHTML = `<span>${m.entity_id}</span><span class="kind">${m.kind}</span>`;
    li.onclick = () => select(m.entity_id);
    ul.appendChild(li);
  }
}
function select(id: string | null) {
  selected = id; world.select(id); renderEntityList();
  $("#sel-title").textContent = id ?? "Telemetry";
  if (!id) $("#telemetry").textContent = "select an entity";
}
const f = (n: number | undefined) => (n ?? 0).toFixed(3).padStart(9);
async function refreshTelemetry() {
  if (!selected) return;
  try {
    const m = world.meta.get(selected);
    const s = m?.is_agent ? await api.observation(selected) : await api.entity(selected);
    const st = m?.is_agent ? s.state : s;
    const q = st.pose.orientation;
    const [roll, pitch, yaw] = quatToEuler(q.x, q.y, q.z, q.w);
    let txt = `<span class="k">Position</span>\n  x ${f(st.pose.position.x)}  y ${f(st.pose.position.y)}  z ${f(st.pose.position.z)}\n`;
    txt += `<span class="k">Velocity (world)</span>\n  x ${f(st.linear_velocity?.x)}  y ${f(st.linear_velocity?.y)}  z ${f(st.linear_velocity?.z)}\n`;
    txt += `<span class="k">Angular vel</span>\n  x ${f(st.angular_velocity?.x)}  y ${f(st.angular_velocity?.y)}  z ${f(st.angular_velocity?.z)}\n`;
    txt += `<span class="k">Orientation (rpy °)</span>\n  ${f(roll * 57.2958)} ${f(pitch * 57.2958)} ${f(yaw * 57.2958)}\n`;
    if (s.imu) txt += `<span class="k">IMU accel</span>\n  x ${f(s.imu.linear_acceleration.x)}  y ${f(s.imu.linear_acceleration.y)}  z ${f(s.imu.linear_acceleration.z)}\n`;
    if (s.sensors) txt += `<span class="k">Sensors</span>\n  ${s.sensors.join(", ")}\n`;
    txt += `<span class="k">Status</span>\n  ${m?.is_agent ? (st.pose.position.z > 0.3 ? "airborne" : "grounded") : m?.kind}`;
    $("#telemetry").innerHTML = txt;
  } catch { /* transient */ }
}
setInterval(refreshTelemetry, 200);

function quatToEuler(x: number, y: number, z: number, w: number) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}

// ---- scene loading -----------------------------------------------------------
async function loadScene() {
  try {
    const desc = await api.scene();
    world.rebuild(desc);
    renderEntityList();
    if (!selected) { const a = desc.models.find(m => m.is_agent); if (a) select(a.entity_id); }
  } catch (e) { console.warn("scene not available yet", e); }
}
async function refreshScenarios() {
  const sel = $("#scenario-select") as HTMLSelectElement;
  const names = await api.scenarios();
  sel.innerHTML = names.map(n => `<option>${n}</option>`).join("");
  const st = await api.status();
  if (st.scenario) sel.value = st.scenario;
}

// ---- websocket ---------------------------------------------------------------
function applyStatus(s: { sim_time: number; paused: boolean; real_time_factor?: number; rtf?: number; iterations: number }) {
  paused = s.paused;
  $("#sim-time").textContent = s.sim_time.toFixed(3);
  $("#rtf").textContent = (s.real_time_factor ?? s.rtf ?? 0).toFixed(2);
  $("#iters").textContent = String(s.iterations);
  $("#btn-play").textContent = paused ? "▶" : "⏸";
}
connectWs((m: WsMessage) => {
  switch (m.type) {
    case "hello": applyStatus(m.status as any); loadScene(); break;
    case "state": applyStatus(m); world.updatePoses(m.poses); break;
    case "scenario_loaded": case "scene_changed": loadScene(); refreshScenarios(); break;
    case "error": console.error(m.message); break;
  }
}, ok => $("#conn").className = "dot" + (ok ? " ok" : ""));

// ---- controls ----------------------------------------------------------------
$("#btn-play").onclick = () => (paused ? api.resume() : api.pause());
$("#btn-step").onclick = () => api.step(1);
$("#btn-reset").onclick = () => api.reset();
$("#btn-load").onclick = () => api.load(($("#scenario-select") as HTMLSelectElement).value);
document.addEventListener("keydown", e => { if (e.code === "Space" && (e.target as HTMLElement).tagName !== "SELECT") { e.preventDefault(); $("#btn-play").click(); } });
document.querySelectorAll<HTMLButtonElement>("#cam-modes button").forEach(b => b.onclick = () => {
  camMode = b.dataset.cam as any;
  document.querySelectorAll("#cam-modes button").forEach(x => x.classList.toggle("active", x === b));
  if (camMode === "top") { camera.position.set(controls.target.x, controls.target.y, 120); camera.up.set(0, 1, 0); }
  else camera.up.set(0, 0, 1);
});
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

// ---- loop --------------------------------------------------------------------
const tmp = new THREE.Vector3();
function animate() {
  requestAnimationFrame(animate);
  if (selected && camMode !== "orbit") {
    const g = world.models.get(selected);
    if (g) {
      if (camMode === "follow") {
        tmp.copy(g.position);
        controls.target.lerp(tmp, 0.15);
        const desired = tmp.clone().add(new THREE.Vector3(-6, -6, 3));
        camera.position.lerp(desired, 0.05);
      } else if (camMode === "top") {
        controls.target.set(g.position.x, g.position.y, g.position.z);
        camera.position.set(g.position.x, g.position.y, g.position.z + 60);
      }
    }
  }
  controls.update();
  renderer.render(world.scene, camera);
}
animate();
refreshScenarios();
