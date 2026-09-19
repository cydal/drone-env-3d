// three.js scene built from the API's SceneDesc, lit by the world's environment preset,
// plus debug overlays. Gazebo is Z-up: coordinates are used verbatim with up = +Z.
import * as THREE from "three";
import type { AgentInfo, Environment, ModelDesc, Pose, SceneDesc, Visual } from "./api";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export interface Overlays { trails: boolean; planned: boolean; velocity: boolean; axes: boolean; labels: boolean; collisions: boolean; frustum: boolean; bounds: boolean; collisionGeom: boolean; grid: boolean; areas: boolean }

const TRAIL_LEN = 900;   // ~36 s at 25 Hz
const EMISSIVE_NAMES = ["lamp", "beacon", "head", "ring", "marking", "edge", "stripe"];

class Trail {
  readonly line: THREE.Line;
  private pos = new Float32Array(TRAIL_LEN * 3);
  private n = 0;
  constructor(color: THREE.Color) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(this.pos, 3)); g.setDrawRange(0, 0);
    this.line = new THREE.Line(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.85 }));
    this.line.frustumCulled = false;
  }
  push(p: THREE.Vector3) {
    if (this.n === TRAIL_LEN) { this.pos.copyWithin(0, 3); this.n--; }
    this.pos.set([p.x, p.y, p.z], this.n * 3); this.n++;
    (this.line.geometry.getAttribute("position") as THREE.BufferAttribute).needsUpdate = true;
    this.line.geometry.setDrawRange(0, this.n);
  }
  clear() { this.n = 0; this.line.geometry.setDrawRange(0, 0); }
}

function canvasSprite(draw: (ctx: CanvasRenderingContext2D, w: number, h: number) => void, w: number, h: number, scale: [number, number]): THREE.Sprite {
  const c = document.createElement("canvas"); c.width = w; c.height = h;
  draw(c.getContext("2d")!, w, h);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false, transparent: true, sizeAttenuation: false }));
  s.scale.set(scale[0], scale[1], 1);
  return s;
}
const makeLabel = (text: string, color: string) => canvasSprite((ctx, _w, _h) => {
  ctx.font = "600 26px ui-monospace, Menlo, monospace";
  const tw = ctx.measureText(text).width + 22;
  ctx.fillStyle = "rgba(9,12,18,0.78)"; ctx.beginPath(); (ctx as any).roundRect(0, 10, tw, 44, 6); ctx.fill();
  ctx.fillStyle = color; ctx.fillText(text, 11, 41);
}, 256, 64, [0.085, 0.021]);
const makeDot = (color: THREE.Color) => canvasSprite((ctx) => {
  ctx.beginPath(); ctx.arc(16, 16, 12, 0, Math.PI * 2);
  ctx.fillStyle = `rgb(${color.r * 255},${color.g * 255},${color.b * 255})`; ctx.fill();
  ctx.lineWidth = 3; ctx.strokeStyle = "rgba(255,255,255,0.95)"; ctx.stroke();
}, 32, 32, [0.017, 0.017]);

export class WorldScene {
  readonly scene = new THREE.Scene();
  readonly models = new Map<string, THREE.Group>();
  readonly meta = new Map<string, ModelDesc>();
  readonly agentColors = new Map<string, THREE.Color>();
  private trails = new Map<string, Trail>();
  private planned = new Map<string, THREE.Line>();
  private arrows = new Map<string, THREE.ArrowHelper>();
  private labels = new Map<string, THREE.Sprite>();
  private markers = new Map<string, THREE.Sprite>();
  private axes = new Map<string, THREE.AxesHelper>();
  private frustums = new Map<string, THREE.LineSegments>();
  private collisionMarks: { m: THREE.Mesh; t: number }[] = [];
  private collisionGeoms: THREE.Object3D[] = [];
  private boundsBox: THREE.LineSegments | null = null;
  private areaObjs: THREE.Object3D[] = [];
  private grid: THREE.GridHelper;
  private sun: THREE.DirectionalLight;
  private hemi: THREE.HemisphereLight;
  private selectedId: string | null = null;
  private selBox: THREE.BoxHelper | null = null;
  private taskObjs: THREE.Object3D[] = [];
  overlays: Overlays = { trails: true, planned: true, velocity: true, axes: false, labels: true, collisions: true, frustum: false, bounds: false, collisionGeom: false, grid: false, areas: false };

  constructor() {
    this.hemi = new THREE.HemisphereLight(0xbfd4ff, 0x3a3a36, 0.9);
    this.scene.add(this.hemi);
    this.sun = new THREE.DirectionalLight(0xfff1d6, 2.2);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(4096, 4096);
    const c = this.sun.shadow.camera as THREE.OrthographicCamera;
    c.left = c.bottom = -220; c.right = c.top = 220; c.near = 1; c.far = 700;
    this.sun.shadow.bias = -0.0005;
    this.scene.add(this.sun); this.scene.add(this.sun.target);
    this.grid = new THREE.GridHelper(600, 120, 0x35404f, 0x212a36);
    this.grid.rotation.x = Math.PI / 2; this.grid.position.z = 0.02; this.grid.visible = false;
    this.scene.add(this.grid);
    this.applyEnvironment(null);
  }

  /** Match the world's lighting preset: sun direction/colour, ambient, sky colour and fog. */
  applyEnvironment(env: Environment | null | undefined) {
    const e = env ?? { preset: "day", hour: 13, sun_direction: [-0.4, 0.3, -0.85], sun_color: [0.95, 0.93, 0.88], ambient: [0.45, 0.45, 0.5], background: [0.68, 0.78, 0.9], fog_color: [0.72, 0.8, 0.9], fog_density: 0.0006, wind: [0, 0, 0], visibility_m: null };
    const bg = new THREE.Color(e.background[0], e.background[1], e.background[2]);
    this.scene.background = bg;
    const fog = new THREE.Color(e.fog_color[0], e.fog_color[1], e.fog_color[2]);
    this.scene.fog = new THREE.FogExp2(fog, Math.max(0.0002, e.fog_density * 1.3));
    const d = new THREE.Vector3(e.sun_direction[0], e.sun_direction[1], e.sun_direction[2]).normalize();
    this.sun.position.copy(d.clone().multiplyScalar(-300));
    this.sun.target.position.set(0, 0, 0);
    this.sun.color.setRGB(e.sun_color[0], e.sun_color[1], e.sun_color[2]);
    const night = e.preset === "night";
    this.sun.intensity = night ? 0.5 : e.preset === "evening" ? 1.6 : 2.4;
    this.hemi.color.setRGB(e.ambient[0] * 1.6, e.ambient[1] * 1.6, e.ambient[2] * 1.7);
    this.hemi.groundColor.setRGB(e.ambient[0] * 0.6, e.ambient[1] * 0.55, e.ambient[2] * 0.5);
    this.hemi.intensity = night ? 0.5 : 1.0;
  }

  rebuild(desc: SceneDesc, agents: AgentInfo[]) {
    for (const g of this.models.values()) this.scene.remove(g);
    for (const t of this.trails.values()) this.scene.remove(t.line);
    for (const l of this.planned.values()) this.scene.remove(l);
    for (const a of this.arrows.values()) this.scene.remove(a);
    for (const o of this.areaObjs) this.scene.remove(o);
    this.models.clear(); this.meta.clear(); this.trails.clear(); this.planned.clear(); this.arrows.clear();
    this.labels.clear(); this.markers.clear(); this.axes.clear(); this.frustums.clear(); this.collisionGeoms = []; this.areaObjs = [];
    const info = new Map(agents.map(a => [a.agent_id, a]));
    for (const m of desc.models) this.addModel(m, info.get(m.entity_id));
    this.applyEnvironment(desc.environment);
    if (this.boundsBox) { this.scene.remove(this.boundsBox); this.boundsBox = null; }
    if (desc.bounds) {
      const b = desc.bounds; const sx = b.x[1] - b.x[0], sy = b.y[1] - b.y[0], sz = b.z[1] - b.z[0];
      this.boundsBox = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(sx, sy, sz)),
        new THREE.LineDashedMaterial({ color: 0xffaa33, dashSize: 4, gapSize: 3, transparent: true, opacity: 0.5 }));
      this.boundsBox.computeLineDistances(); this.boundsBox.position.set(b.x[0] + sx / 2, b.y[0] + sy / 2, b.z[0] + sz / 2);
      this.scene.add(this.boundsBox);
    }
    for (const a of desc.areas ?? []) {
      const w = a.x[1] - a.x[0], d = a.y[1] - a.y[0];
      const geo = new THREE.EdgesGeometry(new THREE.PlaneGeometry(w, d));
      const line = new THREE.LineSegments(geo, new THREE.LineDashedMaterial({ color: 0x2ab1ff, dashSize: 3, gapSize: 2, transparent: true, opacity: 0.6 }));
      line.computeLineDistances(); line.position.set(a.x[0] + w / 2, a.y[0] + d / 2, 0.3);
      const lbl = makeLabel(a.name.toUpperCase(), "#2ab1ff"); lbl.position.set(a.x[0] + 2, a.y[1] - 2, 0.5);
      this.areaObjs.push(line, lbl); this.scene.add(line); this.scene.add(lbl);
    }
    this.applyOverlays();
    if (this.selectedId) this.select(this.selectedId);
  }

  addModel(m: ModelDesc, info?: AgentInfo) {
    const group = new THREE.Group(); group.name = m.entity_id;
    applyPose(group, m.pose);
    let mainColor: THREE.Color | null = null;
    for (const link of m.links) {
      const lg = new THREE.Group(); applyPose(lg, link.pose);
      for (const v of link.visuals) {
        const mesh = makeVisual(v, m);
        if (mesh) { lg.add(mesh); if (!mainColor && v.color) mainColor = new THREE.Color(v.color[0], v.color[1], v.color[2]); }
      }
      group.add(lg);
    }
    if (m.category !== "terrain") {
      const box = new THREE.Box3().setFromObject(group);
      const size = box.getSize(new THREE.Vector3()); const center = box.getCenter(new THREE.Vector3());
      const wire = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(size.x, size.y, size.z)),
        new THREE.LineBasicMaterial({ color: m.is_agent ? 0xff5555 : 0x44ff88, transparent: true, opacity: 0.6 }));
      wire.position.copy(center).sub(group.position); wire.visible = this.overlays.collisionGeom;
      group.add(wire); this.collisionGeoms.push(wire);
    }
    this.scene.add(group); this.models.set(m.entity_id, group); this.meta.set(m.entity_id, m);

    const dyn = m.is_agent || ["target", "vehicle", "dynamic"].includes(m.category);
    if (dyn) {
      const color = mainColor ?? new THREE.Color(0xff8a2a);
      this.agentColors.set(m.entity_id, color);
      const trail = new Trail(color); this.trails.set(m.entity_id, trail); this.scene.add(trail.line);
      const dot = makeDot(color); dot.position.set(0, 0, 0.7); group.add(dot); this.markers.set(m.entity_id, dot);
      const label = makeLabel(m.entity_id, "#" + color.getHexString()); label.position.set(0, 0, 0.95);
      group.add(label); this.labels.set(m.entity_id, label);
      if (m.is_agent) {
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0x2ab1ff, 0.3, 0.15);
        arrow.visible = false; this.scene.add(arrow); this.arrows.set(m.entity_id, arrow);
        const ax = new THREE.AxesHelper(0.8); group.add(ax); this.axes.set(m.entity_id, ax);
        for (const cam of info?.observation_space.frames ?? []) {
          if (cam.type !== "rgb") continue;
          const fr = makeFrustum(cam.hfov, cam.width / cam.height, cam.pose ?? [0.12, 0, -0.02, 0, 0.35, 0]);
          group.add(fr); this.frustums.set(`${m.entity_id}/${cam.name}`, fr);
        }
        const pl = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
          new THREE.LineDashedMaterial({ color: 0xffd166, dashSize: 0.8, gapSize: 0.5, transparent: true, opacity: 0.9 }));
        pl.visible = false; pl.frustumCulled = false; this.scene.add(pl); this.planned.set(m.entity_id, pl);
      }
    }
  }

  updatePoses(poses: Record<string, number[]>, vel: Record<string, number[]>, wp?: Record<string, number[]>) {
    for (const [id, p] of Object.entries(poses)) {
      const g = this.models.get(id); if (!g) continue;
      g.position.set(p[0], p[1], p[2]); g.quaternion.set(p[3], p[4], p[5], p[6]);
      const t = this.trails.get(id); if (t && this.overlays.trails) t.push(g.position);
      const a = this.arrows.get(id); const v = vel[id];
      if (a && v) {
        const len = Math.hypot(v[0], v[1], v[2]);
        a.visible = this.overlays.velocity && len > 0.05;
        if (a.visible) { a.position.copy(g.position); a.setDirection(new THREE.Vector3(v[0], v[1], v[2]).normalize()); a.setLength(Math.min(len, 8), 0.3, 0.15); }
      }
    }
    for (const [id, line] of this.planned) {
      const target = wp?.[id]; const g = this.models.get(id);
      if (target && g && this.overlays.planned) {
        line.visible = true;
        line.geometry.setFromPoints([g.position.clone(), new THREE.Vector3(target[0], target[1], target[2])]);
        line.computeLineDistances();
      } else line.visible = false;
    }
    this.selBox?.update();
    const now = performance.now();
    this.collisionMarks = this.collisionMarks.filter(({ m, t }) => {
      const age = (now - t) / 1000;
      if (age > 4) { this.scene.remove(m); return false; }
      (m.material as THREE.MeshBasicMaterial).opacity = 0.9 * (1 - age / 4); m.scale.setScalar(1 + age * 0.5);
      return true;
    });
  }

  markCollision(p: { x: number; y: number; z: number } | null) {
    if (!p || !this.overlays.collisions) return;
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.6, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff3b3b, transparent: true, opacity: 0.9 }));
    m.position.set(p.x, p.y, p.z); this.scene.add(m); this.collisionMarks.push({ m, t: performance.now() });
  }

  clearTrails() { for (const t of this.trails.values()) t.clear(); }

  applyOverlays() {
    const o = this.overlays;
    for (const t of this.trails.values()) t.line.visible = o.trails;
    for (const l of this.labels.values()) l.visible = o.labels;
    for (const d of this.markers.values()) d.visible = o.labels;
    for (const a of this.axes.values()) a.visible = o.axes;
    for (const f of this.frustums.values()) f.visible = o.frustum;
    for (const c of this.collisionGeoms) c.visible = o.collisionGeom;
    for (const a of this.areaObjs) a.visible = o.areas;
    this.grid.visible = o.grid;
    if (this.boundsBox) this.boundsBox.visible = o.bounds;
    if (!o.velocity) for (const a of this.arrows.values()) a.visible = false;
    if (!o.planned) for (const l of this.planned.values()) l.visible = false;
  }

  // ---- task overlay (target ring, start marker, drone->target line) ----------------
  setTaskOverlay(agentId: string | null, target: number[] | null, start: number[] | null, radius = 1.0, status = "running") {
    for (const o of this.taskObjs) this.scene.remove(o);
    this.taskObjs = [];
    if (!target) return;
    const color = status === "reached" ? 0x2ecc71 : status === "running" ? 0xffd166 : 0xff3b3b;
    const ring = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.06, 8, 48), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }));
    ring.position.set(target[0], target[1], target[2]); ring.rotation.x = Math.PI / 2;
    const ring2 = ring.clone(); ring2.rotation.set(0, 0, 0);
    const beam = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, target[2], 8), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.35 }));
    beam.position.set(target[0], target[1], target[2] / 2); beam.rotation.x = Math.PI / 2;
    this.taskObjs.push(ring, ring2, beam);
    if (start) {
      const sm = new THREE.Mesh(new THREE.RingGeometry(0.6, 0.8, 32), new THREE.MeshBasicMaterial({ color: 0x2ab1ff, side: THREE.DoubleSide, transparent: true, opacity: 0.8 }));
      sm.position.set(start[0], start[1], 0.08); this.taskObjs.push(sm);
    }
    const g = agentId ? this.models.get(agentId) : undefined;
    if (g) {
      const geo = new THREE.BufferGeometry().setFromPoints([g.position.clone(), new THREE.Vector3(target[0], target[1], target[2])]);
      const line = new THREE.Line(geo, new THREE.LineDashedMaterial({ color, dashSize: 0.5, gapSize: 0.3, transparent: true, opacity: 0.7 }));
      line.computeLineDistances(); this.taskObjs.push(line);
    }
    for (const o of this.taskObjs) this.scene.add(o);
  }

  select(id: string | null) {
    this.selectedId = id;
    if (this.selBox) { this.scene.remove(this.selBox); this.selBox = null; }
    const g = id ? this.models.get(id) : undefined;
    if (g) { this.selBox = new THREE.BoxHelper(g, 0x2ab1ff); this.scene.add(this.selBox); }
  }

  pick(raycaster: THREE.Raycaster): string | null {
    const hits = raycaster.intersectObjects([...this.models.values()], true);
    for (const h of hits) {
      let o: THREE.Object3D | null = h.object;
      while (o && !this.models.has(o.name)) o = o.parent;
      if (o && this.meta.get(o.name)?.category !== "terrain") return o.name;
    }
    return null;
  }

  /** Bounding box of the dynamic entities (agents, vehicles, targets) or, if none, everything. */
  focusBox(): THREE.Box3 | null {
    const box = new THREE.Box3(); let any = false;
    for (const m of this.meta.values()) {
      if (!(m.is_agent || ["target", "vehicle", "dynamic"].includes(m.category))) continue;
      const g = this.models.get(m.entity_id); if (g) { box.expandByPoint(g.position); any = true; }
    }
    if (!any) for (const g of this.models.values()) { box.expandByObject(g); any = true; }
    return any ? box : null;
  }
}

function applyPose(o: THREE.Object3D, p: Pose) {
  o.position.set(p.position.x, p.position.y, p.position.z);
  o.quaternion.set(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
}

export function rpyToQuat(r: number, p: number, y: number): THREE.Quaternion {
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, "ZYX"));
}

function makeFrustum(hfov: number, aspect: number, pose: number[]): THREE.LineSegments {
  const d = 3; const hw = Math.tan(hfov / 2) * d; const hh = hw / aspect;
  const pts: number[] = []; const o = [0, 0, 0];
  const corners = [[d, hw, hh], [d, -hw, hh], [d, -hw, -hh], [d, hw, -hh]];
  for (const c of corners) pts.push(...o, ...c);
  for (let i = 0; i < 4; i++) pts.push(...corners[i], ...corners[(i + 1) % 4]);
  const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  const ls = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.7 }));
  ls.position.set(pose[0], pose[1], pose[2]); ls.quaternion.copy(rpyToQuat(pose[3], pose[4], pose[5]));
  return ls;
}

function makeVisual(v: Visual, m: ModelDesc): THREE.Object3D | null {
  const g = v.geometry;
  let geom: THREE.BufferGeometry;
  switch (g.type) {
    case "box": geom = new THREE.BoxGeometry(g.size!.x, g.size!.y, g.size!.z); break;
    case "cylinder": geom = new THREE.CylinderGeometry(g.radius!, g.radius!, g.length!, 40).rotateX(Math.PI / 2); break;
    case "sphere": geom = new THREE.SphereGeometry(g.radius!, 24, 16); break;
    case "plane": geom = new THREE.PlaneGeometry(g.size?.x || 100, g.size?.y || 100); break;
    default: geom = new THREE.BoxGeometry(0.2, 0.2, 0.2);
  }
  const c = v.color ?? [0.6, 0.6, 0.6, 1];
  const color = new THREE.Color(c[0], c[1], c[2]);
  const dyn = m.is_agent || ["target", "vehicle", "dynamic"].includes(m.category);
  const rough = m.category === "building" ? 0.6 : m.category === "terrain" ? 0.95 : dyn ? 0.45 : 0.8;
  const mat = new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: dyn ? 0.25 : m.category === "building" ? 0.15 : 0.05, transparent: c[3] < 1, opacity: c[3] });
  if (dyn) mat.emissive = color.clone().multiplyScalar(0.12);
  if (EMISSIVE_NAMES.some(n => v.name.startsWith(n)) && (c[0] + c[1] + c[2]) > 1.2) mat.emissive = color.clone().multiplyScalar(0.55);
  const mesh = new THREE.Mesh(geom, mat);
  mesh.castShadow = g.type !== "plane" && m.category !== "terrain";
  mesh.receiveShadow = true;
  applyPose(mesh, v.pose);
  return mesh;
}
