// Builds/updates a three.js scene from SceneDesc plus debug overlays.
// Gazebo is Z-up: we keep Gazebo coordinates verbatim and set up = +Z.
import * as THREE from "three";
import type { AgentInfo, ModelDesc, Pose, SceneDesc, Visual } from "./api";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export interface Overlays { trails: boolean; velocity: boolean; axes: boolean; labels: boolean; collisions: boolean; frustum: boolean; bounds: boolean; collisionGeom: boolean }

const TRAIL_LEN = 900;   // ~36 s at 25 Hz

class Trail {
  readonly line: THREE.Line;
  private pos: Float32Array = new Float32Array(TRAIL_LEN * 3);
  private n = 0; private head = 0;
  constructor(color: THREE.Color) {
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(this.pos, 3));
    g.setDrawRange(0, 0);
    this.line = new THREE.Line(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.8 }));
    this.line.frustumCulled = false;
  }
  push(p: THREE.Vector3) {
    // keep chronological order by shifting when full (cheap enough at 25 Hz)
    if (this.n === TRAIL_LEN) { this.pos.copyWithin(0, 3); this.n--; }
    this.pos.set([p.x, p.y, p.z], this.n * 3); this.n++;
    const attr = this.line.geometry.getAttribute("position") as THREE.BufferAttribute;
    attr.needsUpdate = true;
    this.line.geometry.setDrawRange(0, this.n);
  }
  clear() { this.n = 0; this.line.geometry.setDrawRange(0, 0); }
}

function makeLabel(text: string, color: string): THREE.Sprite {
  const c = document.createElement("canvas"); c.width = 256; c.height = 64;
  const ctx = c.getContext("2d")!;
  ctx.font = "bold 28px ui-monospace, Menlo, monospace";
  ctx.fillStyle = "rgba(11,14,19,0.75)"; ctx.fillRect(0, 8, ctx.measureText(text).width + 24, 48);
  ctx.fillStyle = color; ctx.fillText(text, 12, 42);
  const tex = new THREE.CanvasTexture(c);
  // sizeAttenuation: false keeps the label (and the dot below) a constant, readable size on
  // screen no matter how far the agent is -- true-scale drones (~0.2 m) would otherwise
  // shrink to sub-pixel specks a few tens of metres out.
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true, sizeAttenuation: false }));
  s.scale.set(0.09, 0.0225, 1); s.center.set(0, 0.5);
  return s;
}

function makeDot(color: THREE.Color): THREE.Sprite {
  const c = document.createElement("canvas"); c.width = 32; c.height = 32;
  const ctx = c.getContext("2d")!;
  ctx.beginPath(); ctx.arc(16, 16, 13, 0, Math.PI * 2);
  ctx.fillStyle = `rgba(${color.r * 255}, ${color.g * 255}, ${color.b * 255}, 1)`; ctx.fill();
  ctx.lineWidth = 3; ctx.strokeStyle = "rgba(255,255,255,0.9)"; ctx.stroke();
  const tex = new THREE.CanvasTexture(c);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true, sizeAttenuation: false }));
  s.scale.set(0.018, 0.018, 1);
  return s;
}

export class WorldScene {
  readonly scene = new THREE.Scene();
  readonly models = new Map<string, THREE.Group>();
  readonly meta = new Map<string, ModelDesc>();
  readonly agentColors = new Map<string, THREE.Color>();
  private trails = new Map<string, Trail>();
  private arrows = new Map<string, THREE.ArrowHelper>();
  private labels = new Map<string, THREE.Sprite>();
  private markers = new Map<string, THREE.Sprite>();
  private axes = new Map<string, THREE.AxesHelper>();
  private frustums = new Map<string, THREE.LineSegments>();
  private collisionMarks: { m: THREE.Mesh; t: number }[] = [];
  private collisionGeoms: THREE.Object3D[] = [];
  private boundsBox: THREE.LineSegments | null = null;
  private selectedId: string | null = null;
  private selBox: THREE.BoxHelper | null = null;
  overlays: Overlays = { trails: true, velocity: true, axes: false, labels: true, collisions: true, frustum: false, bounds: false, collisionGeom: false };

  constructor() {
    this.scene.background = new THREE.Color(0x0b0e13);
    this.scene.fog = new THREE.Fog(0x0b0e13, 180, 420);
    this.scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x2a2622, 0.9));
    const sun = new THREE.DirectionalLight(0xfff1d6, 2.2);
    sun.position.set(40, -30, 85); sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const c = sun.shadow.camera as THREE.OrthographicCamera;
    c.left = c.bottom = -120; c.right = c.top = 120; c.far = 300;
    this.scene.add(sun);
    const grid = new THREE.GridHelper(400, 80, 0x2a3442, 0x1a2230);
    grid.rotation.x = Math.PI / 2; grid.position.z = 0.01;
    this.scene.add(grid);
  }

  rebuild(desc: SceneDesc, agents: AgentInfo[]) {
    for (const g of this.models.values()) this.scene.remove(g);
    for (const t of this.trails.values()) this.scene.remove(t.line);
    for (const a of this.arrows.values()) this.scene.remove(a);
    for (const l of this.labels.values()) this.scene.remove(l);
    for (const d of this.markers.values()) this.scene.remove(d);
    for (const f of this.frustums.values()) this.scene.remove(f);
    for (const c of this.collisionGeoms) this.scene.remove(c);
    this.models.clear(); this.meta.clear(); this.trails.clear(); this.arrows.clear(); this.labels.clear(); this.markers.clear(); this.axes.clear(); this.frustums.clear(); this.collisionGeoms = [];
    const info = new Map(agents.map(a => [a.agent_id, a]));
    for (const m of desc.models) this.addModel(m, info.get(m.entity_id));
    if (this.boundsBox) { this.scene.remove(this.boundsBox); this.boundsBox = null; }
    if (desc.bounds) {
      const b = desc.bounds; const sx = b.x[1] - b.x[0], sy = b.y[1] - b.y[0], sz = b.z[1] - b.z[0];
      const geo = new THREE.EdgesGeometry(new THREE.BoxGeometry(sx, sy, sz));
      this.boundsBox = new THREE.LineSegments(geo, new THREE.LineDashedMaterial({ color: 0xffaa33, dashSize: 3, gapSize: 2, transparent: true, opacity: 0.5 }));
      this.boundsBox.computeLineDistances();
      this.boundsBox.position.set(b.x[0] + sx / 2, b.y[0] + sy / 2, b.z[0] + sz / 2);
      this.scene.add(this.boundsBox);
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
    // collision geometry approximation: bounding boxes of the visuals, drawn as wireframes
    if (m.entity_id !== "ground") {
      const box = new THREE.Box3().setFromObject(group);
      const size = box.getSize(new THREE.Vector3()); const center = box.getCenter(new THREE.Vector3());
      const wire = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(size.x, size.y, size.z)),
        new THREE.LineBasicMaterial({ color: m.is_agent ? 0xff5555 : 0x44ff88, transparent: true, opacity: 0.6 }));
      wire.position.copy(center).sub(group.position); wire.visible = this.overlays.collisionGeom;
      group.add(wire); this.collisionGeoms.push(wire);
    }
    this.scene.add(group); this.models.set(m.entity_id, group); this.meta.set(m.entity_id, m);

    if (m.is_agent || m.kind === "target" || m.kind === "vehicle") {
      const color = mainColor ?? new THREE.Color(0xff8a2a);
      this.agentColors.set(m.entity_id, color);
      const trail = new Trail(color); this.trails.set(m.entity_id, trail); this.scene.add(trail.line);
      const dot = makeDot(color); dot.position.set(0, 0, 0.6); group.add(dot); this.markers.set(m.entity_id, dot);
      const label = makeLabel(m.entity_id, "#" + color.getHexString()); label.position.set(0, 0, 0.85);
      group.add(label); this.labels.set(m.entity_id, label);
      if (m.is_agent) {
        const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0x2ab1ff, 0.3, 0.15);
        arrow.visible = false; this.scene.add(arrow); this.arrows.set(m.entity_id, arrow);
        const ax = new THREE.AxesHelper(0.8); group.add(ax); this.axes.set(m.entity_id, ax);
        const cam = info?.observation_space.frames.find(f => f.type === "rgb");
        if (cam) { const fr = makeFrustum(cam.hfov, cam.width / cam.height); group.add(fr); this.frustums.set(m.entity_id, fr); }
      }
    }
  }

  updatePoses(poses: Record<string, number[]>, vel: Record<string, number[]>) {
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
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.5, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff3b3b, transparent: true, opacity: 0.9 }));
    m.position.set(p.x, p.y, p.z); this.scene.add(m); this.collisionMarks.push({ m, t: performance.now() });
  }

  clearTrails() { for (const t of this.trails.values()) t.clear(); }

  applyOverlays() {
    const o = this.overlays;
    for (const t of this.trails.values()) t.line.visible = o.trails;
    for (const l of this.labels.values()) l.visible = o.labels;
    for (const a of this.axes.values()) a.visible = o.axes;
    for (const f of this.frustums.values()) f.visible = o.frustum;
    for (const c of this.collisionGeoms) c.visible = o.collisionGeom;
    if (this.boundsBox) this.boundsBox.visible = o.bounds;
    if (!o.velocity) for (const a of this.arrows.values()) a.visible = false;
  }

  // ---- task overlay: target ring, start marker, drone->target line ----------------
  private taskObjs: THREE.Object3D[] = [];
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
      sm.position.set(start[0], start[1], 0.05); this.taskObjs.push(sm);
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
      if (o && o.name !== "ground") return o.name;
    }
    return null;
  }
}

function applyPose(o: THREE.Object3D, p: Pose) {
  o.position.set(p.position.x, p.position.y, p.position.z);
  o.quaternion.set(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
}

function makeFrustum(hfov: number, aspect: number): THREE.LineSegments {
  // camera sits at (0.12, 0, -0.02) pitched 0.35 rad down, looking along +x (Gazebo optical convention)
  const d = 3; const hw = Math.tan(hfov / 2) * d; const hh = hw / aspect;
  const pts: number[] = []; const o = [0, 0, 0];
  const corners = [[d, hw, hh], [d, -hw, hh], [d, -hw, -hh], [d, hw, -hh]];
  for (const c of corners) pts.push(...o, ...c);
  for (let i = 0; i < 4; i++) pts.push(...corners[i], ...corners[(i + 1) % 4]);
  const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  const ls = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.7 }));
  ls.position.set(0.12, 0, -0.02); ls.rotation.y = 0.35;
  return ls;
}

function makeVisual(v: Visual, m: ModelDesc): THREE.Object3D | null {
  const g = v.geometry;
  let geom: THREE.BufferGeometry;
  switch (g.type) {
    case "box": geom = new THREE.BoxGeometry(g.size!.x, g.size!.y, g.size!.z); break;
    case "cylinder": geom = new THREE.CylinderGeometry(g.radius!, g.radius!, g.length!, 32).rotateX(Math.PI / 2); break;
    case "sphere": geom = new THREE.SphereGeometry(g.radius!, 24, 16); break;
    case "plane": geom = new THREE.PlaneGeometry(g.size?.x || 100, g.size?.y || 100); break;
    default: geom = new THREE.BoxGeometry(0.2, 0.2, 0.2);
  }
  const c = v.color ?? [0.6, 0.6, 0.6, 1];
  const color = new THREE.Color(c[0], c[1], c[2]);
  const dyn = m.is_agent || m.kind === "target" || m.kind === "vehicle";
  const mat = new THREE.MeshStandardMaterial({ color, roughness: dyn ? 0.45 : 0.85, metalness: dyn ? 0.25 : 0.05, transparent: c[3] < 1, opacity: c[3] });
  if (dyn) mat.emissive = color.clone().multiplyScalar(0.15);
  const mesh = new THREE.Mesh(geom, mat);
  mesh.castShadow = g.type !== "plane"; mesh.receiveShadow = true;
  applyPose(mesh, v.pose);
  return mesh;
}
