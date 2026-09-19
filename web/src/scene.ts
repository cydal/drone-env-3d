// Builds/updates a three.js scene from SceneDesc. Gazebo is Z-up, so we keep
// Gazebo coordinates verbatim and tell three.js that up is +Z.
import * as THREE from "three";
import type { ModelDesc, Pose, SceneDesc, Visual } from "./api";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export class WorldScene {
  readonly scene = new THREE.Scene();
  readonly models = new Map<string, THREE.Group>();
  readonly meta = new Map<string, ModelDesc>();
  private selectedId: string | null = null;
  private selBox: THREE.BoxHelper | null = null;

  constructor() {
    this.scene.background = new THREE.Color(0x0b0e13);
    this.scene.fog = new THREE.Fog(0x0b0e13, 180, 420);
    const hemi = new THREE.HemisphereLight(0xbfd4ff, 0x2a2622, 0.9);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xfff1d6, 2.2);
    sun.position.set(40, -30, 85);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const c = sun.shadow.camera as THREE.OrthographicCamera;
    c.left = c.bottom = -120; c.right = c.top = 120; c.far = 300;
    this.scene.add(sun);
    const grid = new THREE.GridHelper(400, 80, 0x2a3442, 0x1a2230);
    grid.rotation.x = Math.PI / 2; grid.position.z = 0.01;
    this.scene.add(grid);
  }

  rebuild(desc: SceneDesc) {
    for (const g of this.models.values()) this.scene.remove(g);
    this.models.clear(); this.meta.clear();
    for (const m of desc.models) this.addModel(m);
    if (this.selectedId) this.select(this.selectedId);
  }

  addModel(m: ModelDesc) {
    const group = new THREE.Group();
    group.name = m.entity_id;
    applyPose(group, m.pose);
    for (const link of m.links) {
      const lg = new THREE.Group();
      applyPose(lg, link.pose);
      for (const v of link.visuals) {
        const mesh = makeVisual(v, m);
        if (mesh) lg.add(mesh);
      }
      group.add(lg);
    }
    this.scene.add(group);
    this.models.set(m.entity_id, group);
    this.meta.set(m.entity_id, m);
  }

  updatePoses(poses: Record<string, number[]>) {
    for (const [id, p] of Object.entries(poses)) {
      const g = this.models.get(id);
      if (!g) continue;
      g.position.set(p[0], p[1], p[2]);
      g.quaternion.set(p[3], p[4], p[5], p[6]);
    }
    this.selBox?.update();
  }

  select(id: string | null) {
    this.selectedId = id;
    if (this.selBox) { this.scene.remove(this.selBox); this.selBox = null; }
    const g = id ? this.models.get(id) : undefined;
    if (g) {
      this.selBox = new THREE.BoxHelper(g, 0x2ab1ff);
      this.scene.add(this.selBox);
    }
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

function makeVisual(v: Visual, m: ModelDesc): THREE.Object3D | null {
  const g = v.geometry;
  let geom: THREE.BufferGeometry | null = null;
  switch (g.type) {
    case "box": geom = new THREE.BoxGeometry(g.size!.x, g.size!.y, g.size!.z); break;
    case "cylinder": geom = new THREE.CylinderGeometry(g.radius!, g.radius!, g.length!, 32).rotateX(Math.PI / 2); break;
    case "sphere": geom = new THREE.SphereGeometry(g.radius!, 24, 16); break;
    case "plane": geom = new THREE.PlaneGeometry(g.size?.x || 100, g.size?.y || 100); break;
    default: geom = new THREE.BoxGeometry(0.2, 0.2, 0.2);
  }
  const c = v.color ?? [0.6, 0.6, 0.6, 1];
  const color = new THREE.Color(c[0], c[1], c[2]);
  const mat = new THREE.MeshStandardMaterial({
    color, roughness: m.is_agent ? 0.45 : 0.85, metalness: m.is_agent ? 0.25 : 0.05,
    transparent: c[3] < 1, opacity: c[3],
  });
  if (m.is_agent) mat.emissive = color.clone().multiplyScalar(0.12);
  const mesh = new THREE.Mesh(geom, mat);
  mesh.castShadow = g.type !== "plane";
  mesh.receiveShadow = true;
  applyPose(mesh, v.pose);
  return mesh;
}
