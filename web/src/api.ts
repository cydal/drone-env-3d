// Simulation API client for the browser. HTTP for commands, WS for telemetry.
// The browser uses exactly the same public operations as external controllers.
export const API = (import.meta as any).env?.VITE_SIMAPI ?? "http://127.0.0.1:8000";
export const WS_BASE = API.replace(/^http/, "ws");

export interface Vec3 { x: number; y: number; z: number }
export interface Quat { x: number; y: number; z: number; w: number }
export interface Pose { position: Vec3; orientation: Quat }
export interface Geometry { type: string; size?: Vec3; radius?: number; length?: number; normal?: Vec3; uri?: string; scale?: Vec3 }
export interface Visual { name: string; pose: Pose; geometry: Geometry; color?: number[] | null }
export interface LinkDesc { name: string; pose: Pose; visuals: Visual[] }
export interface ModelDesc { entity_id: string; kind: string; category: string; is_agent: boolean; pose: Pose; links: LinkDesc[]; is_static: boolean }
export interface Environment { preset: string; hour: number; sun_direction: number[]; sun_color: number[]; ambient: number[]; background: number[]; fog_color: number[]; fog_density: number; wind: number[]; visibility_m: number | null }
export interface SceneDesc { world: string; models: ModelDesc[]; bounds?: Record<string, number[]> | null; environment?: Environment | null; areas?: { name: string; x: number[]; y: number[] }[] }
export interface Episode { episode_id: string; scenario_id: string; seed: number; mode: string; status: string; sim_time: number; step_count: number; iterations: number; randomization?: Record<string, any> }
export interface SimStatus { running: boolean; paused: boolean; mode: "realtime" | "stepped"; speed: number; sim_time: number; real_time_factor: number; iterations: number; scenario?: string; seed?: number; step_size?: number; episode?: Episode | null }
export interface FrameDesc { name: string; type: "rgb" | "depth"; width: number; height: number; hfov: number; rate_hz: number; pose?: number[] }
export interface AgentInfo {
  agent_id: string; agent_type: string; template: string; control_mode: string; sensors: string[];
  observation_space: { profile: string; components: string[]; frames: FrameDesc[] };
  action_space: { level: string; types: string[]; limits: Record<string, number> };
}
export interface EntityDetail {
  entity_id: string; kind: string; category: string; is_agent: boolean; is_static: boolean; template?: string; type_label?: string;
  dimensions?: Vec3 | null; collision: boolean; trajectory?: string | null; control?: string | null; sensors: string[];
  sensor_mounts: any[]; physical: Record<string, any>; state?: any;
}
export interface SimEvent { seq: number; event: string; sim_time: number; entities: string[]; data: any }
export interface Metrics { real_time_factor: number; sim_hz: number; step_size: number; entity_count: number; agent_count: number; telemetry_clients: number; sensor_clients: number; api_step_latency_ms: number | null; sensor_fps: Record<string, number>; events_total: number }
export interface RecordingMeta { recording_id: string; episode_id: string; scenario: string; seed: number; mode: string; rows: number; actions: number; final_iteration: number; status: string; created: number }
export interface SnapshotMeta { snapshot_id: string; name: string; created: number; episode_id: string; scenario: string; seed: number; sim_time: number; iteration: number }

async function j<T>(path: string, init?: RequestInit): Promise<T> {
  const r = await fetch(API + path, init);
  if (!r.ok) throw new Error(`${path}: ${r.status} ${await r.text()}`);
  return r.json();
}
const post = (p: string, body?: unknown) => j<any>(p, { method: "POST", headers: { "content-type": "application/json" }, body: body !== undefined ? JSON.stringify(body) : undefined });

export const api = {
  status: () => j<SimStatus>("/status"),
  metrics: () => j<Metrics>("/metrics"),
  scenarios: () => j<string[]>("/scenarios"),
  load: (name: string, mode?: string) => post(`/simulation/load/${name}` + (mode ? `?mode=${mode}` : "")),
  scene: () => j<SceneDesc>("/scene"),
  agents: () => j<AgentInfo[]>("/agents"),
  entity: (id: string) => j<any>(`/entities/${id}`),
  detail: (id: string) => j<EntityDetail>(`/entities/${id}/detail`),
  observation: (id: string) => j<any>(`/agents/${id}/observation`),
  events: (since = 0) => j<SimEvent[]>(`/events?since=${since}`),
  pause: () => post("/simulation/pause"),
  resume: () => post("/simulation/resume"),
  mode: (mode: string) => post("/simulation/mode", { mode }),
  speed: (rtf: number) => post("/simulation/speed", { real_time_factor: rtf }),
  step: (n = 1) => post("/simulation/step", { steps: n, observe: false }),
  reset: (seed?: number | null) => post("/episode/reset", seed != null ? { seed } : {}),
  action: (agent: string, action: any) => post(`/agents/${agent}/action`, { action }),
  overlay: () => j<any>("/overlay"),
  worldState: () => j<any>("/world/state"),
  recordings: () => j<RecordingMeta[]>("/recordings"),
  recordingStart: () => post("/recordings/start", { observations: true, states: true }),
  recordingStop: () => post("/recordings/stop"),
  replay: (id: string, until?: number | null) => post(`/recordings/${id}/replay`, until != null ? { until_iteration: until } : {}),
  snapshots: () => j<SnapshotMeta[]>("/snapshots"),
  snapshot: (name?: string) => post("/snapshots", { name }),
  restore: (id: string) => post(`/snapshots/${id}/restore`),
};

export type WsMessage =
  | { type: "hello"; status: SimStatus }
  | { type: "state"; sim_time: number; paused: boolean; mode: string; rtf: number; iterations: number; poses: Record<string, number[]>; vel: Record<string, number[]>; wp?: Record<string, number[]> }
  | { type: "event" } & SimEvent
  | { type: "scenario_loaded"; episode: Episode } | { type: "scene_changed" } | { type: "reset"; episode: Episode }
  | { type: "restored"; snapshot: string; divergence: any }
  | { type: "ack"; for: string; status: SimStatus } | { type: "error"; for: string; message: string }
  | { type: "overlay"; data: TaskOverlay | null };

/** Annotation pushed by an external task/tool via POST /overlay. The simulator does not interpret it. */
export interface TaskOverlay {
  task: string; level?: string; agent: string; target: number[]; start?: number[]; distance: number; success_radius?: number;
  step: number; max_steps?: number; reward: number; return: number; status: string; action: number[] | null;
  observation?: number[]; observation_mode?: string;
}

export class Telemetry {
  private ws: WebSocket | null = null; private closed = false; private timer: any;
  constructor(private onMsg: (m: WsMessage) => void, private onOpen: (ok: boolean) => void) { this.open(); }
  private open() {
    this.ws = new WebSocket(WS_BASE + "/ws");
    this.ws.onopen = () => this.onOpen(true);
    this.ws.onmessage = (e) => this.onMsg(JSON.parse(e.data));
    this.ws.onclose = () => { this.onOpen(false); if (!this.closed) this.timer = setTimeout(() => this.open(), 1000); };
    this.ws.onerror = () => this.ws?.close();
  }
  /** Commands go through the same public action API as any external client. */
  send(cmd: Record<string, unknown>) { if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(cmd)); }
  close() { this.closed = true; clearTimeout(this.timer); this.ws?.close(); }
}

/** Binary sensor stream -> <img>. One authoritative frame source; only the container differs. */
export class FrameStream {
  private ws: WebSocket; private url: string | null = null;
  fps = 0; private n = 0; private t0 = performance.now();
  constructor(agent: string, sensor: string, format: string, private img: HTMLImageElement, fps = 10) {
    this.ws = new WebSocket(`${WS_BASE}/ws/sensors/${agent}/${sensor}?format=${format}&fps=${fps}`);
    this.ws.binaryType = "blob";
    this.ws.onmessage = (e) => {
      if (typeof e.data === "string") return;
      if (this.url) URL.revokeObjectURL(this.url);
      this.url = URL.createObjectURL(e.data as Blob);
      this.img.src = this.url;
      this.n++; const now = performance.now();
      if (now - this.t0 > 1000) { this.fps = this.n * 1000 / (now - this.t0); this.n = 0; this.t0 = now; }
    };
  }
  close() { this.ws.close(); if (this.url) URL.revokeObjectURL(this.url); }
}

export const fmtClock = (t: number) => {
  const m = Math.floor(t / 60), s = t - m * 60;
  return `${String(m).padStart(2, "0")}:${s.toFixed(2).padStart(5, "0")}`;
};
