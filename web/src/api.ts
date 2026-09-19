// Simulation API client for the browser. HTTP for commands, WS for telemetry.
export const API = (import.meta as any).env?.VITE_SIMAPI ?? "http://127.0.0.1:8000";

export interface Vec3 { x: number; y: number; z: number }
export interface Quat { x: number; y: number; z: number; w: number }
export interface Pose { position: Vec3; orientation: Quat }
export interface Geometry { type: string; size?: Vec3; radius?: number; length?: number; normal?: Vec3; uri?: string; scale?: Vec3 }
export interface Visual { name: string; pose: Pose; geometry: Geometry; color?: number[] | null }
export interface LinkDesc { name: string; pose: Pose; visuals: Visual[] }
export interface ModelDesc { entity_id: string; kind: string; is_agent: boolean; pose: Pose; links: LinkDesc[]; is_static: boolean }
export interface SceneDesc { world: string; models: ModelDesc[] }
export interface SimStatus { running: boolean; paused: boolean; sim_time: number; real_time_factor: number; iterations: number; scenario?: string }

async function j<T>(path: string, init?: RequestInit): Promise<T> {
  const r = await fetch(API + path, init);
  if (!r.ok) throw new Error(`${path}: ${r.status} ${await r.text()}`);
  return r.json();
}
const post = (p: string, body?: unknown) => j(p, { method: "POST", headers: { "content-type": "application/json" }, body: body ? JSON.stringify(body) : undefined });

export const api = {
  status: () => j<SimStatus>("/status"),
  scenarios: () => j<string[]>("/scenarios"),
  load: (name: string) => post(`/simulation/load/${name}`),
  scene: () => j<SceneDesc>("/scene"),
  entity: (id: string) => j<any>(`/entities/${id}`),
  observation: (id: string) => j<any>(`/agents/${id}/observation`),
  pause: () => post("/simulation/pause"),
  resume: () => post("/simulation/resume"),
  step: (n = 1) => post("/simulation/step", { steps: n }),
  reset: () => post("/simulation/reset"),
};

export type WsMessage =
  | { type: "hello"; status: SimStatus }
  | { type: "state"; sim_time: number; paused: boolean; rtf: number; iterations: number; poses: Record<string, number[]> }
  | { type: "scenario_loaded" } | { type: "scene_changed" } | { type: "reset" }
  | { type: "ack" } | { type: "error"; message: string };

export function connectWs(onMsg: (m: WsMessage) => void, onOpen: (ok: boolean) => void): () => void {
  let ws: WebSocket | null = null, closed = false, timer: any;
  const open = () => {
    ws = new WebSocket(API.replace(/^http/, "ws") + "/ws");
    ws.onopen = () => onOpen(true);
    ws.onmessage = (e) => onMsg(JSON.parse(e.data));
    ws.onclose = () => { onOpen(false); if (!closed) timer = setTimeout(open, 1000); };
    ws.onerror = () => ws?.close();
  };
  open();
  return () => { closed = true; clearTimeout(timer); ws?.close(); };
}
