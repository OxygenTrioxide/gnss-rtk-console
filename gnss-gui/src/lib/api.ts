// 后端 API 类型与封装（后端在 vite dev server 同进程，/api/* 路径）

export interface ConfigEntry { key: string; value: string }
export interface ConfigData {
  path: string
  exists: boolean
  entries: ConfigEntry[]
  savedAt: string | null
}
export interface EnvInfo {
  exe: string
  exeExists: boolean
  configPath: string
  configExists: boolean
  runsDir: string
}
export interface FsData {
  path: string
  parent: string | null
  dirs: string[]
  files: { name: string; size: number; mtime: string }[]
}
export interface RunFile { name: string; size: number; mtime: string }
export interface RunStats { float?: string; fixed?: string; live?: LiveStats }
export interface LiveStats {
  epoch: number
  fixed: number
  kfReset: number
  state: string
  baseSat?: number
  rovSat?: number
  refG?: string
  refC?: string
  baseLen?: number
  ratio?: number
  sigma0?: number
  fixRate?: number
}
export interface RunInfo {
  id: string
  startedAt: string
  status: string
  stats: RunStats
  files: RunFile[]
}
export interface FilePreview {
  name: string
  content: string
  totalLines: number
  truncated: boolean
}

async function req<T>(url: string, init?: RequestInit): Promise<T> {
  const r = await fetch(url, init)
  const data = await r.json().catch(() => ({}))
  if (!r.ok) throw new Error((data as any).error || `请求失败 (${r.status})`)
  return data as T
}

export const api = {
  env: () => req<EnvInfo>("/api/env"),
  getConfig: () => req<ConfigData>("/api/config"),
  saveConfig: (values: Record<string, string>) =>
    req<{ ok: boolean; savedAt: string }>("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ values }),
    }),
  listDir: (path: string) => req<FsData>("/api/fs?path=" + encodeURIComponent(path)),
  startRun: () =>
    req<{ ok: boolean; runId: string; dir: string }>("/api/run", { method: "POST" }),
  stopRun: () => req<{ ok: boolean }>("/api/run/stop", { method: "POST" }),
  listRuns: () => req<RunInfo[]>("/api/runs"),
  previewFile: (id: string, name: string) =>
    req<FilePreview>("/api/run/file?id=" + encodeURIComponent(id) + "&name=" + encodeURIComponent(name)),
}

export function fmtSize(n: number): string {
  if (n < 1024) return n + " B"
  if (n < 1024 * 1024) return (n / 1024).toFixed(1) + " KB"
  return (n / 1024 / 1024).toFixed(2) + " MB"
}

export function fmtTime(iso: string): string {
  try {
    const d = new Date(iso)
    return d.toLocaleString("zh-CN", { hour12: false })
  } catch {
    return iso
  }
}
