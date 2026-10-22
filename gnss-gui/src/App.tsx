import { useCallback, useEffect, useMemo, useRef, useState } from "react"
import { Badge } from "@/components/ui/badge"
import { Satellite, CheckCircle2, XCircle } from "lucide-react"
import ConfigPanel, { type Values } from "@/components/ConfigPanel"
import RunPanel, { type RunStatus } from "@/components/RunPanel"
import ResultsPanel from "@/components/ResultsPanel"
import { api, fmtTime, type ConfigData, type EnvInfo, type RunStats } from "@/lib/api"

export default function App() {
  const [env, setEnv] = useState<EnvInfo | null>(null)
  const [config, setConfig] = useState<ConfigData | null>(null)
  const [values, setValues] = useState<Values>({})
  const [snapshot, setSnapshot] = useState("")
  const [saving, setSaving] = useState(false)
  const [notice, setNotice] = useState<string | null>(null)

  const [runStatus, setRunStatus] = useState<RunStatus>("idle")
  const [runId, setRunId] = useState<string | null>(null)
  const [runStartedAt, setRunStartedAt] = useState<string | null>(null)
  const [lines, setLines] = useState<string[]>([])
  const [stats, setStats] = useState<RunStats>({})
  const [runsRefresh, setRunsRefresh] = useState(0)

  const linesRef = useRef<string[]>([])
  const activeRunRef = useRef<string | null>(null)

  const loadConfig = useCallback(async () => {
    const d = await api.getConfig()
    const v: Values = {}
    for (const e of d.entries) v[e.key] = e.value
    setConfig(d)
    setValues(v)
    setSnapshot(JSON.stringify(v))
  }, [])

  useEffect(() => {
    api.env().then(setEnv).catch(() => setEnv(null))
    loadConfig().catch((e) => setNotice(e.message))
  }, [loadConfig])

  // SSE：实时接收解算日志与状态
  useEffect(() => {
    const es = new EventSource("/api/run/events")
    es.onmessage = (ev) => {
      let msg: any
      try { msg = JSON.parse(ev.data) } catch { return }
      if (msg.type === "hello") {
        if (msg.current?.status === "running") {
          setRunStatus("running")
          setRunId(msg.current.id)
          setRunStartedAt(msg.current.startedAt || null)
          activeRunRef.current = msg.current.id
        }
      } else if (msg.type === "log") {
        if (msg.runId && activeRunRef.current && msg.runId !== activeRunRef.current) return
        linesRef.current = [...linesRef.current, msg.line].slice(-2000)
        setLines(linesRef.current)
        if (msg.stats) setStats(msg.stats)
      } else if (msg.type === "status") {
        setRunStatus(msg.status)
        if (msg.runId) {
          setRunId(msg.runId)
          activeRunRef.current = msg.runId
        }
        if (msg.status === "running") {
          linesRef.current = []
          setLines([])
          setStats({})
          setRunStartedAt(msg.startedAt || null)
        } else {
          if (msg.stats) setStats(msg.stats)
          setRunsRefresh((k) => k + 1)
        }
      }
    }
    return () => es.close()
  }, [])

  const dirty = useMemo(() => JSON.stringify(values) !== snapshot, [values, snapshot])

  const onChange = useCallback((key: string, value: string) => {
    setValues((v) => ({ ...v, [key]: value }))
  }, [])

  const onSave = useCallback(async () => {
    setSaving(true)
    setNotice(null)
    try {
      await api.saveConfig(values)
      setSnapshot(JSON.stringify(values))
      setConfig((c) => c ? { ...c, savedAt: new Date().toISOString() } : c)
      setNotice("配置已保存到 rtk.ini（原文件备份为 rtk.ini.bak-gui）")
    } catch (e: any) {
      setNotice("保存失败：" + e.message)
    } finally {
      setSaving(false)
    }
  }, [values])

  const onStart = useCallback(async () => {
    if (dirty && !window.confirm("配置有未保存的修改，将按当前 rtk.ini 运行。继续？")) return
    setNotice(null)
    try {
      const r = await api.startRun()
      activeRunRef.current = r.runId
      setRunId(r.runId)
    } catch (e: any) {
      setNotice("启动失败：" + e.message)
    }
  }, [dirty])

  const onStop = useCallback(async () => {
    try { await api.stopRun() } catch { /* 忽略 */ }
  }, [])

  return (
    <div className="min-h-screen bg-background">
      <header className="border-b bg-card sticky top-0 z-10">
        <div className="mx-auto max-w-7xl px-4 py-3 flex items-center gap-3 flex-wrap">
          <Satellite className="h-5 w-5 text-primary" />
          <h1 className="text-lg font-semibold">GNSS RTK 解算控制台</h1>
          {env && (
            <div className="flex items-center gap-2 text-xs flex-wrap">
              <Badge variant={env.exeExists ? "secondary" : "destructive"} className="font-mono gap-1">
                {env.exeExists ? <CheckCircle2 className="h-3 w-3" /> : <XCircle className="h-3 w-3" />}
                Nav.exe {env.exeExists ? "就绪" : "缺失"}
              </Badge>
              <Badge variant={env.configExists ? "outline" : "destructive"} className="font-mono">
                rtk.ini {env.configExists ? "就绪" : "缺失"}
              </Badge>
              {config?.savedAt && (
                <span className="text-muted-foreground">保存于 {fmtTime(config.savedAt)}</span>
              )}
              {dirty && <Badge variant="default">有未保存修改</Badge>}
            </div>
          )}
        </div>
        {notice && (
          <div className="border-t bg-muted/50 px-4 py-1.5 text-xs text-muted-foreground">{notice}</div>
        )}
      </header>

      <main className="mx-auto max-w-7xl p-4 grid gap-4 xl:grid-cols-5">
        <div className="xl:col-span-3">
          <ConfigPanel
            config={config}
            values={values}
            onChange={onChange}
            onSave={onSave}
            onReload={() => loadConfig().catch((e) => setNotice(e.message))}
            saving={saving}
            dirty={dirty}
          />
        </div>
        <div className="xl:col-span-2 space-y-4">
          <RunPanel
            status={runStatus}
            lines={lines}
            stats={stats}
            runId={runId}
            startedAt={runStartedAt}
            maxRunMin={parseFloat(values["MAX RUNNING MINUTES"] || "0") || 0}
            exeReady={!!env?.exeExists}
            onStart={onStart}
            onStop={onStop}
          />
          <ResultsPanel refreshKey={runsRefresh} />
        </div>
      </main>
    </div>
  )
}
