import { useEffect, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Play, Square, Terminal } from "lucide-react"
import type { RunStats } from "@/lib/api"

export type RunStatus = "idle" | "running" | "success" | "failed" | "stopped"

const STATUS_META: Record<RunStatus, { label: string; variant: "default" | "secondary" | "destructive" | "outline" }> = {
  idle: { label: "未运行", variant: "outline" },
  running: { label: "解算中…", variant: "default" },
  success: { label: "完成", variant: "secondary" },
  failed: { label: "失败", variant: "destructive" },
  stopped: { label: "已停止", variant: "outline" },
}

interface Props {
  status: RunStatus
  lines: string[]
  stats: RunStats
  runId: string | null
  startedAt?: string | null
  maxRunMin?: number
  exeReady: boolean
  onStart: () => void
  onStop: () => void
}

export default function RunPanel({ status, lines, stats, runId, startedAt, maxRunMin, exeReady, onStart, onStop }: Props) {
  const scrollRef = useRef<HTMLDivElement>(null)
  const [now, setNow] = useState(() => Date.now())

  // 到点自动停止的倒计时（MAX RUNNING MINUTES > 0 且任务运行中时显示）
  const timed = status === "running" && !!startedAt && !!maxRunMin && maxRunMin > 0
  useEffect(() => {
    if (!timed) return
    const t = setInterval(() => setNow(Date.now()), 1000)
    return () => clearInterval(t)
  }, [timed])
  let countdown: string | null = null
  if (timed && startedAt) {
    const remainSec = Math.max(0, maxRunMin! * 60 - (now - new Date(startedAt).getTime()) / 1000)
    const h = Math.floor(remainSec / 3600)
    const m = Math.floor((remainSec % 3600) / 60)
    const s = Math.floor(remainSec % 60)
    countdown = h > 0 ? `${h}:${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`
      : `${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`
  }
  useEffect(() => {
    const el = scrollRef.current
    if (el) el.scrollTop = el.scrollHeight
  }, [lines.length])

  const meta = STATUS_META[status]

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between gap-2">
          <CardTitle className="text-base flex items-center gap-2">
            <Terminal className="h-4 w-4" /> 解算运行
          </CardTitle>
          <div className="flex items-center gap-2">
            <Badge variant={meta.variant}>{meta.label}</Badge>
            {countdown && <Badge variant="outline" className="font-mono">剩余 {countdown}</Badge>}
            {runId && <span className="text-[11px] text-muted-foreground font-mono">{runId}</span>}
          </div>
        </div>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex gap-2">
          {status === "running" ? (
            <Button variant="destructive" size="sm" onClick={onStop}>
              <Square className="h-3.5 w-3.5 mr-1" /> 停止
            </Button>
          ) : (
            <Button size="sm" onClick={onStart} disabled={!exeReady}>
              <Play className="h-3.5 w-3.5 mr-1" /> 开始解算
            </Button>
          )}
          {!exeReady && (
            <span className="text-xs text-destructive self-center">未找到 Nav.exe，请先用 Release 模式编译</span>
          )}
        </div>

        {stats.live && (
          <div className="rounded-md border bg-muted/40 px-3 py-2 space-y-1.5">
            <div className="flex items-center gap-2">
              <span className="text-xs font-medium text-muted-foreground">实时统计</span>
              <Badge
                variant={
                  stats.live.state === "固定解" ? "default" :
                  stats.live.state === "浮点解" ? "secondary" :
                  stats.live.state === "等待数据" ? "outline" : "destructive"
                }
              >{stats.live.state}</Badge>
              {stats.live.kfReset > 0 && (
                <Badge variant="destructive">滤波重置 {stats.live.kfReset} 次</Badge>
              )}
            </div>
            <div className="grid grid-cols-4 gap-x-3 gap-y-1 font-mono text-xs">
              <span>历元 {stats.live.epoch}</span>
              <span>固定 {stats.live.fixed} 次</span>
              <span>固定率 {stats.live.fixRate != null ? (stats.live.fixRate * 100).toFixed(1) + "%" : "--"}</span>
              <span>基线 {stats.live.baseLen != null ? stats.live.baseLen.toFixed(3) + " m" : "--"}</span>
              <span>ratio {stats.live.ratio != null ? stats.live.ratio.toFixed(2) : "--"}</span>
              <span>Sigma0 {stats.live.sigma0 != null ? stats.live.sigma0.toFixed(4) : "--"}</span>
              <span>星数 基{stats.live.baseSat ?? "--"}/流{stats.live.rovSat ?? "--"}</span>
              <span>参考星 {stats.live.refG ?? "--"}/{stats.live.refC ?? "--"}</span>
            </div>
          </div>
        )}

        {stats.float && (
          <div className="rounded-md border bg-muted/40 px-3 py-2 space-y-1">
            <div className="text-xs font-medium text-muted-foreground">精度统计（与参考真值 E/N/U 差值）</div>
            {stats.float && <div className="font-mono text-xs whitespace-nowrap overflow-x-auto">{stats.float}</div>}
            {stats.fixed && <div className="font-mono text-xs whitespace-nowrap overflow-x-auto">{stats.fixed}</div>}
          </div>
        )}

        <div ref={scrollRef} className="h-64 overflow-y-auto rounded-md border bg-black/90 p-2 font-mono text-[11px] leading-4">
          {lines.length === 0 && (
            <div className="text-white/40">点击“开始解算”启动 Nav.exe，输出将实时显示在这里…</div>
          )}
          {lines.map((ln, i) => (
            <div key={i} className={
              /error|fail|cannot|拒绝|失败|重置|\[KF \]/i.test(ln) ? "text-red-400" :
              /Float|Fixed|succeeded|完成|written/i.test(ln) ? "text-green-400" :
              "text-white/80"
            }>{ln}</div>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
