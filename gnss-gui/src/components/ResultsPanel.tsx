import { useEffect, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Dialog, DialogContent, DialogHeader, DialogTitle } from "@/components/ui/dialog"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { FileText, Eye } from "lucide-react"
import { api, fmtSize, fmtTime, type FilePreview, type RunInfo } from "@/lib/api"

// 结果面板：历史运行列表 + 结果文件查看（txt/csv 预览）
export default function ResultsPanel({ refreshKey }: { refreshKey: number }) {
  const [runs, setRuns] = useState<RunInfo[]>([])
  const [selected, setSelected] = useState<string>("")
  const [preview, setPreview] = useState<FilePreview | null>(null)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    api.listRuns().then((r) => {
      setRuns(r)
      setSelected((sel) => sel || r[0]?.id || "")
    }).catch(() => { /* 忽略 */ })
  }, [refreshKey])

  const run = runs.find((r) => r.id === selected) || runs[0]

  const view = async (name: string) => {
    if (!run) return
    setError(null)
    try {
      setPreview(await api.previewFile(run.id, name))
    } catch (e: any) {
      setError(e.message)
    }
  }

  const statusVariant = (s: string) =>
    s === "success" ? "secondary" : s === "failed" ? "destructive" : "outline"

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between gap-2">
          <CardTitle className="text-base flex items-center gap-2">
            <FileText className="h-4 w-4" /> 结果文件
          </CardTitle>
          {runs.length > 1 && (
            <Select value={run?.id} onValueChange={setSelected}>
              <SelectTrigger className="h-7 w-40 text-xs font-mono">
                <SelectValue />
              </SelectTrigger>
              <SelectContent>
                {runs.map((r) => (
                  <SelectItem key={r.id} value={r.id} className="font-mono text-xs">
                    {r.id} {r.status === "success" ? "✓" : r.status === "failed" ? "✗" : ""}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          )}
        </div>
      </CardHeader>
      <CardContent className="space-y-3">
        {error && <div className="text-xs text-destructive">{error}</div>}
        {!run && <div className="text-sm text-muted-foreground">还没有运行记录</div>}
        {run && (
          <>
            <div className="flex flex-wrap items-center gap-x-3 gap-y-1 text-xs text-muted-foreground">
              <Badge variant={statusVariant(run.status)}>
                {run.status === "success" ? "成功" : run.status === "failed" ? "失败" : run.status === "running" ? "运行中" : run.status}
              </Badge>
              <span>{fmtTime(run.startedAt)}</span>
              <span className="font-mono">{run.id}</span>
            </div>
            {(run.stats.float || run.stats.fixed) && (
              <div className="rounded-md border bg-muted/40 px-3 py-2 space-y-1">
                {run.stats.float && <div className="font-mono text-xs whitespace-nowrap overflow-x-auto">{run.stats.float}</div>}
                {run.stats.fixed && <div className="font-mono text-xs whitespace-nowrap overflow-x-auto">{run.stats.fixed}</div>}
              </div>
            )}
            {run.stats.live && run.stats.live.epoch > 0 && (
              <div className="rounded-md border bg-muted/40 px-3 py-2 font-mono text-xs flex flex-wrap gap-x-3 gap-y-1">
                <span>历元 {run.stats.live.epoch}</span>
                <span>固定 {run.stats.live.fixed} 次</span>
                <span>固定率 {run.stats.live.fixRate != null ? (run.stats.live.fixRate * 100).toFixed(1) + "%" : "--"}</span>
                {run.stats.live.kfReset > 0 && <span className="text-destructive font-semibold">滤波重置 {run.stats.live.kfReset} 次（受过干扰）</span>}
              </div>
            )}
            {run.files.length === 0 && <div className="text-sm text-muted-foreground">此运行没有结果文件</div>}
            <div className="space-y-1">
              {run.files.map((f) => (
                <div key={f.name} className="flex items-center gap-2 rounded-md border px-3 py-1.5">
                  <FileText className="h-3.5 w-3.5 text-muted-foreground shrink-0" />
                  <span className="text-sm truncate">{f.name}</span>
                  <span className="text-xs text-muted-foreground shrink-0">{fmtSize(f.size)}</span>
                  <Button variant="ghost" size="sm" className="ml-auto h-6 text-xs" onClick={() => view(f.name)}>
                    <Eye className="h-3 w-3 mr-1" /> 查看
                  </Button>
                </div>
              ))}
            </div>
          </>
        )}
      </CardContent>

      <Dialog open={!!preview} onOpenChange={(o) => !o && setPreview(null)}>
        <DialogContent className="max-w-4xl">
          <DialogHeader>
            <DialogTitle className="font-mono text-sm">
              {preview?.name}
              {preview && <span className="ml-2 text-xs text-muted-foreground">共 {preview.totalLines} 行{preview.truncated ? "（仅显示前 400 行）" : ""}</span>}
            </DialogTitle>
          </DialogHeader>
          <pre className="max-h-[60vh] overflow-auto rounded-md border bg-muted/40 p-3 text-[11px] leading-4 font-mono whitespace-pre">
            {preview?.content}
          </pre>
        </DialogContent>
      </Dialog>
    </Card>
  )
}
