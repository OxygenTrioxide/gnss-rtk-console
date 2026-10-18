import { useCallback, useEffect, useState } from "react"
import { Dialog, DialogContent, DialogHeader, DialogTitle } from "@/components/ui/dialog"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { ScrollArea } from "@/components/ui/scroll-area"
import { FolderOpen, File, CornerLeftUp, RefreshCw, HardDrive } from "lucide-react"
import { api, fmtSize, type FsData } from "@/lib/api"

interface Props {
  open: boolean
  onOpenChange: (open: boolean) => void
  /** 初始目录 */
  initialPath: string
  /** true=选择目录，false=选择文件 */
  pickDirectory?: boolean
  /** 文件扩展名过滤（小写，如 [".bin", ".dat"]），空=全部 */
  extensions?: string[]
  onSelect: (path: string) => void
}

// 服务器端目录浏览选择器（浏览器无法直接取得本地文件真实路径，故由后端列目录）
export default function FilePickerDialog({ open, onOpenChange, initialPath, pickDirectory, extensions, onSelect }: Props) {
  const [data, setData] = useState<FsData | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [manual, setManual] = useState("")

  const load = useCallback(async (p: string) => {
    setError(null)
    try {
      setData(await api.listDir(p))
      setManual(p)
    } catch (e: any) {
      setError(e.message)
    }
  }, [])

  useEffect(() => {
    if (open && initialPath) load(initialPath)
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [open])

  const visibleFiles = (data?.files || []).filter((f) => {
    if (!extensions || extensions.length === 0) return true
    const lower = f.name.toLowerCase()
    return extensions.some((ext) => lower.endsWith(ext))
  })

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-2xl">
        <DialogHeader>
          <DialogTitle>{pickDirectory ? "选择目录" : "选择文件"}</DialogTitle>
        </DialogHeader>
        <div className="flex items-center gap-2">
          <Button variant="outline" size="icon" disabled={!data?.parent} onClick={() => data?.parent && load(data.parent)}
            title="上一级">
            <CornerLeftUp className="h-4 w-4" />
          </Button>
          <Input value={manual} onChange={(e) => setManual(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && load(manual)} className="font-mono text-xs" />
          <Button size="icon" variant="outline" onClick={() => load(manual)} title="转到">
            <FolderOpen className="h-4 w-4" />
          </Button>
          <Button size="icon" variant="outline" onClick={() => data && load(data.path)} title="刷新">
            <RefreshCw className="h-4 w-4" />
          </Button>
        </div>
        <div className="text-xs text-muted-foreground font-mono truncate">{data?.path}</div>
        {error && <div className="text-sm text-destructive">{error}</div>}
        <ScrollArea className="h-80 rounded-md border">
          <div className="p-1">
            {data?.dirs.map((d) => (
              <button key={d} onClick={() => load(data.path + "\\" + d)}
                onDoubleClick={pickDirectory ? () => { onSelect(data.path + "\\" + d); onOpenChange(false) } : undefined}
                className="flex w-full items-center gap-2 rounded px-2 py-1.5 text-left text-sm hover:bg-accent">
                <HardDrive className="h-4 w-4 text-muted-foreground" />
                <span>{d}</span>
                {pickDirectory && (
                  <Button size="sm" variant="ghost" className="ml-auto h-6 text-xs"
                    onClick={(e) => { e.stopPropagation(); onSelect(data.path + "\\" + d); onOpenChange(false) }}>
                    选此目录
                  </Button>
                )}
              </button>
            ))}
            {!pickDirectory && visibleFiles.map((f) => (
              <button key={f.name}
                onClick={() => { onSelect(data!.path + "\\" + f.name); onOpenChange(false) }}
                className="flex w-full items-center gap-2 rounded px-2 py-1.5 text-left text-sm hover:bg-accent">
                <File className="h-4 w-4 text-muted-foreground" />
                <span className="truncate">{f.name}</span>
                <span className="ml-auto text-xs text-muted-foreground">{fmtSize(f.size)}</span>
              </button>
            ))}
            {pickDirectory && data && (
              <div className="p-2">
                <Button className="w-full" size="sm" onClick={() => { onSelect(data.path); onOpenChange(false) }}>
                  选择当前目录
                </Button>
              </div>
            )}
            {data && data.dirs.length === 0 && visibleFiles.length === 0 && !pickDirectory && (
              <div className="p-4 text-sm text-muted-foreground">此目录为空</div>
            )}
          </div>
        </ScrollArea>
      </DialogContent>
    </Dialog>
  )
}
