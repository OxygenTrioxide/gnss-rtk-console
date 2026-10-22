import { useState } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Switch } from "@/components/ui/switch"
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group"
import { Accordion, AccordionContent, AccordionItem, AccordionTrigger } from "@/components/ui/accordion"
import { FolderSearch, Save, RotateCcw } from "lucide-react"
import FilePickerDialog from "./FilePickerDialog"
import type { ConfigData } from "@/lib/api"

export type Values = Record<string, string>

interface Props {
  config: ConfigData | null
  values: Values
  onChange: (key: string, value: string) => void
  onSave: () => void
  onReload: () => void
  saving: boolean
  dirty: boolean
}

const K = {
  dataSrc: "DATA SOURCE MODE",
  legacySrc: "FROM FILE OR COM",
  procMode: "RTK PROCESSING MODE",
  trop: "TROP MODEL",
  outFormat: "RESULT FILE FORMAT",
  spOut: "ENABLE SPP OUTPUT",
  flOut: "ENABLE FLOAT OUTPUT",
  fxOut: "ENABLE FIXED OUTPUT",
  arTests: "ENABLE AR TESTS",
  recRaw: "RECORD RAW DATA",
  baseFile: "BASE OBSDATA SOURCE FILE",
  roverFile: "ROVER OBSDATA SOURCE FILE",
  posFile: "POSITION RESULT FILE",
  outDir: "OUTPUT DIRECTORY",
  roverIp: "ROVER IP ADDRESS AND PORT",
  baseIp: "BASE IP ADDRESS AND PORT",
  roverCom: "ROVER COM SETUP",
  baseCom: "BASE COM SETUP",
  noise: "CODE AND CARRIER PHASE NOISE",
  elev: "THRESHOLD FOR ELEVATION MASK",
  ratio: "RATIO FOR DD FIXED SOLUTION",
  csThres: "CYCLE SLIP THRESHOLD",
  outThres: "OUTLIER THRESHOLD",
  basePos: "BASE POSITION",
  maxRun: "MAX RUNNING MINUTES",
}

function split2(v: string): [string, string] {
  const p = v.trim().split(/\s+/)
  return [p[0] || "", p[1] || ""]
}
function split3(v: string): [string, string, string] {
  const p = v.trim().split(/\s+/)
  return [p[0] || "", p[1] || "", p[2] || ""]
}

// 单选行（水平排列）
function RadioRow({ label, value, options, onChange }: {
  label: string; value: string
  options: { value: string; label: string }[]
  onChange: (v: string) => void
}) {
  return (
    <div className="grid grid-cols-[130px_1fr] items-center gap-2">
      <Label className="text-xs text-muted-foreground">{label}</Label>
      <RadioGroup value={value} onValueChange={onChange} className="flex flex-wrap gap-x-4 gap-y-1">
        {options.map((o) => (
          <div key={o.value} className="flex items-center gap-1.5">
            <RadioGroupItem value={o.value} id={`${label}-${o.value}`} className="h-3.5 w-3.5" />
            <Label htmlFor={`${label}-${o.value}`} className="text-sm font-normal cursor-pointer">{o.label}</Label>
          </div>
        ))}
      </RadioGroup>
    </div>
  )
}

function SwitchRow({ label, hint, checked, onChange }: {
  label: string; hint?: string; checked: boolean; onChange: (v: boolean) => void
}) {
  return (
    <div className="flex items-center justify-between rounded-md border px-3 py-2">
      <div>
        <div className="text-sm">{label}</div>
        {hint && <div className="text-xs text-muted-foreground">{hint}</div>}
      </div>
      <Switch checked={checked} onCheckedChange={onChange} />
    </div>
  )
}

export default function ConfigPanel({ config, values, onChange, onSave, onReload, saving, dirty }: Props) {
  const [picker, setPicker] = useState<{ key: string; dir?: boolean; exts?: string[] } | null>(null)

  const get = (key: string, def = "") => values[key] ?? def
  const set = (key: string) => (v: string) => onChange(key, v)
  const setPair = (key: string) => (i: 0 | 1) => (e: React.ChangeEvent<HTMLInputElement>) => {
    const [a, b] = split2(get(key))
    onChange(key, i === 0 ? `${e.target.value} ${b}` : `${a} ${e.target.value}`.trim())
  }
  const setTriple = (key: string) => (i: 0 | 1 | 2) => (e: React.ChangeEvent<HTMLInputElement>) => {
    const [a, b, c] = split3(get(key))
    const arr = [a, b, c]; arr[i] = e.target.value
    onChange(key, arr.join(" ").trim())
  }

  const dataSrc = get(K.dataSrc) || (get(K.legacySrc, "1") === "1" ? "0" : "2")
  const showFile = dataSrc === "0"
  const showNet = dataSrc === "1"
  const showCom = dataSrc === "2"

  const browseInitial = (cur: string) => {
    if (cur) {
      const s = cur.replace(/\//g, "\\")
      const idx = s.lastIndexOf("\\")
      if (idx > 0) return s.slice(0, idx)
    }
    return "C:\\Users\\OXYGEN DIOXIDE\\Desktop\\gnss"
  }

  const pathRow = (key: string, label: string, pickDir = false, exts?: string[]) => (
    <div className="grid grid-cols-[130px_1fr] items-center gap-2">
      <Label className="text-xs text-muted-foreground">{label}</Label>
      <div className="flex gap-1.5">
        <Input value={get(key)} onChange={(e) => set(key)(e.target.value)} className="h-8 font-mono text-xs" placeholder={pickDir ? "留空 = 不输出" : "文件路径"} />
        <Button variant="outline" size="icon" className="h-8 w-8 shrink-0" onClick={() => setPicker({ key, dir: pickDir, exts })}>
          <FolderSearch className="h-3.5 w-3.5" />
        </Button>
      </div>
    </div>
  )

  const pairRow = (key: string, label: string, ph: [string, string], type = "text") => {
    const [a, b] = split2(get(key))
    return (
      <div className="grid grid-cols-[130px_1fr] items-center gap-2">
        <Label className="text-xs text-muted-foreground">{label}</Label>
        <div className="flex gap-1.5">
          <Input value={a} type={type} onChange={setPair(key)(0)} placeholder={ph[0]} className="h-8 font-mono text-xs" />
          <Input value={b} type={type} onChange={setPair(key)(1)} placeholder={ph[1]} className="h-8 font-mono text-xs" />
        </div>
      </div>
    )
  }

  const numRow = (key: string, label: string, unit: string, step = "any") => (
    <div className="grid grid-cols-[130px_1fr] items-center gap-2">
      <Label className="text-xs text-muted-foreground">{label}</Label>
      <div className="flex items-center gap-1.5">
        <Input value={get(key)} type="number" step={step} onChange={(e) => set(key)(e.target.value)}
          className="h-8 font-mono text-xs" />
        {unit && <span className="text-xs text-muted-foreground shrink-0">{unit}</span>}
      </div>
    </div>
  )

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between">
          <div>
            <CardTitle className="text-base">解算配置（rtk.ini）</CardTitle>
            <CardDescription className="font-mono text-[11px] truncate max-w-md">
              {config?.path}
            </CardDescription>
          </div>
          <div className="flex gap-1.5">
            <Button variant="outline" size="sm" onClick={onReload} disabled={saving}>
              <RotateCcw className="h-3.5 w-3.5 mr-1" /> 重新加载
            </Button>
            <Button size="sm" onClick={onSave} disabled={saving || !dirty}>
              <Save className="h-3.5 w-3.5 mr-1" /> {saving ? "保存中…" : dirty ? "保存配置" : "已保存"}
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent className="space-y-5">
        <div className="space-y-2.5">
          <div className="text-sm font-medium">数据源</div>
          <RadioRow label="数据源模式" value={dataSrc} onChange={(v) => onChange(K.dataSrc, v)} options={[
            { value: "0", label: "文件" },
            { value: "1", label: "网络" },
            { value: "2", label: "串口" },
          ]} />
          {showFile && (
            <div className="space-y-2 rounded-md border p-3 bg-muted/40">
              {pathRow(K.baseFile, "基站观测文件", false, [".bin", ".dat", ".log", ".txt"])}
              {pathRow(K.roverFile, "流动站观测文件", false, [".bin", ".dat", ".log", ".txt"])}
              {pathRow(K.posFile, "参考坐标文件", false, [".pos-lsq", ".txt", ".csv"])}
            </div>
          )}
          {showNet && (
            <div className="space-y-2 rounded-md border p-3 bg-muted/40">
              {pairRow(K.baseIp, "基站 IP / 端口", ["47.114.134.129", "7190"])}
              {pairRow(K.roverIp, "流动站 IP / 端口", ["8.148.22.229", "7201"])}
            </div>
          )}
          {showCom && (
            <div className="space-y-2 rounded-md border p-3 bg-muted/40">
              {pairRow(K.baseCom, "基站串口 COM号 / 波特率", ["3", "115200"], "number")}
              {pairRow(K.roverCom, "流动站串口 COM号 / 波特率", ["4", "115200"], "number")}
            </div>
          )}
        </div>

        <div className="space-y-2.5">
          <div className="text-sm font-medium">解算模式</div>
          <RadioRow label="处理模式" value={get(K.procMode, "1")} onChange={set(K.procMode)} options={[
            { value: "1", label: "EKF 卡尔曼滤波" },
            { value: "2", label: "LSQ 最小二乘" },
          ]} />
          <RadioRow label="对流层模型" value={get(K.trop, "0")} onChange={set(K.trop)} options={[
            { value: "0", label: "Hopfield" },
            { value: "1", label: "Saastamoinen" },
          ]} />
          <RadioRow label="结果文件格式" value={get(K.outFormat, "0")} onChange={set(K.outFormat)} options={[
            { value: "0", label: "txt" },
            { value: "1", label: "csv" },
          ]} />
        </div>

        <div className="space-y-2.5">
          <div className="text-sm font-medium">解算参数</div>
          {pairRow(K.noise, "伪距 / 载波噪声 (m)", ["0.5", "0.002"])}
          {numRow(K.elev, "截止高度角", "°")}
          {numRow(K.ratio, "ratio 检验阈值", "", "0.1")}
          {pairRow(K.csThres, "周跳 MW / GF 阈值", ["3.0", "0.15"])}
          {pairRow(K.outThres, "粗差 GF / MW 阈值", ["0.05", "3.0"])}
          {(() => {
            // 时长填空：__小时 __分钟 __秒（配置文件中仍以分钟存储，支持小数）
            const totalSec = Math.round((parseFloat(get(K.maxRun, "0")) || 0) * 60)
            const parts: [number, string, number][] = [
              [Math.floor(totalSec / 3600), "小时", 999],
              [Math.floor((totalSec % 3600) / 60), "分钟", 59],
              [totalSec % 60, "秒", 59],
            ]
            const setPart = (idx: 0 | 1 | 2) => (e: React.ChangeEvent<HTMLInputElement>) => {
              const v = Math.max(0, Math.min(parts[idx][2], Math.floor(Number(e.target.value) || 0)))
              const arr = [parts[0][0], parts[1][0], parts[2][0]]
              arr[idx] = v
              const min = (arr[0] * 3600 + arr[1] * 60 + arr[2]) / 60
              onChange(K.maxRun, min > 0 ? String(Math.round(min * 1000) / 1000) : "0")
            }
            return (
              <div className="grid grid-cols-[130px_1fr] items-center gap-2">
                <Label className="text-xs text-muted-foreground">最长解算时长</Label>
                <div className="flex items-center gap-1.5">
                  {parts.map(([v, unit, max], i) => (
                    <div key={unit} className="flex items-center gap-1">
                      <Input value={v} type="number" min={0} max={max} step={1} onChange={setPart(i as 0 | 1 | 2)}
                        className="h-8 w-20 font-mono text-xs" />
                      <span className="text-xs text-muted-foreground shrink-0">{unit}</span>
                    </div>
                  ))}
                  <span className="text-[11px] text-muted-foreground shrink-0">（全 0 = 不限时）</span>
                </div>
              </div>
            )
          })()}
        </div>

        <div className="space-y-2.5">
          <div className="text-sm font-medium">输出开关</div>
          <div className="grid gap-2 sm:grid-cols-2">
            <SwitchRow label="SPP 单点定位结果" checked={get(K.spOut, "1") === "1"} onChange={(v) => set(K.spOut)(v ? "1" : "0")} />
            <SwitchRow label="浮点解结果" checked={get(K.flOut, "1") === "1"} onChange={(v) => set(K.flOut)(v ? "1" : "0")} />
            <SwitchRow label="固定解结果" checked={get(K.fxOut, "1") === "1"} onChange={(v) => set(K.fxOut)(v ? "1" : "0")} />
            <SwitchRow label="模糊度确认三项检验" hint="基线一致性 χ² / 单位权中误差 / ratio" checked={get(K.arTests, "1") === "1"} onChange={(v) => set(K.arTests)(v ? "1" : "0")} />
            <SwitchRow label="录制原始数据流" hint="网络/串口模式：把基站、流动站原始数据录为 .bin 存入本次运行目录" checked={get(K.recRaw, "0") === "1"} onChange={(v) => set(K.recRaw)(v ? "1" : "0")} />
          </div>
          {pathRow(K.outDir, "附加输出目录", true)}
        </div>

        <Accordion type="single" collapsible>
          <AccordionItem value="adv">
            <AccordionTrigger className="text-sm font-medium py-2">高级选项</AccordionTrigger>
            <AccordionContent className="space-y-2 pb-1">
              <RadioRow label="旧版数据源开关" value={get(K.legacySrc, "1")} onChange={set(K.legacySrc)} options={[
                { value: "1", label: "文件（1）" },
                { value: "0", label: "串口（0）" },
              ]} />
              <div className="text-[11px] text-muted-foreground -mt-1 ml-[130px]">
                FROM FILE OR COM，仅当未设置 DATA SOURCE MODE 时生效
              </div>
              <div className="grid grid-cols-[130px_1fr] items-center gap-2">
                <Label className="text-xs text-muted-foreground">基站坐标 ECEF (m)</Label>
                <div className="flex gap-1.5">
                  {[0, 1, 2].map((i) => (
                    <Input key={i} value={split3(get(K.basePos))[i as 0 | 1 | 2]} type="number" step="any"
                      onChange={setTriple(K.basePos)(i as 0 | 1 | 2)}
                      placeholder={["X", "Y", "Z"][i]} className="h-8 font-mono text-xs" />
                  ))}
                </div>
              </div>
              <div className="text-[11px] text-muted-foreground ml-[130px]">
                留空 = 自动取基站数据流 bestpos / 参考文件首行
              </div>
            </AccordionContent>
          </AccordionItem>
        </Accordion>
      </CardContent>

      <FilePickerDialog
        open={!!picker}
        onOpenChange={(o) => !o && setPicker(null)}
        initialPath={picker ? browseInitial(get(picker.key)) : ""}
        pickDirectory={picker?.dir}
        extensions={picker?.exts}
        onSelect={(p) => picker && onChange(picker.key, p.replace(/\\/g, "/"))}
      />
    </Card>
  )
}
