// 共享 API 模块：开发模式由 vite dev server 挂载，打包后由 Electron 主进程挂载。
// createApi(options) 返回 connect 风格中间件 (req, res, next)。
//
// options:
//   exeFile   Nav.exe 绝对路径
//   iniFile   rtk.ini 绝对路径（界面直接读写该文件）
//   runsDir   每次解算的运行目录根（其下按时间戳建子目录）
//   appName   应用显示名（/api/env 返回）
const path = require("path")
const fs = require("fs")
const { spawn } = require("child_process")
const iconv = require("iconv-lite")

// 程序按键名读取配置：解析 "KEY: value" 行（# 开头与 [段] 行跳过），
// 写回时只替换键行数值部分，注释与其他行原样保留
function loadIni(iniFile) {
  const data = { path: iniFile, exists: fs.existsSync(iniFile), entries: [], lines: [] }
  if (!data.exists) return data
  const raw = fs.readFileSync(iniFile, "utf8")
  data.lines = raw.replace(/^\uFEFF/, "").split(/\r?\n/)
  data.lines.forEach((ln, i) => {
    if (/^\s*#/.test(ln) || /^\s*\[/.test(ln)) return
    const m = ln.match(/^\s*([A-Z][A-Z0-9 ]*?)\s*:\s*(.*)$/)
    if (m) data.entries.push({ key: m[1].trim(), value: m[2].trim(), line: i })
  })
  return data
}

function saveIni(iniFile, values) {
  const data = loadIni(iniFile)
  if (!data.exists) throw new Error("配置文件不存在: " + iniFile)
  // Nav 实际按 "DATA SOURCE MODE"（0=文件 1=网络 2=串口）选择数据源，
  // GUI 界面只写 "FROM FILE OR COM"（1=文件 0=网络）——两者必须同步，
  // 否则切换模式后 Nav 仍按旧模式运行（曾因此误进串口模式秒退）
  values = { ...values }
  if ("FROM FILE OR COM" in values) {
    const m = String(values["FROM FILE OR COM"]).trim()
    values["DATA SOURCE MODE"] = (m === "1") ? "0" : (m === "0" ? "1" : m)
  }
  // 备份原文件
  fs.copyFileSync(iniFile, iniFile + ".bak-gui")
  const lines = [...data.lines]
  const seen = new Set()
  for (const e of data.entries) {
    if (e.key in values) {
      const v = values[e.key]
      lines[e.line] = v ? `${e.key}: ${v}` : `${e.key}:`
      seen.add(e.key)
    }
  }
  // 文件中没有的键追加到末尾
  const extra = Object.keys(values).filter((k) => !seen.has(k))
  if (extra.length) {
    lines.push(...extra.map((k) => (values[k] ? `${k}: ${values[k]}` : `${k}:`)))
  }
  fs.writeFileSync(iniFile, "\uFEFF" + lines.join("\r\n"), "utf8")
}

// ---------------- 运行状态 ----------------
let current = null
const sseClients = new Set()

function sendSse(obj) {
  const payload = `data: ${JSON.stringify(obj)}\n\n`
  for (const res of sseClients) {
    try { res.write(payload) } catch { /* 客户端断开 */ }
  }
}

function listRunFiles(dir) {
  if (!fs.existsSync(dir)) return []
  return fs.readdirSync(dir)
    .filter((n) => /\.(txt|csv)$/i.test(n))
    .map((n) => {
      const st = fs.statSync(path.join(dir, n))
      return { name: n, size: st.size, mtime: st.mtime.toISOString() }
    })
}

// 实时统计：逐行解析 Nav 输出，提取当前解状态、基线、ratio、重置次数等，
// 供前端状态卡片显示（stats.live 随日志事件一起推送）
function updateLiveStats(run, line) {
  const live = (run.stats.live = run.stats.live || {
    epoch: 0, fixed: 0, kfReset: 0, state: "等待数据",
  })
  let m
  if ((m = line.match(/^--- Epoch (\d+)/))) {
    live.epoch = parseInt(m[1], 10)
  } else if ((m = line.match(/\[SYNC\].*?\(nSat=(\d+)\).*?\(nSat=(\d+)\)/))) {
    live.baseSat = parseInt(m[1], 10)
    live.rovSat = parseInt(m[2], 10)
  } else if ((m = line.match(/\[REF\]\s+GPS RefSat =\s*(\S+)\s+BDS RefSat =\s*(\S+)/))) {
    live.refG = m[1]
    live.refC = m[2]
  } else if ((m = line.match(/\[FIXED\].*?\|b\|=\s*([\d.]+)\s*m\s+ratio=\s*([\d.]+)/))) {
    live.state = "固定解"
    live.baseLen = parseFloat(m[1])
    live.ratio = parseFloat(m[2])
    live.fixed++
  } else if ((m = line.match(/\[FLOAT\].*?\|b\|=\s*([\d.]+)\s*m\b/))) {
    // 注意：FLOAT 的 Sigma0 在下一行（以空格缩进），单独匹配
    live.state = "浮点解"
    live.baseLen = parseFloat(m[1])
  } else if ((m = line.match(/^\s*Sigma0=\s*([\d.]+)/))) {
    live.sigma0 = parseFloat(m[1])
  } else if ((m = line.match(/\[KF \]/))) {
    live.kfReset = (live.kfReset || 0) + 1
  } else if (/SPP FAILED/i.test(line)) {
    live.state = line.includes("BASE") ? "基站SPP失败" : "流动站SPP失败"
  }
  if (live.epoch > 0) {
    live.fixRate = live.fixed / live.epoch
  }
}

function createApi({ exeFile, iniFile, runsDir, appName }) {
  function finishRun(status) {
    if (!current) return
    current.status = status
    current.child = undefined
    current.files = listRunFiles(current.dir)
    // 结束汇总：把关键计数合成一行日志，输出窗格与历史记录都能直接看到
    const lv = current.stats.live || {}
    if (lv.epoch > 0) {
      const rate = lv.fixed > 0 ? (lv.fixed / lv.epoch * 100).toFixed(1) + "%" : "0%"
      const mins = ((Date.now() - new Date(current.startedAt).getTime()) / 60000).toFixed(1)
      sendSse({
        type: "log",
        line: `[GUI] 解算结束：共 ${lv.epoch} 历元，固定 ${lv.fixed} 次（固定率 ${rate}），`
          + `滤波器自动重置 ${lv.kfReset || 0} 次，用时 ${mins} 分钟`,
        runId: current.id,
      })
    }
    try {
      fs.writeFileSync(
        path.join(current.dir, "meta.json"),
        JSON.stringify({ id: current.id, startedAt: current.startedAt, status, stats: current.stats }, null, 2),
        "utf8"
      )
    } catch { /* 忽略 */ }
    sendSse({ type: "status", status, stats: current.stats, files: current.files, runId: current.id })
  }

  function startRun() {
    if (current && current.status === "running") throw new Error("已有解算任务在运行")
    if (!fs.existsSync(exeFile)) throw new Error("未找到解算程序: " + exeFile)
    if (!fs.existsSync(iniFile)) throw new Error("未找到配置文件: " + iniFile)
    fs.mkdirSync(runsDir, { recursive: true })
    const id = new Date().toISOString().replace(/[-:T]/g, "").slice(0, 14)
    const dir = path.join(runsDir, id)
    fs.mkdirSync(dir)
    fs.copyFileSync(iniFile, path.join(dir, "rtk.ini"))
    const run = { id, dir, startedAt: new Date().toISOString(), status: "running", lines: [], stats: {}, files: [] }

    const child = spawn(exeFile, [], { cwd: dir, stdio: ["pipe", "pipe", "pipe"] })
    run.child = child
    // 程序结尾有 "Press Enter to exit..."：提前写入回车，管道缓冲会保留到 getchar 读取
    // 注意：若 Nav 启动即失败（如配置路径无效），管道已断开，写入会抛 EPIPE——
    // 必须捕获，否则未处理异常会让整个 GUI 退出（表现为"点开始解算窗口消失"）
    try {
      child.stdin?.write("\n")
      child.stdin?.end()
    } catch (e) {
      run.lines.push("[警告] 无法向解算进程写入: " + (e && e.message ? e.message : e))
    }

    // 子进程输出按行解码（防止多字节字符跨 chunk 被截断）
    // Nav 两端均以 UTF-8 输出中文（源码按 UTF-8 编译），统一按 UTF-8 解码
    const NAV_ENCODING = "utf-8"
    let buf = Buffer.alloc(0)
    const onData = (chunk) => {
      buf = Buffer.concat([buf, chunk])
      let idx
      while ((idx = buf.indexOf(0x0a)) >= 0) {
        const line = iconv.decode(buf.subarray(0, idx + 1), NAV_ENCODING).replace(/\r?\n$/, "")
        buf = buf.subarray(idx + 1)
        if (!line.trim()) continue
        run.lines.push(line)
        if (run.lines.length > 5000) run.lines.splice(0, 1000)
        if (/^\s*Float\s+n=\d+/.test(line)) run.stats.float = line.trim()
        if (/^\s*Fixed\s+n=\d+/.test(line)) run.stats.fixed = line.trim()
        updateLiveStats(run, line)
        sendSse({ type: "log", line, runId: id, stats: run.stats })
      }
    }
    child.stdout?.on("data", onData)
    child.stderr?.on("data", onData)
    child.on("error", (err) => {
      sendSse({ type: "log", line: "[启动失败] " + err.message, runId: id })
      finishRun("failed")
    })
    child.on("exit", (code) => {
      if (buf.length) {
        const line = iconv.decode(buf, NAV_ENCODING).trim()
        if (line) { run.lines.push(line); sendSse({ type: "log", line, runId: id }) }
        buf = Buffer.alloc(0)
      }
      finishRun(code === 0 ? "success" : "failed")
    })
    current = run
    sendSse({ type: "status", status: "running", runId: id, startedAt: run.startedAt, dir: run.dir })
    return run
  }

  function readTextFile(file) {
    const buf = fs.readFileSync(file)
    // 结果文件可能是 UTF-8（csv）或 GBK（txt 中文表头）：先试 UTF-8，失败按 GBK
    let text
    try {
      text = new TextDecoder("utf-8", { fatal: true }).decode(buf)
    } catch {
      text = iconv.decode(buf, "gbk")
    }
    const all = text.split(/\r?\n/)
    const MAX = 400
    return { content: all.slice(0, MAX).join("\n"), totalLines: all.length, truncated: all.length > MAX }
  }

  return function apiMiddleware(req, res, next) {
    const handle = (fn) => {
      try { fn() } catch (e) {
        res.statusCode = 400
        res.end(JSON.stringify({ error: e?.message || String(e) }))
      }
    }
    const json = (obj) => {
      res.setHeader("Content-Type", "application/json; charset=utf-8")
      res.end(JSON.stringify(obj))
    }

    const url = new URL(req.url || "/", "http://localhost")
    const p = url.pathname

    if (p === "/api/env") {
      return handle(() => json({
        app: appName,
        exe: exeFile, exeExists: fs.existsSync(exeFile),
        configPath: iniFile, configExists: fs.existsSync(iniFile),
        runsDir,
      }))
    }

    if (p === "/api/config" && req.method === "GET") {
      return handle(() => {
        const d = loadIni(iniFile)
        json({
          path: d.path, exists: d.exists,
          entries: d.entries.map(({ key, value }) => ({ key, value })),
          savedAt: d.exists ? fs.statSync(iniFile).mtime.toISOString() : null,
        })
      })
    }

    if (p === "/api/config" && req.method === "POST") {
      let body = ""
      req.on("data", (c) => { body += c.toString() })
      req.on("end", () => handle(() => {
        const { values } = JSON.parse(body || "{}")
        if (!values || typeof values !== "object") throw new Error("请求格式错误")
        saveIni(iniFile, values)
        json({ ok: true, savedAt: fs.statSync(iniFile).mtime.toISOString() })
      }))
      return
    }

    if (p === "/api/fs" && req.method === "GET") {
      return handle(() => {
        const target = url.searchParams.get("path") || path.dirname(exeFile)
        const st = fs.statSync(target) // 不存在会抛错
        const dir = st.isDirectory() ? target : path.dirname(target)
        const entries = fs.readdirSync(dir, { withFileTypes: true })
        const dirs = entries.filter((e) => e.isDirectory()).map((e) => e.name).sort()
        const files = entries.filter((e) => e.isFile()).map((e) => {
          const fst = fs.statSync(path.join(dir, e.name))
          return { name: e.name, size: fst.size, mtime: fst.mtime.toISOString() }
        }).sort((a, b) => a.name.localeCompare(b.name))
        json({ path: dir, parent: path.dirname(dir) === dir ? null : path.dirname(dir), dirs, files })
      })
    }

    if (p === "/api/run" && req.method === "POST") {
      return handle(() => {
        const run = startRun()
        json({ ok: true, runId: run.id, dir: run.dir })
      })
    }

    if (p === "/api/run/stop" && req.method === "POST") {
      return handle(() => {
        if (!current || current.status !== "running") throw new Error("没有运行中的任务")
        current.child?.kill()
        json({ ok: true })
      })
    }

    if (p === "/api/run/events" && req.method === "GET") {
      res.setHeader("Content-Type", "text/event-stream; charset=utf-8")
      res.setHeader("Cache-Control", "no-cache")
      res.setHeader("Connection", "keep-alive")
      res.flushHeaders?.()
      sseClients.add(res)
      res.write(`data: ${JSON.stringify({
        type: "hello",
        current: current ? { id: current.id, status: current.status, startedAt: current.startedAt } : null,
      })}\n\n`)
      req.on("close", () => sseClients.delete(res))
      return
    }

    if (p === "/api/runs" && req.method === "GET") {
      return handle(() => {
        if (!fs.existsSync(runsDir)) return json([])
        const runs = fs.readdirSync(runsDir).map((id) => {
          const dir = path.join(runsDir, id)
          let meta = {}
          try { meta = JSON.parse(fs.readFileSync(path.join(dir, "meta.json"), "utf8")) } catch { /* 无 meta */ }
          return {
            id,
            startedAt: meta.startedAt || fs.statSync(dir).mtime.toISOString(),
            status: meta.status || "unknown",
            stats: meta.stats || {},
            files: listRunFiles(dir),
          }
        }).sort((a, b) => b.id.localeCompare(a.id)).slice(0, 20)
        json(runs)
      })
    }

    if (p === "/api/run/file" && req.method === "GET") {
      return handle(() => {
        const id = url.searchParams.get("id") || ""
        const name = url.searchParams.get("name") || ""
        if (!/^[\w.-]+$/.test(name)) throw new Error("非法文件名")
        const file = path.join(runsDir, id, name)
        if (!fs.existsSync(file)) throw new Error("文件不存在")
        json({ name, ...readTextFile(file) })
      })
    }

    next?.()
  }
}

// 生产静态资源服务（Electron 打包模式）：dist 目录 + SPA 回退到 index.html
const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".ico": "image/x-icon",
  ".json": "application/json; charset=utf-8",
  ".woff2": "font/woff2",
}

function createStatic(staticDir) {
  return function staticMiddleware(req, res) {
    const url = new URL(req.url || "/", "http://localhost")
    let pathname = decodeURIComponent(url.pathname)
    if (pathname.endsWith("/")) pathname += "index.html"
    let file = path.join(staticDir, pathname)
    if (!file.startsWith(staticDir) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
      // SPA 回退
      file = path.join(staticDir, "index.html")
    }
    if (!fs.existsSync(file)) {
      res.statusCode = 404
      return res.end("not found")
    }
    res.setHeader("Content-Type", MIME[path.extname(file).toLowerCase()] || "application/octet-stream")
    fs.createReadStream(file).pipe(res)
  }
}

module.exports = { createApi, createStatic }
