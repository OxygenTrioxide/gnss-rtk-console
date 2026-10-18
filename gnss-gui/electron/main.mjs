// Electron 主进程：内嵌 HTTP 服务（共享 API + 前端静态资源）并创建桌面窗口。
// 打包后目录结构：
//   GNSS-RTK-Console/
//     GNSS-RTK-Console.exe        ← 双击启动
//     gnss-console.log            ← 启动诊断日志（启动失败时查这里）
//     gnss-runs/                  ← 每次解算的运行目录（自动创建，随软件带走）
//     resources/
//       nav/Nav.exe               ← 内置解算程序
//       nav/rtk.ini               ← 配置文件（界面直接读写）
//       app.asar/dist/            ← 前端界面
import { app, BrowserWindow, shell } from "electron"
import http from "http"
import path from "path"
import fs from "fs"
import { fileURLToPath } from "url"
import { createRequire } from "module"

const here = path.dirname(fileURLToPath(import.meta.url))

// ---------------- 启动诊断日志（写入 exe 旁边，排查“双击无响应”用） ----------------
// 注意：log() 必须在 require api.cjs 之前定义并先输出一行，
// 才能区分“模块加载失败”与“后续逻辑失败”
const LOGFILE = path.join(path.dirname(process.execPath), "gnss-console.log")
function log(...args) {
  const line = `[${new Date().toISOString()}] ${args.map((a) => {
    if (typeof a === "string") return a
    try { return JSON.stringify(a) } catch { return String(a) }
  }).join(" ")}\n`
  try { fs.appendFileSync(LOGFILE, line, "utf8") } catch { /* 忽略 */ }
}
log("=== main.mjs 开始加载 === execPath =", process.execPath, "cwd =", process.cwd())
process.on("uncaughtException", (e) => { log("UNCAUGHT EXCEPTION:", e?.stack || e); app.exit(1) })
process.on("unhandledRejection", (e) => { log("UNHANDLED REJECTION:", e?.stack || e) })

const require2 = createRequire(import.meta.url)
let createApi, createStatic
try {
  ;({ createApi, createStatic } = require2("../server/api.cjs"))
  log("api.cjs 加载成功")
} catch (e) {
  log("api.cjs 加载失败:", e?.stack || e)
  throw e
}

// 路径解析：打包 / 开发 两种布局
let exeFile, iniFile, runsDir, staticDir
if (app.isPackaged) {
  const navDir = path.join(process.resourcesPath, "nav")
  exeFile = path.join(navDir, "Nav.exe")
  iniFile = path.join(navDir, "rtk.ini")
  runsDir = path.join(path.dirname(app.getPath("exe")), "gnss-runs")
  staticDir = path.join(process.resourcesPath, "app.asar", "dist")
} else {
  const root = path.resolve(here, "../..") // gnss-gui 位于工作区根目录
  exeFile = path.join(root, "x64", "Release", "Nav.exe")
  iniFile = path.join(root, "gnss", "rtk.ini")
  runsDir = path.join(root, "gnss-gui-runs")
  staticDir = path.join(root, "gnss-gui", "dist")
}

log("=== 启动 ===")
log("electron", process.versions.electron, "node", process.versions.node)
log("isPackaged =", app.isPackaged, "resourcesPath =", process.resourcesPath, "exe =", app.getPath("exe"))
log("exeFile =", exeFile, "exists =", fs.existsSync(exeFile))
log("iniFile =", iniFile, "exists =", fs.existsSync(iniFile))
log("staticDir =", staticDir, "exists =", fs.existsSync(staticDir))

// 单实例：重复启动时聚焦已有窗口
const gotLock = app.requestSingleInstanceLock()
if (!gotLock) {
  log("已有实例在运行，退出")
  app.quit()
} else {
  app.on("second-instance", () => {
    const w = BrowserWindow.getAllWindows()[0]
    if (w) { if (w.isMinimized()) w.restore(); w.focus() }
  })

  let mainWindow = null

  function createWindow(port) {
    log("创建窗口，端口 =", port)
    try {
      mainWindow = new BrowserWindow({
        width: 1480,
        height: 940,
        minWidth: 1100,
        minHeight: 700,
        autoHideMenuBar: true,
        title: "GNSS RTK 解算控制台",
        backgroundColor: "#ffffff",
        show: false,
        webPreferences: { contextIsolation: true },
      })
      mainWindow.once("ready-to-show", () => {
        log("窗口 ready-to-show")
        mainWindow.show()
      })
      mainWindow.webContents.on("did-fail-load", (e, code, desc) => {
        log("页面加载失败:", code, desc)
      })
      mainWindow.webContents.on("render-process-gone", (e, details) => {
        log("渲染进程退出:", JSON.stringify(details))
      })
      mainWindow.on("closed", () => { log("窗口已关闭"); mainWindow = null })
      // 外部链接交给系统浏览器
      mainWindow.webContents.setWindowOpenHandler(({ url }) => {
        shell.openExternal(url)
        return { action: "deny" }
      })
      mainWindow.loadURL(`http://127.0.0.1:${port}/`).then(
        () => log("loadURL 已提交"),
        (e) => log("loadURL 失败:", e?.message || e)
      )
    } catch (e) {
      log("创建窗口异常:", e?.stack || e)
    }
  }

  app.whenReady().then(() => {
    log("app ready")
    if (!fs.existsSync(staticDir)) {
      log("缺少前端资源:", staticDir)
      const { dialog } = require2("electron")
      dialog.showErrorBox("缺少前端资源", "未找到 dist/，请先运行 npm run build")
      app.quit()
      return
    }
    const api = createApi({ exeFile, iniFile, runsDir, appName: "GNSS RTK 解算控制台" })
    const statics = createStatic(staticDir)
    const server = http.createServer((req, res) => {
      api(req, res, () => statics(req, res))
    })
    server.on("error", (e) => log("HTTP 服务错误:", e?.message || e))
    server.listen(0, "127.0.0.1", () => {
      const port = server.address().port
      log("HTTP 服务监听 127.0.0.1:" + port)
      createWindow(port)
    })
  }).catch((e) => log("whenReady 异常:", e?.stack || e))

  app.on("window-all-closed", () => { log("全部窗口关闭，退出"); app.quit() })
}
