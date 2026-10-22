// Electron 主进程（CommonJS 版：最兼容的加载方式，日志先于一切依赖写入）：
// 内嵌 HTTP 服务（共享 API + 前端静态资源）并创建桌面窗口。
// 打包后目录结构：
//   GNSS-RTK-Console/
//     GNSS-RTK-Console.exe        ← 双击启动
//     gnss-console.log            ← 启动诊断日志（启动失败时查这里）
//     gnss-runs/                  ← 每次解算的运行目录（自动创建，随软件带走）
//     resources/
//       nav/Nav.exe               ← 内置解算程序
//       nav/rtk.ini               ← 配置文件（界面直接读写）
//       app.asar/dist/            ← 前端界面
"use strict"

const path = require("path")
const fs = require("fs")

// ---------------- 启动诊断日志（写入 exe 旁边，排查“双击无响应”用） ----------------
// 这段代码只依赖 Node 内置模块，且在 require("electron") 之前执行：
// 只要进程起来执行了 JS，就必定会产生日志；没有日志 = 进程根本没运行（查杀软）
const LOGFILE = path.join(path.dirname(process.execPath), "gnss-console.log")
function log() {
  const line = "[" + new Date().toISOString() + "] " +
    Array.prototype.map.call(arguments, (a) => {
      if (typeof a === "string") return a
      try { return JSON.stringify(a) } catch { return String(a) }
    }).join(" ") + "\n"
  try { fs.appendFileSync(LOGFILE, line, "utf8") } catch { /* 忽略 */ }
}
log("=== main.cjs 开始执行 === execPath =", process.execPath, "cwd =", process.cwd(),
  "versions =", JSON.stringify(process.versions))

let app, BrowserWindow, shell
try {
  ;({ app, BrowserWindow, shell } = require("electron"))
  log("require(electron) 成功")
} catch (e) {
  log("require(electron) 失败:", e && e.stack ? e.stack : e)
  throw e
}

// ---------------- 兼容性开关（必须在 app ready 之前设置） ----------------
// 部分机器（尤其装了 360 / 火绒等安全软件的环境）上，Chromium 的 GPU 进程
// 或渲染子进程一启动就被拦截或崩溃，表现为 render-process-gone
// （reason=launch-failed, exitCode=18）：窗口闪一下就消失。
// 这里强制关闭硬件加速和沙箱，用软件渲染换取在这些机器上的稳定启动。
// 副作用仅是界面帧率略低，对本工具类界面没有可感知影响。
try {
  app.disableHardwareAcceleration()
  app.commandLine.appendSwitch("disable-gpu")
  app.commandLine.appendSwitch("disable-gpu-sandbox")
  app.commandLine.appendSwitch("no-sandbox")
  log("已启用兼容模式：disable-gpu / disable-gpu-sandbox / no-sandbox")
} catch (e) {
  log("设置兼容开关失败（继续启动）:", e && e.message ? e.message : e)
}

let createApi, createStatic
try {
  ;({ createApi, createStatic } = require("../server/api.cjs"))
  log("api.cjs 加载成功")
} catch (e) {
  log("api.cjs 加载失败:", e && e.stack ? e.stack : e)
  throw e
}

// 未处理异常只记日志、不退出：解算任务启动失败等异常不应把整个窗口关掉
process.on("uncaughtException", (e) => { log("UNCAUGHT EXCEPTION:", e && e.stack ? e.stack : e) })
process.on("unhandledRejection", (e) => { log("UNHANDLED REJECTION:", e && e.stack ? e.stack : e) })

// 路径解析：打包 / 开发 两种布局
// 跨平台：Windows 下解算程序为 Nav.exe，Linux 下为 Nav（无扩展名）
const NAV_BIN = process.platform === "win32" ? "Nav.exe" : "Nav"
let exeFile, iniFile, runsDir, staticDir
if (app.isPackaged) {
  const navDir = path.join(process.resourcesPath, "nav")
  exeFile = path.join(navDir, NAV_BIN)
  iniFile = path.join(navDir, "rtk.ini")
  runsDir = path.join(path.dirname(process.execPath), "gnss-runs")
  staticDir = path.join(process.resourcesPath, "app.asar", "dist")
} else {
  const root = path.resolve(__dirname, "../..") // gnss-gui 位于工作区根目录
  exeFile = path.join(root, "x64", "Release", NAV_BIN)
  iniFile = path.join(root, "gnss", "rtk.ini")
  runsDir = path.join(root, "gnss-gui-runs")
  staticDir = path.join(root, "gnss-gui", "dist")
}

log("isPackaged =", app.isPackaged, "resourcesPath =", process.resourcesPath)
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
      let renderRetry = 0
      mainWindow.webContents.on("render-process-gone", (e, details) => {
        log("渲染进程退出:", JSON.stringify(details), " 重试次数:", renderRetry)
        // 渲染进程意外退出（被杀软拦截、GPU 崩溃等）时自动重载，最多 2 次
        if (mainWindow && renderRetry < 2 &&
            details.reason !== "clean-exit" && details.reason !== "killed") {
          renderRetry++
          setTimeout(() => {
            if (mainWindow && !mainWindow.isDestroyed()) {
              log("尝试重载页面（第", renderRetry, "次）")
              mainWindow.reload()
            }
          }, 800)
        }
      })
      mainWindow.on("closed", () => { log("窗口已关闭"); mainWindow = null })
      // 外部链接交给系统浏览器
      mainWindow.webContents.setWindowOpenHandler(({ url }) => {
        shell.openExternal(url)
        return { action: "deny" }
      })
      mainWindow.loadURL("http://127.0.0.1:" + port + "/").then(
        () => log("loadURL 已提交"),
        (e) => log("loadURL 失败:", e && e.message ? e.message : e)
      )
    } catch (e) {
      log("创建窗口异常:", e && e.stack ? e.stack : e)
    }
  }

  app.whenReady().then(() => {
    log("app ready")
    if (!fs.existsSync(staticDir)) {
      log("缺少前端资源:", staticDir)
      const { dialog } = require("electron")
      dialog.showErrorBox("缺少前端资源", "未找到 dist/，请先运行 npm run build")
      app.quit()
      return
    }
    const api = createApi({ exeFile, iniFile, runsDir, appName: "GNSS RTK 解算控制台" })
    const statics = createStatic(staticDir)
    const server = require("http").createServer((req, res) => {
      api(req, res, () => statics(req, res))
    })
    server.on("error", (e) => log("HTTP 服务错误:", e && e.message ? e.message : e))
    server.listen(0, "127.0.0.1", () => {
      const port = server.address().port
      log("HTTP 服务监听 127.0.0.1:" + port)
      createWindow(port)
    })
  }).catch((e) => log("whenReady 异常:", e && e.stack ? e.stack : e))

  app.on("window-all-closed", () => { log("全部窗口关闭，退出"); app.quit() })
}
