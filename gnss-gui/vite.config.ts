import path from "path"
import { createRequire } from "node:module"
import react from "@vitejs/plugin-react"
import { defineConfig, type Plugin } from "vite"
import { inspectAttr } from 'kimi-plugin-inspect-react'

// 共享 API 模块（CommonJS）：用 createRequire 在运行时加载，
// 避免被 vite 配置打包器内联（内联后 CJS 的 require 内置模块会失败）
// @ts-ignore 无类型声明的 CJS 模块
const { createApi } = createRequire(import.meta.url)(path.resolve(__dirname, "server", "api.cjs"))

// 开发模式：API 与 vite dev server 同进程（打包模式见 electron/main.mjs）
// 工作区布局（.. 即工作区根目录）：
//   gnss/rtk.ini          配置文件（界面直接读写）
//   x64/Release/Nav.exe   解算程序
//   gnss-gui-runs/<id>/   每次解算的运行目录
const ROOT = path.resolve(__dirname, "..")

function apiPlugin(): Plugin {
  const api = createApi({
    exeFile: path.join(ROOT, "x64", "Release", "Nav.exe"),
    iniFile: path.join(ROOT, "gnss", "rtk.ini"),
    runsDir: path.join(ROOT, "gnss-gui-runs"),
    appName: "GNSS RTK 解算控制台 (dev)",
  })
  return {
    name: "gnss-console-api",
    configureServer(server) {
      server.middlewares.use((req, res, next) => api(req, res, next))
    },
  }
}

// https://vite.dev/config/
export default defineConfig({
  base: './',
  plugins: [inspectAttr(), react(), apiPlugin()],
  server: {
    port: 7100,
  },
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
});
