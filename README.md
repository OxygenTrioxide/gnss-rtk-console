# GNSS RTK 解算控制台

**武汉大学《卫星导航算法与程序设计》课程作业 / 课程设计**

GNSS 动态相对定位（RTK）解算软件：C++ 解算核心 + Electron 图形界面。基于北斗/GPS 双系统观测数据，实现 SPP 单点定位、载波相位差分（RTK）、LAMBDA 模糊度固定与解算结果精度评定。

> 关键词：武汉大学、卫星导航算法与程序设计、GNSS、北斗、RTK、载波相位差分、LAMBDA、课程设计

## 功能特性

- **多数据源**：观测文件（RINEX/二进制）、网络实时流（TCP）、串口实时流
- **解算模式**：EKF 卡尔曼滤波 / LSQ 最小二乘
- **对流层模型**：Hopfield / Saastamoinen（配置文件可选）
- **模糊度固定**：LAMBDA 降相关搜索 + 三项检验（基线一致性 χ² / 单位权中误差 / ratio）
- **结果输出**：SPP / 浮点解 / 固定解，支持 txt / csv 格式，可附加真值 E/N/U 差值对比与统计摘要
- **原始数据录制**：网络/串口模式可把基站、流动站原始数据流录为 .bin 供事后回放分析
- **实时状态流**：每历元输出 state / 基线长 / ratio / Sigma0 / 卫星数到 solve_status.csv
- **图形界面**：配置编辑（rtk.ini）、实时日志（错误标红）、运行历史与精度统计

## 目录结构

```
gnss/          C++ 解算核心（Visual Studio 工程）
  src/         解算源码（SPP、RTK、LAMBDA、OEM7 解码等）
  include/     头文件
  rtk.ini      解算配置文件（含中文注释）
gnss-gui/      Electron 图形界面
  src/         React 前端
  electron/    主进程
  server/      本地 HTTP API（读写 rtk.ini、启动 Nav.exe）
代码与课件要求复核报告（第二轮）.md   代码复核记录
代码与课件要求对照审查报告.md         首轮对照审查
进度与下一步计划.md                   开发进度记录
面试题/                               面试问答整理
```

## 构建

**解算核心（Windows / Visual Studio）**：

```
MSBuild gnss\gnss.vcxproj /p:Configuration=Release /p:Platform=x64
```

产物：`gnss\x64\Release\Nav.exe`

**图形界面**：

```
cd gnss-gui
npm install
npm run build
```

## 使用

图形界面应用（GNSS-RTK-Console）在界面中编辑配置、选择数据源后点击「开始解算」即可；
界面直接读写 `rtk.ini`，全部配置项均可视化修改。命令行方式直接运行 `Nav.exe`，
读取同目录下 `rtk.ini` 开始解算。

## 数据源示例

- 基站：47.114.134.129:7190（工学部）
- 流动站：8.148.22.229:7201（测绘学院楼顶）
- 基站精确坐标由数据流 bestpos 报文解码自动获取
