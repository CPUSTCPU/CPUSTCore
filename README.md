<div align="center">

# CPUSTCore

**面向 FPGA 的 32 位 LoongArch 乱序处理器核**

[![Test](https://github.com/CPUSTCPU/CPUSTCore/actions/workflows/test.yml/badge.svg)](https://github.com/CPUSTCPU/CPUSTCore/actions/workflows/test.yml)
[![Chisel](https://img.shields.io/badge/Chisel-6.7.0-DC322F)](https://www.chisel-lang.org/)
[![Scala](https://img.shields.io/badge/Scala-2.13.16-CB0000)](https://www.scala-lang.org/)

[English](README.en.md)

</div>

CPUSTCore 是一个使用 Chisel 实现、可综合的 32 位 LoongArch 乱序处理器核，面向
FPGA 实现与处理器微架构研究。仓库包含处理器源码、ChipLab 集成顶层、仿真顶层和
定向 Chisel 测试。

设计资料： [设计报告](docs/design.pdf) · [可编辑架构图](docs/drawio/CPUSTCore.drawio)

## 设计亮点

| | |
| --- | --- |
| **宽发射乱序后端** | 3 路整数、2 路访存发射，33 项 ROB，支持分支恢复与精确异常。 |
| **分层分支预测** | 2-way BTB 与 2048 项 Agree 提供快速预测，4 x 512 项 MiniTAGE 补充长历史相关性。 |
| **非阻塞访存系统** | Load replay、Store forwarding、4 项 MSHR、2 项写回缓冲和可选统一 L2 解耦 miss 与流水线。 |
| **面向 FPGA 的存储组织** | LVT PRF、Banked 队列和 Xilinx RAM wrapper 降低多端口存储与高扇出路径的实现成本。 |

## 微架构规格

| 项目 | 默认配置 |
| --- | --- |
| ISA / 数据宽度 | 32 位 LoongArch / 32 位整数数据通路 |
| 取指 / 译码 / 提交 | 4 / 3 / 3 指令每周期 |
| 发射 | 3 路整数 + 2 路访存 |
| ROB / 物理寄存器 | 33 / 64 项 |
| IntIQ / MemIQ | 9 / 6 项 |
| Load Queue / Store Queue | 10 / 8 项 |
| 分支预测 | 2-way BTB、2048 项 Agree、4 x 512 项 MiniTAGE、8 项 RAS |
| L1 ICache | 16 KiB，4-way，64 B cache line |
| L1 DCache | 8 KiB，2-way，双请求端口，64 B cache line |
| L2 Cache | 默认 64 KiB，4-way，可在 elaboration 时关闭 |

容量和开关的权威定义位于
[`Parameters.scala`](src/main/scala/Config/Parameters.scala) 与
[`MemoryConfig.scala`](src/main/scala/MemorySystem/MemoryConfig.scala)。

## 架构概览

![CPUSTCore 微架构概览](docs/architecture.svg)

Archify 源文件：[architecture.json](docs/architecture.json)。详细设计图：[CPUSTCore.drawio](docs/drawio/CPUSTCore.drawio)。

## 快速开始

### 环境

- JDK 17 或更新版本
- sbt 1.12.9（由 [`project/build.properties`](project/build.properties) 固定）
- 首次构建时可访问 Maven 依赖仓库
- Vivado 2023.2（仅 FPGA 综合、实现和 ChipLab 集成需要）

项目使用 Chisel 6.7.0 和 Scala 2.13.16，完整依赖见 [`build.sbt`](build.sbt)。

### 获取与验证

```bash
git clone https://github.com/CPUSTCPU/CPUSTCore.git
cd CPUSTCore
sbt --no-colors test
```

当前测试覆盖分支预测、访存 replay/flush、MSHR 合并、ICache/TLB 响应配对和
Load recovery 等关键行为。

## 仓库结构

```text
.
├── src/main/scala/
│   ├── Backend/              # Rename、Issue、Execute、ROB、Writeback
│   ├── Frontend/             # IFU、FTQ、IBuffer 与分支预测
│   ├── MemorySystem/         # Cache、LSQ、TLB、MSHR 与总线接口
│   ├── Decode/               # 预译码、主译码与功能单元译码
│   ├── Config/               # 核心参数和公共硬件函数
│   └── Utils/                # 队列、RAM 和 FPGA wrapper
├── src/test/scala/           # Chisel 定向测试
├── docs/
│   ├── architecture.svg       # GitHub 展示用架构概览
│   ├── architecture.json      # Archify 架构图源文件
│   ├── drawio/               # 可编辑架构图
│   │   └── CPUSTCore.drawio
│   └── design.pdf            # 设计报告
```

物理目录按微架构组织；Scala 源码统一使用 `CPUSTC.*` 顶层包名。

## FPGA 与 ChipLab

完整 ChipLab 功能、性能、综合实现和 bitstream 流程依赖外部 ChipLab 工程，
不包含在本仓库中。
