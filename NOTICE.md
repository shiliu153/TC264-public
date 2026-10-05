# 第三方组件与归属说明

本工程是**衍生作品**：应用层与控制算法为本人编写，底层依赖两个第三方库。
按各自许可要求，出处与改动情况如实列明如下。

## 1. 逐飞科技 TC264 开源库

- 路径：`libraries/zf_common/`、`libraries/zf_driver/`、`libraries/zf_device/`、`libraries/zf_components/`
- 版本：**V3.4.3**（见 `libraries/doc/version.txt`）
- 版权：Copyright (C) 2022 SEEKFREE 逐飞科技
- 许可：**GNU General Public License v3.0 or later**
- 许可正文：`libraries/LICENSE`；另见 `libraries/doc/GPL3_permission_statement.txt`
- 上游：<https://seekfree.taobao.com/>

### 对本库的改动（逐文件披露）

| 文件 | 改动 |
|---|---|
| `libraries/zf_driver/zf_driver_spi.c` | +56 / −3 行：SPI 通讯时序相关修改 |
| `libraries/zf_common/zf_common_headfile.h` | +1 行：新增一个头文件包含 |
| `libraries/zf_device/zf_device_config.a` | 二进制被替换（提交 `b02fcc3` "Enable right wheel"） |

除上述三项外，`libraries/` 下**没有任何文件被改动**，全部保持上游原始字节。
所有文件内的版权声明均原样保留。

> **编码说明**：逐飞上游源文件为 GBK 编码，本项目**保持原样未做转换**，以便后续跟进上游更新。
> 工程编码规则见 `.settings/org.eclipse.core.resources.prefs`（工程默认 UTF-8，`libraries/` 指定 GBK）。
> 若你要把这套库并入自己的 GBK 工程，请在 IDE 里为 `libraries/` 单独设置编码。

## 2. 英飞凌 iLLD（AURIX Low-Level Driver）

- 路径：`libraries/infineon_libraries/`
- 版本：**iLLD_1_0_1_11_0**（见各文件头 `\version`）
- 版权：Copyright (c) 2018-2019 Infineon Technologies AG. All rights reserved.
- 许可：**Boost Software License - Version 1.0**
- 本工程**未做任何改动**，全部文件保持原始字节，文件头版权声明原样保留。

## 3. 本工程自研部分

- 路径：`user/`、`code/`、`docs/`、`tools/`
- 版权：Copyright (C) 2026 陈富
- 许可：**GPL-3.0-or-later**
- 规模：39 个 `.c/.h` 文件、10,019 行 C 代码；另含 3,496 行 Python 上位机（`user/pc_wifi_uart_plot_qt.py`）

**为什么整体是 GPL-3.0**：自研代码与上述 GPL-3.0 的逐飞库静态链接为同一个可执行文件，
因此整个工程必须以 GPL-3.0 发布。这不是选择，是许可条款的必然结果。

## 4. 已知的注释乱码（如实披露，未作修改）

部分源文件中的**中文注释存在乱码**，这是**源工程既有的历史状态**（某次 GBK/UTF-8 转换事故的残留），
不是本次开源整理引入的。实测共 10 个文件含 U+FFFD 替换字符（`\xef\xbf\xbd`），
另有「锟斤拷」形态的乱码：

| 文件 | 位置 |
|---|---|
| `user/cpu1_main.c`、`user/isr_config.h` | 逐飞 SEEKFREE 版权头 + 若干行尾注释 |
| `user/isr.c` | 若干行尾注释 |
| `libraries/zf_common/zf_common_headfile.h` | 逐飞 SEEKFREE 版权头 |
| `libraries/zf_driver/zf_driver_spi.c`、`zf_driver_uart.c` | 逐飞版权头 + 行尾注释 |
| `libraries/zf_device/zf_device_ips200.c`、`zf_device_key.c`、`zf_device_wifi_uart.h` | 逐飞版权头 + 行尾注释 |
| `libraries/infineon_libraries/.../IfxGtm_regdef.h` | 1 处寄存器说明注释 |

**为什么没有修**：这类乱码多数**不可逆**——原始字节已被替换字符覆盖或经过多次错误转换，
想「修好」只能靠猜测原文，那等于编造他人代码的注释。因此本次**保留原样**，只做如实披露。

**影响范围**：已逐行核实，乱码**全部位于注释内**，代码本身完好（标识符、常量、分号均为 ASCII）。
不影响编译，也不影响功能。

**但有一处需要你知情**：上述文件里的逐飞科技版权声明
（`Copyright (c) 2022 SEEKFREE 逐飞科技`）在乱码中已部分不可读。逐飞 GPL-3.0 要求保留版权声明，
本仓库通过**本文档 + `libraries/LICENSE` + `libraries/doc/` 下的逐飞原始许可声明文件**
保留了完整归属信息。`libraries/LICENSE` 系本次补齐——逐飞文件头明确要求该文件存在于
`libraries/` 下，但上游发行包中**原本缺失**。

## 6. 未收录的内容（有意排除）

- `Debug/`：Eclipse/ADS 自动生成的构建产物，且其中的 `subdir.mk` 含个人绝对路径。
- `libraries/zf_driver/zf_driver_spi_multi.c/.h`：2026-08-18 的未完成实验代码，
  从未被任何分支提交、无任何代码引用、无版权头，故不收录。
- 私有开发仓库中的完整提交历史（本仓库为**快照式发布**，每个分支仅 1 个提交）。

## 7. 快照与分支说明

本仓库只公开两个分支，各含 1 个快照提交，完整开发历史（127 次提交）不公开：

| 分支 | 特征功能 |
|---|---|
| `feat/20260711-encoder-turn-exit` | 路线元素组、中心枢轴转向、编码器里程退出、可调直道加速、校准 UART |
| `feat/20260718-adjustable-power-supply` | 看门狗喂狗、无屏/有屏双启动、双核性能监视器、`0x06` 遥测帧、可调电源驱动 |

两条线自 2026-06-01 分叉后互不包含。详见 `README.md` 的「已知限制与分支说明」。
