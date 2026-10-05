# 发布记录

本文记录本仓库（快照式发布）的来源、验证结果与已知偏差。**所有结论均附实测数据**。

## 1. 快照来源

| 项 | 值 |
|---|---|
| 私有开发仓库 | `git@github.com:shiliu153/TC264.git`（不公开） |
| 本仓库 | `https://github.com/shiliu153/TC264-public` |
| 本分支 | `feat/20260718-adjustable-power-supply` |
| 树来源 | 私有仓库提交 **`f62f1f9`**（2026-07-18，"feat: add adjustable power supply device library"） |
| 公开历史深度 | **1 个提交**（orphan 快照，无开发历史） |
| 私有仓库开发历史 | 127 次提交（不公开） |
| 本分支独有模块 | `user/watchdog.c/.h`、`libraries/zf_device/zf_device_adjustable_power.c/.h` |
| 本分支不含 | `user/calib_uart.c/.h`（仅在 `feat/20260711-encoder-turn-exit` 上） |

**为什么用全新历史**：私有仓库的历史中包含一度被提交过的 WiFi 口令（提交 `4a3d763`）
与内部开发文档。这类内容即使当前文件已清理，仍可通过 `git log -S` 从历史 blob 中取出，
因此必须斩断历史而非仅清理工作区。

## 2. 验证结果

### V1 自研代码零逻辑改动 —— 通过（1 处有意偏差）

方法：对 `user/`、`code/` 下每个 `.c/.h` 文件，去除**块注释、行注释与全部空白**后
取 SHA256 前 16 位作为「token 指纹」，与基线逐一比对。注释与行尾变化不影响指纹，
**任何代码改动必然改变指纹**。

| 项 | 结果 |
|---|---|
| 参与比对文件数 | 39 |
| 指纹一致 | 38 |
| 有意偏差 | 1 —— `user/wifi_cfg.h` |

**`user/wifi_cfg.h` 的偏差是有意的**：其中的 WiFi SSID 与口令被替换为
`YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD` 占位符（见 §3）。这是本次唯一的源码语义改动。

基线中的该文件同步做了同样替换，因此除该文件外 V1 是全等比对。

### V2 无遗漏文件 —— 通过

| 目录 | 基线 | 公开树 | 结果 |
|---|---:|---:|---|
| `code/` | 9 | 9 | ✅ |
| `user/` | 34 | 34 | ✅ |
| `libraries/` | 636 | 638 | ✅（+2 = 新增 `libraries/LICENSE` 与 `libraries/doc/PUBLIC-RELEASE-NOTICE.md`） |

另做**逐文件清单比对**：基线中无任何文件在公开树里缺失；公开树相对基线只多
`libraries/LICENSE` 与 `libraries/doc/PUBLIC-RELEASE-NOTICE.md` 两项。

### V1b / V5b（本分支基线为 `f62f1f9`）

- **V5b 第三方库未被改动**：对 `libraries/` 下全部 **636** 个文件逐一 SHA256 比对，
  **636/636 逐字节一致**。
- **V1 零逻辑改动**：与分支一同理，`user/`+`code/` 的 42 个 `.c/.h` 文件做 token 指纹比对；
  `user/wifi_cfg.h` 是唯一有意偏差（凭据替换为占位符）。

### V3 无敏感信息残留 —— 通过

方法：`tools/scan_secrets.py` 遍历仓库**全部对象**（`git cat-file --batch-all-objects`，
含悬垂对象，比 `git log -S` 更彻底）与工作区文件。

检测项（hex 编码存储，避免扫描器自匹配）：WiFi 口令、WiFi SSID、学号/用户名、
两种形态的绝对路径、私有工程目录名。

结果：`通过：全部历史与工作区均无敏感串`（退出码 0）。

### V4 编码分布正确 —— 通过

| 范围 | 结果 |
|---|---|
| `user/` + `code/`（39 个 `.c/.h`） | 全部 UTF-8 无 BOM ✅ |
| `libraries/zf_*`（113 个 `.c/.h`） | GBK 99 / UTF-8 14 —— 保持上游原样 ✅ |
| `libraries/infineon_libraries/`（469 个 `.c/.h`） | UTF-8 465 / GBK 2 —— 保持上游原样 ✅ |

工程编码规则见 `.settings/org.eclipse.core.resources.prefs`（工程默认 UTF-8，`libraries/` 指定 GBK）。

### V5 第三方库未被改动 —— 通过

方法：对 `libraries/` 下全部 634 个文件逐一做 SHA256 比对。

结果：**634/634 逐字节一致**，唯一新增是 `libraries/LICENSE`。

### V6 许可完整性 —— 通过

| 检查项 | 结果 |
|---|---|
| 根 `LICENSE` 为 GPL-3.0 全文（35,149 字节） | ✅ |
| `libraries/LICENSE` 为 GPL-3.0 全文（35,149 字节） | ✅ |
| `NOTICE.md` 披露 3 处对逐飞库的改动 | ✅ |
| `NOTICE.md` 声明自研部分版权 | ✅ |
| `NOTICE.md` 披露有意排除的内容 | ✅ |
| `NOTICE.md` 披露注释乱码现状 | ✅ |

**许可正文的来源可信度**：从 `https://www.gnu.org/licenses/gpl-3.0.txt` 取得，
并与 Git for Windows 自带的 GPL-3.0 副本做**哈希交叉验证**（两者 SHA256 相同），
而非仅凭下载来源。

## 3. 有意做出的改动（完整清单）

除下列各项外，公开树与基线**逐字节一致**：

| # | 改动 | 原因 |
|---|---|---|
| 1 | `user/wifi_cfg.h` 的 SSID/口令 → 占位符 + 顶部使用提示 | 原文件含真实 WiFi 凭据，且该口令在私有历史中 |
| 2 | 39 个自研 `.c/.h` 增加**文件级头注释**（模块、职责、依赖、风险等级、许可） | 可读性；V1 证明未触碰任何代码 |
| 3 | 自研代码行尾统一为 LF | 原为 CRLF，仓库统一 |
| 4 | `.settings/org.eclipse.core.resources.prefs` 编码规则重写 | 修掉「工程 GBK 默认 + 15 个 UTF-8 例外文件」导致的**保存即乱码**隐患 |
| 5 | `.gitignore` 重写（原为 0 字节） | 原文件为空，导致 206 个构建产物成为噪音 |
| 6 | 新增 `LICENSE`、`libraries/LICENSE`、`NOTICE.md`、`CONTRIBUTING.md`、`README.md`、`docs/`×5、`tools/scan_secrets.py` | 发布面补齐 |
| 7 | 新增 `libraries/doc/PUBLIC-RELEASE-NOTICE.md` | 提醒访客 `libraries/` 是第三方副本 |

## 4. 有意排除的内容

| 内容 | 原因 |
|---|---|
| `Debug/`（构建产物，含 `subdir.mk`） | Eclipse 自动生成；`subdir.mk` 内嵌了开发者本机的绝对路径（含用户名），属个人信息泄露点 |
| `docs/superpowers/`（31 份内部文档） | 含 AI 协作流程与内部推演记录，不适合公开 |
| `CLAUDE.md` | 同上 |
| `libraries/zf_driver/zf_driver_spi_multi.c/.h` | 2026-08-18 的未完成实验代码：从未被任何分支提交、无任何代码引用、无版权头 |
| 私有仓库的完整提交历史 | 含口令与内部文档（见 §1） |
| 根目录若干硬件参考文件 | 其内容已重写并整合进 `docs/pinout.md`；且原文有过时之处（例如称灰度 4067「未启用」，实际在用） |

## 5. 未验证项（如实声明）

| 项 | 状态 |
|---|---|
| **编译验证** | **未执行**。Tasking TriCore 编译器（`cctc`）仅随 AURIX Development Studio 提供，不在命令行环境中。本次整理**未在 IDE 中编译过**，因而无法证明公开树可构建。 |
| **实车验证** | **未执行**。需人下载到 TC264D 板验证。 |
| 上位机运行 | **未执行**。`user/pc_wifi_uart_plot_qt.py` 需 PyQt5 环境，未实测启动。 |
| JSON/YAML 类配置 | 不适用（本工程无此类文件）。 |
| 注释乱码修复 | **有意不修**。10 个文件含 U+FFFD 与「锟斤拷」乱码，属源工程既有状态；已核实**乱码仅在注释内**，代码完好。修复需猜测原文，等于编造，故只披露不修改。详见 `NOTICE.md` §4。 |

**因此：本次发布的正确性由 V1–V6 的机械验证保证，但「能否编译通过」不在保证范围内。**
改动 C 代码后请在 ADS 中编译并经实车验证。

## 6. 复现验证

在仓库根目录执行：

```bash
# 敏感信息（全历史 + 工作区）
python tools/scan_secrets.py .

# 第三方库是否被改动（需一份基线副本）
# 逐字节比对 libraries/ 下所有文件
```

V1 的 token 指纹方法：去除块注释、行注释与全部空白后取 SHA256。
`tools/` 下未附带指纹脚本——它属于一次性工具，且其路径参数依赖本地基线目录。
如需复现，按上句描述实现即可（约 10 行 Python）。
