# 关于本目录

本目录（`libraries/`）是**第三方库原样副本**，不是本工程的自研代码。

- `zf_common/`、`zf_driver/`、`zf_device/`、`zf_components/`：
  逐飞科技 TC264 开源库 **V3.4.3**，GPL-3.0-or-later
- `infineon_libraries/`：
  英飞凌 iLLD **1.0.1.11.0**，Boost Software License 1.0

## 贡献代码请勿修改本目录

改动应放在 `code/` 或 `user/`。在这里提交改动会导致：

- 后续无法与上游逐字对比、无法跟进上游更新；
- 许可归属混乱。

若确实必须修改库文件，请在仓库根 `NOTICE.md` 的「对本库的改动」表中补充一行说明，
并保留文件内的原有版权声明。

## 编码注意

逐飞上游源文件为 **GBK** 编码，本工程**保持原样未做转换**。
工程编码规则见 `.settings/org.eclipse.core.resources.prefs`（工程默认 UTF-8，`libraries/` 指定 GBK）。

## 许可

许可正文见 `libraries/LICENSE`；完整归属说明见仓库根 `NOTICE.md`。
