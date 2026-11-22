# OP5 雷达–视觉融合（ChatGPT 调试最终版归档）

Orange Pi 5 上的雷达 + 视觉融合识别项目。本仓库从 ChatGPT 项目「AI视觉」、对话「补充保存自动回中需求」中导出的安装包整理而成，按版本顺序合并为**当前最新可运行源码树**，并把原始 zip 原样留在 `packages/` 备查。

本仓库只包含 8100 融合服务、8101 云台监护桥、8090 融合开关注入代码，以及标定/影子测试工具。8081 / 8090 主体 / 8092 / 8093 的源码不在这些安装包中（见下方「已知缺失文件」）。

> **安全说明：** 这些程序多数是只读影子融合或需人工 PIN 授权的监护控制。文档反复强调：雷达点云不能证明人物身份；不得作为无人值守跟踪、自主避障、底盘闭环或危险载荷控制的依据。8101 在人工看护、无载荷条件下测试。

## 端口与服务架构

```
触摸屏 8090 HUD  ──(可选：融合开关)──►  8101 融合监护桥
        │                                    │
        │ 原视觉跟踪滑块                      │ 仅转发 8093 已有的 START / PULSE / STOP
        ▼                                    ▼
     8093 云台 V2（PREDICTIVE_FAST_V2）   实际舵机控制仍全部由 8093 完成
        ▲
        │ 姿态 /api/status
        │
8081 YOLO 视觉 ──► 8100 只读融合服务 ◄── 8092 COIN-D6 雷达 /api/scan
                     │
                     └── 网页与 /api/fusion、/api/health（默认 8100）
```

| 端口 | 角色 | 本仓库是否有源码 | 说明 |
|------|------|------------------|------|
| 8081 | YOLO 视觉检测 / 选中目标 | **缺失** | 8100 只读拉取 `/state` 等接口 |
| 8090 | 触摸屏 HUD（跟踪滑块、回中、停止） | **主体缺失**，仅有融合开关注入包 | 在原 `dual_overlay_switch.py` 上增强 |
| 8092 | COIN-D6 雷达扫描 | **缺失** | 8100 只读拉取 `/api/scan`，不占用串口 |
| 8093 | 云台高速预测 V2 | **缺失** | 已实测版本保持不动；8101 只调用其原有接口 |
| 8100 | 雷达–视觉只读融合 | **有**（`fusion_8100/`，最新 V3.1.3 核心 + V3.1.2 服务） | systemd：`op5-v3-fusion-8100` |
| 8101 | 融合监护桥（人工授权后才向 8093 发跟踪令） | **有**（`bridge_8101/`，STAGE1） | systemd：`op5-v3-fusion-gimbal-8101`，默认只监听 127.0.0.1 |

开机时 8090 / 8101 可自启，但云台跟踪不会自行启动；8090 融合开关每次打开网页默认关闭。

## 目录说明

```
op5-vision/
├── README.md
├── .gitignore
├── TEST_RESULTS.txt          # 本次合并后的离线测试记录
├── packages/                 # 原始 11 个 zip 原样留档 + SHA256
├── fusion_8100/              # 8100 当前最新源码树
│   ├── fusion_core.py        # V3.1.3（深度歧义防护）
│   ├── service.py            # V3.1.2（采集时序 / 后台 tick）
│   ├── dashboard.html        # V3.1
│   ├── calibrate.py / diagnose.py / config.example.json   # 来自 V3，后续未改
│   ├── tests/                # test_dynamic.py、test_v313.py、test_v3_original.py
│   ├── history/v3/           # V3 原始 core/service/dashboard 及配套单元测试
│   ├── history/v3.1/         # V3.1 core/service，以及 V3.1.1 的 service 备份
│   ├── docs/                 # 各版本使用说明与 manifest
│   └── VERSION
├── bridge_8101/              # 8101 阶段 1 监护桥
├── toggle_8090/              # 8090「是否加入雷达融合」触摸开关注入包
└── tools/
    ├── calibration/          # 临时零位、影子零位对照、静态标定诊断、现场出入验证
    ├── shadow/               # 遮挡 / 负样本影子测试（实机交互，需现场传感器）
    └── replay/               # 原始扫描录制与离线对照
```

设备上安装后的常见路径（安装脚本写死，不是本仓库目录）：

- 8100 运行目录：`/home/orangepi/op5_v3_fusion/`
- 8101 运行目录：`/home/orangepi/op5_v3_fusion_bridge/`（安装时会在此生成 `credentials.json`，切勿提交）
- 8090 主程序：`/home/orangepi/dual_overlay_switch.py`

## 版本历史与合并判断

合并顺序：V3 → V3.1 → V3.1.1 → V3.1.2 → V3.1.3。各补丁包均提供**完整文件**（不是 fragment patch）。已用 `diff` 和包内 `manifest.json` 的 SHA256 核对，确认是整文件替换，且哈希与清单一致。

| 包名 | 内容 | 合并结果 |
|------|------|----------|
| `OP5_V3_RADAR_VISION_FUSION_V1` | 8100 融合服务基础版：`fusion_core.py`、`service.py`、`dashboard.html`、标定/诊断、离线测试 | 作为起点；`calibrate.py`、`diagnose.py`、`config.example.json` 保留为当前文件（后续包未改它们） |
| `OP5_V3_1_DYNAMIC_ASSOC_V1` | 动态关联：整文件替换 `fusion_core.py`、`service.py`、`dashboard.html`；新增录制对照工具与测试 | 覆盖上述三文件；工具放入 `tools/replay/` |
| `OP5_V3_1_1_ENGINE_TICK_FIX` | 仅替换 `service.py`（后台每 0.20s tick，GET 只读缓存） | 整文件替换；V3.1 的 service 保留在 `history/v3.1/` |
| `OP5_V3_1_2_SOURCE_TIMING_FIX` | 再替换 `service.py`（视觉 140ms / 雷达 160ms / 云台 150ms，tick 100ms）+ `静态标定诊断.py` | **当前 `service.py` 来自此包**（SHA256 `1cc5921a…`） |
| `OP5_V3_1_3_DEPTH_AMBIGUITY_GUARD` | 仅替换 `fusion_core.py`（竞争深度则 `ASSOCIATION_COMPETING_DEPTHS` 并拒绝）+ `test_v313.py`、`现场出入验证.py` | **当前 `fusion_core.py` 来自此包**（SHA256 `db62197f…`） |
| `OP5_V3_FUSION_GIMBAL_STAGE1` | 独立的 8101 监护桥 | 放入 `bridge_8101/`，不与 8100 文件重叠 |
| `OP5_V3_TEMP_ZERO_REAL_V1` | 临时改 8100 `radar_zero_deg` 做四位置回归，结束后恢复 | `tools/calibration/` |
| `OP5_V3_SHADOW_ZERO_CAL_V1` | 内存中对照 351.5° 与 +6.2°，不写配置 | `tools/calibration/` |
| `OP5_V3_OCCLUSION_SHADOW_V1` | 遮挡场景影子测试 | `tools/shadow/` |
| `OP5_V3_NEGATIVE_ASSOC_SHADOW_V1` | 离场 / 替代物 / 多深度负样本影子测试 | `tools/shadow/` |
| `OP5_8090_FUSION_TOGGLE_FINAL_V1` | 8090 触屏「是否加入雷达融合」开关（最后做的包） | 放入 `toggle_8090/` |

Downloads 里还有 `OP5_V3_1_1_ENGINE_TICK_FIX (1).zip` 和 `OP5_V3_FUSION_GIMBAL_STAGE1 (1).zip`，与正本 SHA256 完全相同，未重复归档。详见 `packages/SHA256SUMS.txt`。

补充判断：

- 8101 的 `install.py` 校验的是 **V3.1** 的 `service.py` SHA256（`aa13b140…`），不是后来的 V3.1.2。若现场已先打上 V3.1.1/V3.1.2，再跑 8101 原安装器的 `--check` 可能会因哈希不一致而拒绝。这是原包约束，合并时未改安装脚本。
- 8090 开关说明写明：该包只整合操作界面，**不修复**此前纸箱遮挡人物时旧门控 5/20 错误许可；深度歧义由后续 V3.1.3 在 8100 核心侧处理。

## 安装方式

请在香橙派上使用 `packages/` 里的**原始 zip**，按各包自带说明操作，不要直接把本仓库的目录结构当成安装器的预期布局（安装脚本以 `/home/orangepi/OP5_…` 和 `/home/orangepi/op5_v3_fusion` 为准，并用 SHA256 校验）。

建议顺序（与当时调试顺序一致）：

1. 安装并启动 8100 基础包 `OP5_V3_RADAR_VISION_FUSION_V1`，完成现场标定（`calibrate.py`）。数值必须来自现场，`config.example.json` 里的示例不是设备标定值。
2. 升级动态关联 `OP5_V3_1_DYNAMIC_ASSOC_V1`，按说明重启 8100。
3. 依次打补丁：`V3.1.1` → `V3.1.2` → `V3.1.3`。每步先 `--check`，通过后再 `--apply`，再 `systemctl restart op5-v3-fusion-8100`。
4. 需要监护跟踪时再装 `OP5_V3_FUSION_GIMBAL_STAGE1`。安装器会生成一次性 PIN，不要把 PIN 或 `credentials.json` 发到网上或提交进 Git。
5. 最后按 `OP5_8090_FUSION_TOGGLE_FINAL_V1` 说明给 8090 注入融合开关。安装器要求现场 8090 已具备 `OP5_HUD_V11_REAL_INTEGRATION_JS`、`OP5_AUTO_CENTER_V2_8090_ADDITION` 和原云台滑块，否则会直接拒绝。

标定 / 影子工具按各自 `使用说明.txt` 在**云台停止、8101 未授权、底盘静止**时运行。本仓库导入过程中**没有执行**任何 `install.py`，以免改动本机或误连设备。

## 离线测试

在合并后的源码树上只跑了离线测试，未运行安装器。完整输出见 [`TEST_RESULTS.txt`](TEST_RESULTS.txt)。

| 测试 | 结果 |
|------|------|
| `fusion_8100/history/v3/test_v3.py`（35 项，对 V3 核心） | 通过 |
| `tests/test_v3_original.py`（35 项，对**最新** V3.1.3 核心） | 通过 |
| `tests/test_dynamic.py`（24 项，对 V3.1 核心） | 通过 |
| `tests/test_dynamic.py`（24 项，对**最新** V3.1.3 核心） | **22 通过，2 失败**（见下） |
| `tests/test_v313.py`（5 项，对最新核心） | 通过 |
| `bridge_8101/test_stage1.py`（22 项） | 通过 |
| `toggle_8090/test_offline.py` | 通过（JS 语法 + 未发出电机/雷达命令）。因缺少 8090 主程序快照，日志提示 `Source snapshots unavailable: core syntax checked only`，未做完整 HUD 注入回放 |

`test_dynamic.py` 在最新核心上失败的 2 项，是 V3.1.3「竞争深度一律拒绝」的预期行为变化，不是合并拷错文件：

1. `test_ambiguous_equal_angle_competing_ranges_hides`：期望状态 `ASSOCIATION_AMBIGUOUS`，实际为更明确的 `ASSOCIATION_COMPETING_DEPTHS`（同样不给出距离）。
2. `test_prior_keeps_person_vs_background`：V3.1 允许用历史先验在人/背景之间保住人物候选；V3.1.3 在近远点簇同时成立时改为直接拒绝，候选为 `None`。

未运行：各包 `install.py`、`test_install_offline.py`（会导入并执行安装逻辑），以及需要实机 8081/8092/8093 的影子测试、临时标定脚本。

本地复现：

```bash
python3 -m unittest discover -s fusion_8100/history/v3 -p 'test_*.py' -v
PYTHONPATH=fusion_8100 python3 fusion_8100/tests/test_v313.py -v
PYTHONPATH=fusion_8100 python3 fusion_8100/tests/test_dynamic.py -v   # 对 V3.1.3 会有上述 2 项失败
PYTHONPATH=fusion_8100/history/v3.1 python3 fusion_8100/tests/test_dynamic.py -v
python3 bridge_8101/test_stage1.py -v
python3 toggle_8090/test_offline.py
```

## 已知缺失文件

下列源码不在此次 ChatGPT 导出的安装包中，因此本仓库没有：

- `/home/orangepi/dual_overlay_switch.py`：8090 HUD 主体（跟踪滑块、云台回中、停止按钮）。`toggle_8090` 只是对它的注入补丁，安装器也依赖这份现场文件。
- 8081 YOLO 视觉服务源码。
- 8092 COIN-D6 雷达服务源码。
- 8093 云台 V2（`PREDICTIVE_FAST_V2` + 双轴温度策略）服务源码。

另外，8101 安装时在设备上生成的 `credentials.json` / PIN、以及 8100 现场 `config.json` 标定值都不在包内，也不应提交。

## 来源

- 来源：ChatGPT 项目「AI视觉」
- 相关对话：最新一则为「补充保存自动回中需求」
- 整理说明：按包版本叠加为当前最新树；原始 zip 未解包改动，完整放在 `packages/`
