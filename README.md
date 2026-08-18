# dexmate-calib

Dexmate 相机标定工具，目前包含两条流程：

1. **头部 ZED X Mini 左目内参**（`dexcalib intrinsics`），正式模式固定为
   **HD1200 / 1920×1200 / `sl::VIEW::LEFT` rectified image**；
2. **外部 Azure Kinect ↔ 机器人 base 手眼标定**（`dexcalib extrinsics`，eye-to-hand），
   板贴在左臂 `L_ee`，机器人只允许手动逐关节遥操作移动，详见 [docs/handeye.md](docs/handeye.md)。

推荐使用 `intrinsics quickstart`：本机通过公钥 SSH 保持一个 Nano 会话，在该会话中
启动固定参数 streamer，验证 stream 后进入采集，并在采集结束或异常时关闭这次会话和
streamer；正常完成采集后默认离线求解新 session。它不会在 Nano 安装 service，不写
sudoers，不使用 `--clean`。

## Quick start

完整的首次网络、公钥和安全说明见 [docs/quickstart.md](docs/quickstart.md)。满足其中前提后：

```bash
cd ~/Documents/Projects/dexmate/dexmate-calib
uv sync --extra dev
source .venv/bin/activate

dexcalib intrinsics quickstart \
  --board dexmate-10x7 \
  --samples 40
```

采集完成后默认停止本次启动的 streamer，再求解刚创建的 session。仅做少量 smoke test
或只想保留原始数据时显式添加 `--no-solve`。

默认情况下 SSH 登录本身免密，但 Nano 上的 `sudo` 可能在当前终端提示一次密码。密码不
经过 Python、不保存；同一 SSH 会话持续到采集结束，所以停止 streamer 不需要再次认证。

当前 factory 参数、5 个合格实测 session、`dexmate-calib` 自标定建议参数以及完整结果检查
方法，统一记录在 [docs/head_left_intrinsics.md](docs/head_left_intrinsics.md)。

## 已锁定的硬件和标定板

- Camera Jetson: `192.168.50.22`, TCP stream port `30000`
- Head camera serial: `59595115`
- Protocol: ZS02，receiver 同时兼容旧 ZS01
- Native output: `1920×1200`, HD1200
- View: rectified left; 输出畸变固定为零
- Board: 10×7 squares, 27.000 mm square, 20.250 mm marker, `DICT_5X5_50`
- Board physical size: 270×189 mm; 100 mm check 已实测

配置都在 `configs/`，不会在求解代码中静默猜测或交换板的 X/Y。

## 当前参数基准

当前 streamer 输出是 ZED SDK rectified `LEFT`。默认生产基准为当前 serial 在 HD1200 下的
SDK factory rectified 参数：

```text
fx = 746.9691   fy = 746.9691
cx = 959.9904   cy = 585.4913
D  = [0, 0, 0, 0, 0]
```

筛选 5 个通过质量门的独立 `dexmate-calib` session 后，当前自标定建议值为：

```text
fx = 747.6196   fy = 748.7749
cx = 959.6954   cy = 584.7266
D  = [0, 0, 0, 0, 0]
```

自标定建议值来自 5 个 session、193 个最终视图的一次 pooled joint solve，不是分别求解后
再平均；联合命令、验证结果、每次结果及适用边界见
[头部左相机内参记录](docs/head_left_intrinsics.md)。两套参数都只适用于 serial `59595115`、
HD1200、1920×1200、rectified `LEFT`，不能与 raw/HD1080/crop 后的图像混用。

## 方法来源与 Dexmate 约束

采集质量、标定板区域清晰度、姿态多样性、稳健离群值剔除和交叉验证的组织方式参考了
同级 `RobotCamCalib/intr_calib_charuco.py` 的成熟流程，并在本仓库中独立实现。当前工具
不会 import 或运行 RobotCamCalib，也不会引入其中的 USB camera、fisheye、AprilTag Grid
或 raw-image distortion 模型。Dexmate 输入是 ZED SDK 的 rectified `LEFT`，所以求解器始终
固定全部 distortion coefficients 为零，并拒绝非 HD1200/1920×1200 session。

## 安装

一站式脚本（完整说明、依赖表和常见问题见 [docs/install.md](docs/install.md)）：

```bash
cd ~/Projects/dexmate-calib
sudo scripts/install_kinect_sdk.sh   # 仅 Kinect 接本机时：Azure Kinect SDK 1.4.2 + udev 规则
scripts/setup_env.sh                 # uv + 托管 Python 3.12 + .venv（--minimal / --no-kinect 可选）
source .venv/bin/activate
```

仓库内只有一个 `.venv`，内参与手眼标定共用；手工等价命令是
`uv sync --managed-python --extra dev --extra robot --extra kinect`。

确认 CLI 和标定板：

```bash
dexcalib board list
dexcalib board show dexmate-10x7
dexcalib board validate configs/boards/dexmate_charuco_10x7_27mm.yaml
```

用已经拍摄的实物板照片验证配置：

```bash
dexcalib board verify \
  --board dexmate-10x7 \
  --image ~/Downloads/IMG_0276.jpeg
```

期望检测到 35 markers、54 ChArUco corners。

## 网络与取流路径

SSH 只负责在 Nano 上启动 streamer；图像数据是 Mac 直接连接 Nano：

```text
Mac -- SSH（可经 .20 ProxyCommand） --> 192.168.50.22
Mac -- direct TCP ------------------> 192.168.50.22:30000
```

`.20` 若用于 `ssh -W`，只转发 SSH 的 TCP 字节，不转发相机数据。

连接机器人后先做只读检查：

```bash
dexcalib doctor network
```

离线时该命令失败是正常的，不影响离线测试和求解。

### 一站式启动的行为边界

```bash
dexcalib intrinsics quickstart --board dexmate-10x7 --samples 40
```

它按顺序执行：

1. 用 `BatchMode=yes` 测试直连 `.22`，失败后才 fallback 到经 `.20` 的 ProxyCommand。
2. 若 `.22:30000` 尚未监听，在保持的 SSH PTY 中启动固定的 HD1200 left-only 命令。
3. 强制验证 ZS protocol、serial `59595115`、1920×1200、timestamp 和 JPEG decode。
4. 进入 ChArUco 采集。
5. 仅当 streamer 是本次 quickstart 启动的，采集结束或异常后才关闭它。
6. 默认在本机求解刚创建的 session；`--no-solve` 可停在采集结束。

若端口开始时已经有 streamer，quickstart 会验证并复用，但不会在结束时停止别人的进程。
如果 Nano 已配置当前用户直接访问相机，可以加 `--no-sudo`。默认不使用 `--clean`，不会
自动杀其他 ZED 程序。

## 在 Nano 上启动内参采集 streamer

这是 quickstart 失败时的手动 fallback。先确认没有其他任务需要 ZED，然后登录 Nano：

```bash
ssh dexmate-nano
cd ~/zed_stream
sudo ./build/zed_streamer \
  --jpeg-quality 100 \
  --max-fps 30 \
  --resolution HD1200 \
  --no-right \
  --no-depth \
  --no-pc \
  --no-imu
```

这里关闭 depth，因为内参只需要左目 JPEG。不要用 HD1080；采集端会拒绝任何不是
1920×1200 的 stream。如果确实需要 `--clean`，必须先确认其他 ZED 任务可以被终止，再由
操作者显式添加；quickstart 永远不会自动添加它。

另一个本机终端验证实际数据：

```bash
dexcalib doctor stream --frames 30
```

它会检查协议、serial、分辨率、timestamp、接收 FPS 和 JPEG decode。ZS01 没有
serial 字段；仅在测试旧 server 时可用 `--expected-serial 0` 关闭 serial gate。

## 采集

```bash
dexcalib intrinsics capture \
  --board dexmate-10x7 \
  --samples 40 \
  --output calibration_data/head_left
```

独立的 `intrinsics capture` 只采集，不自动求解；一站式 `intrinsics quickstart` 默认在
采集后求解。

默认自动采集，按 `q` 或 `Esc` 提前结束。使用 `--manual` 后按空格保存符合质量门限的
画面。每个 session 锁定：

- camera serial、ZS protocol、resolution、channel mask
- `HD1200`, `LEFT`, `rectified`
- 完整 board profile snapshot 及 SHA-256
- 原始 server JPEG（不二次编码）和 source/receive timestamps

建议采集 40 张左右，覆盖画面中心、四角和边缘，并改变距离、倾角和旋转。不要只把板
平行地放在画面中央。

默认质量门限可通过命令行调整：

```bash
dexcalib intrinsics capture --help
```

实时窗口会绘制 ArUco marker 和 ChArUco corner，并显示 corner/grid 数量、画面覆盖、
board bbox、清晰度、pixels-per-square、冷却和重复视角状态。质量筛选除角点数量外还会
检查角点在板内的行列分布，避免只检测到集中在一小块区域的角点。`--manual` 依赖预览
窗口接收空格键，因此不能与 `--no-preview` 同时使用。

自动模式默认以 10 Hz 处理 ChArUco（`--detect-fps 10`），但仍持续读取所有 TCP frame，
不会因为降频而让 socket 数据积压。0.8 秒保存 cooldown 内使用轻量质量路径；只有可能
成为候选的帧才执行 homography 展平和 Laplacian/Tenengrad 详细分析。手动模式保持逐帧
检测。调试时可用 `--detect-fps 0` 恢复逐 stream frame 检测。

## 求解

`intrinsics quickstart` 默认已自动求解。使用 `--no-solve`、独立 `intrinsics capture`
或需要用其他求解参数重新计算时，可在断开机器人后运行：

```bash
dexcalib intrinsics solve \
  calibration_data/head_left/20260814_230000_head_left_HD1200
```

求解器会重新检测所有原始 JPEG，固定所有 distortion coefficients 为零，只估计
`fx, fy, cx, cy` 和每个 board pose，并执行：

- 标定板 homography 展平后的尺度感知清晰度检查
- 确定性的姿态多样性选择（样本超过 `--max-views` 时）
- 每视图 reprojection error
- robust outlier rejection
- deterministic K-fold held-out validation
- even/odd split 参数稳定性检查

结果写入 session 的 `results/`：

```text
intrinsics_head_left_HD1200_1920x1200.yaml
intrinsics_head_left_HD1200_1920x1200.json
K.npy
reprojection_errors.csv
sample_selection.csv
cross_validation.json
quality_summary.json
capture_contact_sheet.jpg
reprojection_contact_sheet.jpg
```

绿色点是检测角点，紫色点是最终模型的重投影位置，黄色短线显示两者之间的残差。
`sample_selection.csv` 会为每张图片记录最终保留或拒绝原因，包括 motion blur、pose
redundancy 和 reprojection outlier。

1920×1200 是 source of truth。任何下游 resize/crop 必须显式变换 `K`；不能直接把
HD1200 内参用于 HD1080，也不能把 1920×1200 非等比例拉伸到 640×360 后仍沿用原 K。

## 外部相机手眼标定（Azure Kinect）

```bash
dexcalib kinect info                       # 出厂内参、depth→color 外参、serial
dexcalib extrinsics selftest               # 合成场景自检
dexcalib extrinsics capture --teleop --samples 30   # 预览窗口里 w/s 逐关节小步动手臂，空格保存
dexcalib extrinsics solve calibration_data/handeye_kinect/<session>
```

求解 `T_base_link_i · Y = X · T_cam_board_i` 中的 `X = T_base_cam` 与 `Y = T_link_board`：
PnP → Kronecker 闭式初值 → SE(3) 上以重投影误差做 LM refine（Huber）→ 逐视图剔除 →
leave-one-out。结果、判定标准、session 与 `results/` 格式见 [docs/handeye.md](docs/handeye.md)。
所有锁定参数在 `configs/handeye.yaml`。

## 测试

```bash
pytest
```

单元测试不需要连接机器人或相机，覆盖 ZS01/ZS02 header、跳过 depth payload、HD1200/serial
gate、board schema、质量筛选、交叉验证、synthetic rectified pinhole calibration、完整
synthetic session 输出，以及手眼标定的 SE(3) 工具、闭式初值约定、含噪/离群合成场景、URDF
FK 链和 capture→solve 的合成 session 端到端。

## 数据与安全

- `calibration_data/` 默认被 Git 忽略。
- 仓库中不得保存 SSH 密码、PIN、私钥或证书。
- 工具不会修改 Nano 上已有且未提交的 `zed_stream` 源码。
- quickstart 不会在 Nano 安装服务、包或持久配置，也不会写入 sudoers。
- SSH 私钥始终只留在 Mac；只把 `.pub` 公钥追加到远端 `authorized_keys`。
- 不会在断线重连时静默切换 camera、resolution 或 board profile。
- 手眼标定不会自动规划或执行位姿序列：机器人只能由操作者逐关节小步遥操作移动（`--teleop`
  或 dexcontrol 自带键盘示例），每次按键一步、松手即停。
