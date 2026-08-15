# dexmate-calib

Dexmate 相机标定工具。当前实现聚焦头部 ZED X Mini 左目内参，正式模式固定为
**HD1200 / 1920×1200 / `sl::VIEW::LEFT` rectified image**。

推荐使用 `intrinsics quickstart`：本机通过公钥 SSH 保持一个 Nano 会话，在该会话中
启动固定参数 streamer，验证 stream 后进入采集，并在采集结束或异常时关闭这次会话和
streamer。它不会在 Nano 安装 service，不写 sudoers，不使用 `--clean`。

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

默认情况下 SSH 登录本身免密，但 Nano 上的 `sudo` 可能在当前终端提示一次密码。密码不
经过 Python、不保存；同一 SSH 会话持续到采集结束，所以停止 streamer 不需要再次认证。

## 已锁定的硬件和标定板

- Camera Jetson: `192.168.50.22`, TCP stream port `30000`
- Head camera serial: `59595115`
- Protocol: ZS02，receiver 同时兼容旧 ZS01
- Native output: `1920×1200`, HD1200
- View: rectified left; 输出畸变固定为零
- Board: 10×7 squares, 27.000 mm square, 20.250 mm marker, `DICT_5X5_50`
- Board physical size: 270×189 mm; 100 mm check 已实测

配置都在 `configs/`，不会在求解代码中静默猜测或交换板的 X/Y。

## 安装

建议使用独立环境：

```bash
cd ~/Documents/Projects/dexmate/dexmate-calib
uv sync --extra dev
source .venv/bin/activate
```

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

## 求解

采集可在断开机器人后离线求解：

```bash
dexcalib intrinsics solve \
  calibration_data/head_left/20260814_230000_head_left_HD1200
```

求解器会重新检测所有原始 JPEG，固定所有 distortion coefficients 为零，只估计
`fx, fy, cx, cy` 和每个 board pose，并执行：

- 每视图 reprojection error
- robust outlier rejection
- deterministic held-out validation
- even/odd split 参数稳定性检查

结果写入 session 的 `results/`：

```text
intrinsics_head_left_HD1200_1920x1200.yaml
intrinsics_head_left_HD1200_1920x1200.json
K.npy
reprojection_errors.csv
```

1920×1200 是 source of truth。任何下游 resize/crop 必须显式变换 `K`；不能直接把
HD1200 内参用于 HD1080，也不能把 1920×1200 非等比例拉伸到 640×360 后仍沿用原 K。

## 测试

```bash
pytest
```

单元测试不需要连接机器人，覆盖 ZS01/ZS02 header、跳过 depth payload、HD1200/serial
gate、board schema 和 synthetic rectified pinhole calibration。

## 数据与安全

- `calibration_data/` 默认被 Git 忽略。
- 仓库中不得保存 SSH 密码、PIN、私钥或证书。
- 工具不会修改 Nano 上已有且未提交的 `zed_stream` 源码。
- quickstart 不会在 Nano 安装服务、包或持久配置，也不会写入 sudoers。
- SSH 私钥始终只留在 Mac；只把 `.pub` 公钥追加到远端 `authorized_keys`。
- 不会在断线重连时静默切换 camera、resolution 或 board profile。
