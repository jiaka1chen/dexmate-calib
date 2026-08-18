# 外部相机手眼标定（Azure Kinect ↔ Vega base）

最后更新：2026-08-17（America/Los_Angeles）

本文说明如何用 `dexcalib extrinsics` 标定固定在环境中的 Azure Kinect 相对 Dexmate Vega
机器人 `base` 的外参 `T_base_cam`，并记录当前锁定的硬件与设计决定。

## 场景与约定

- 类型：**eye-to-hand**。相机固定不动，ChArUco 板刚性固定在机器人 link 上，机器人运动。
- 相机：Microsoft Azure Kinect，serial `000299113912`，通过 USB 3 连接运行 `dexcalib`
  的 Linux 主机。Color `1536P`（2048×1536），depth `NFOV_UNBINNED`（640×576），15 fps。
- 内参：直接使用 SDK 出厂标定（color `K` + 8 参数 rational 畸变，depth `K`，
  `T_color_depth`）。捕获时整体导出为 `camera_calibration.json`，离线求解不依赖 SDK。
- 机器人：`vega_1p`（`dexmate-urdf`），`base_frame = base`（URDF 根），板贴在 **`L_ee`**
  （左臂法兰）。`base → L_ee` 链上的关节：`torso_j1..3`、`L_arm_j1..7`。
- 板：`dexmate-10x7`（同内参标定，27 mm 方格，`DICT_5X5_50`）。
- 运动方式：**只允许手动**。用 dexcontrol 自带的
  `examples/advanced_examples/keyboard_joint_control.py`（或任意遥操作）移动机器人；
  `dexcalib` 只读关节角，从不下发运动指令。

坐标约定：`T_a_b` 是 4×4 齐次矩阵，把 `b` 系下的点变换到 `a` 系，即 `b` 在 `a` 中的位姿。
`T_base_cam` 中的 `cam` 是 Kinect **color** 相机光心系（OpenCV：x 右、y 下、z 前）。
`T_base_depth = T_base_cam @ T_color_depth` 一并写入结果。

## 数学模型

每个样本 `i` 提供两件事：

- `T_base_link_i`：由记录的关节角经 URDF 正运动学得到；
- 板角点的 2D 观测，配合已知的板几何可得 `T_cam_board_i`（PnP）。

未知量是 `X = T_base_cam`（目标）和 `Y = T_link_board`（板在 link 上的安装位姿，贴板时
无法精确知道，因此一并求解）。对每个样本有

```text
T_base_link_i · Y = X · T_cam_board_i        （AX = YB 形式）
```

求解流程（`src/dexmate_calib/extrinsics/handeye.py`）：

1. 每张图 `solvePnP` + LM refine 得到 `T_cam_board_i`（带完整畸变）。
2. Kronecker 线性闭式解（Li 2010）给出 `X`、`Y` 初值。这里没有使用 OpenCV 的
   `calibrateRobotWorldHandEye`，因为 opencv-contrib-python 5.0 的 wheel 不再暴露它。
3. 在 SE(3) 上做 Levenberg–Marquardt，**直接最小化所有角点的重投影误差**（12 个参数），
   Huber 权重（默认 2 px）；因此结果不继承单帧 PnP 的位姿噪声。
4. 逐视图 RMS 高于阈值（默认 `max(3 px, median + 3·MAD)`）的样本被剔除后重新求解，
   每轮最多剔除 20%。
5. Leave-one-view-out：每次留出一个样本重解，报告 `T_base_cam` 的旋转/平移离散度和
   留出样本的重投影误差。
6. 额外报告运动多样性：link 位姿的最大两两旋转角、旋转轴秩（应为 3）、平移跨度。

`dexcalib extrinsics selftest` 用合成场景验证整条链路：0.4 px 像素噪声 + 0.03°/0.5 mm
FK 噪声、25 视图时，`T_base_cam` 误差约 0.05° / 1 mm；注入的离群视图会被识别并剔除。

## 操作流程

### 0. 环境

```bash
cd ~/Projects/dexmate-calib
uv sync --extra dev --extra robot --extra kinect
source .venv/bin/activate
```

`kinect` extra 需要系统已安装 Azure Kinect SDK（`libk4a1.4`、`libk4a1.4-dev`、`k4a-tools`）
和 `/etc/udev/rules.d/99-k4a.rules`。Ubuntu 24.04 上使用微软 18.04 的 deb 加 jammy 的
`libsoundio1` 即可（安装脚本见 `~/.cache/dexmate-calib/k4a/install_k4a.sh`）。

`robot` extra 需要 `dexcontrol`、`dexmate-urdf`、`pin`（pinocchio）。dexcontrol 连接机器人
需要 `ROBOT_NAME` 和 `~/.dexmate/comm/zenoh/` 配置，与 V2AP-demo 的 `setup.sh` 相同。

### 1. 检查 Kinect

```bash
lsusb | grep 045e            # 必须出现 045e:097a Generic Superspeed Hub
dexcalib kinect info         # 出厂内参 / depth→color 外参 / serial
dexcalib kinect snapshot --output calibration_data/kinect_snapshot.png
```

如果 `lsusb -t` 里 Kinect 只有 480M（USB 2.0），换 USB 3 线或端口；depth 在 USB 2 下无法工作。

### 2. 固定板与相机

- 板刚性固定在左臂法兰（`L_ee`）附近，整个 session 内不得松动。板不需要精确对准 link 坐标系。
- Kinect 固定在三脚架/支架上，整个 session 内不得移动；标定完成后相机与机器人 base 的相对
  位置一旦改变（包括移动底盘）都必须重标。
- 采集期间**底盘和躯干不要动**（躯干关节在 FK 链上，允许动但建议固定；底盘运动会直接使
  `base` 漂移）。

### 3. 采集

终端 A（遥操作，来自 dexcontrol）：

```bash
python ~/Projects/dexcontrol/examples/advanced_examples/keyboard_joint_control.py
```

终端 B（本仓库）：

```bash
dexcalib extrinsics capture --samples 30
```

预览窗口按 **空格** 保存一个样本，`q`/`Esc` 结束。每次保存时程序会：

1. 检查板检测质量（角点数 ≥ 20、角点行列 ≥ 3×3）；
2. 连读 3 次关节角（间隔 0.15 s），最大变化 ≤ 0.002 rad 才认为机器人静止；
3. 静止确认后再抓一帧图像，用这帧和这组关节角落盘。

建议 25–40 个样本，覆盖：板在画面中心/四角、远近（0.5–1.2 m）、三个轴各 ±30° 以上的旋转。
只做平移或只绕一个轴转会让 `X` 不可观测，`solve` 会在 `motion diversity` 中提示。

选项：`--no-depth` 不采 depth；`--allow-moving` 放宽静止检查（不建议）；`--no-robot`
仅测试相机（生成的 session 不能求解）；`--solve` 采集结束后立即求解。

Session 目录：`calibration_data/handeye_kinect/<时间戳>_handeye_kinect_external_L_ee/`

```text
manifest.yaml              schema dexmate_calib.handeye_session.v1，robot/camera/board 锁定信息
board_profile.yaml         板配置快照（hash 写入 manifest）
camera_calibration.json    Kinect 出厂标定（color/depth K、畸变、T_color_depth）
color/0000.png             原始 BGR PNG（无损）
depth/0000.png             uint16 毫米深度（可选）
samples.jsonl              每行一个样本：图片路径、时间戳、关节角、静止检查、检测统计
```

采集端**不做 FK**，只存关节角；机器人型号或 link 写错时可以用 `solve --robot-model/--target-link`
重解，无需重采。

### 4. 求解

```bash
dexcalib extrinsics solve calibration_data/handeye_kinect/<session>
```

输出写入 `<session>/results/`：

```text
T_base_kinect_external.yaml   下游使用：K、畸变、T_base_cam、T_base_depth、RMS、样本数
handeye_result.json           完整结果：初值、refine 历史、LOO 统计、逐视图报告、剔除原因
T_base_cam.npy                4×4 numpy
per_view.csv                  每视图 RMS/最大误差、与 PnP 的位姿差、板距离、状态
reprojection_contact_sheet.jpg 绿=检测角点，紫=最终模型重投影，黄线=残差；红框=被剔除
```

判定标准（经验值，正式结果应满足）：

- `rms_reprojection_error_px` < 1.0（2048×1536 下）；
- LOO `T_base_cam` 平移离散度 max < 3 mm，旋转 < 0.1°；
- `motion_diversity.rotation_axis_rank == 3`，`max_pairwise_rotation_deg` > 40；
- 剔除样本不超过 20%。

## 代码结构

```text
src/dexmate_calib/geometry/transforms.py   SO(3)/SE(3) exp/log、逆、位姿误差（纯 numpy）
src/dexmate_calib/extrinsics/handeye.py    PnP、闭式初值、LM refine、剔除、LOO（numpy + cv2）
src/dexmate_calib/extrinsics/synthetic.py  合成场景（测试与 selftest）
src/dexmate_calib/extrinsics/capture.py    手动采集循环与 session 落盘
src/dexmate_calib/extrinsics/solve.py      读 session、FK、求解、写 results/
src/dexmate_calib/extrinsics/config.py     configs/handeye.yaml 加载
src/dexmate_calib/robot/kinematics.py      pinocchio + dexmate-urdf 正运动学
src/dexmate_calib/robot/dexmate.py         dexcontrol 只读关节采样（静止检查）
src/dexmate_calib/cameras/kinect.py        pyk4a 封装：BGR 帧、depth、出厂标定导出
configs/handeye.yaml                       上述所有锁定参数的默认值
```

方法上参考了同级 `RobotCamCalib/extr_calib.py`（AX=YB 联合估计、Huber、多样性检查），
但本仓库独立实现，不 import 它，也不引入 Viser、xArm、RealSense、AprilTag 依赖。

## 已知边界

- 只实现 eye-to-hand。若以后需要腕部相机（eye-in-hand），把 `T_base_link` 与
  `T_cam_board` 的角色互换即可复用同一求解器。
- Kinect 内参默认使用出厂值；如需自标可用同一块板走 `intrinsics` 流程做交叉验证，但目前
  `intrinsics solve` 锁定 ZED 1920×1200 rectified，需要扩展后才能用于 Kinect。
- 时间同步依赖"静止后再拍"，没有硬件触发；因此静止检查不要关闭。
