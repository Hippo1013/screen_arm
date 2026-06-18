# 人脸姿态建模模块 v2：D435i RGB + 3DDFA_V2

本目录是独立实验分支模块，基于官方 `cleardusk/3DDFA_V2` 实现第二版人脸姿态估计。它不修改 `face_pose_module`、`screen_arm` 或根目录 `test`。

## 目标链路

```text
D435i RGB 视频流 + IMU
  ↓
FaceBoxes 人脸框初始化 / 重捕获
  ↓
3DDFA_V2 三维人脸对齐与头部姿态估计
  ↓
输出 normal / yaw / pitch / roll / imu
  ↓
OpenCV 可视化测试、UDP、JSON 文件
```

v2 只启动彩色流和 IMU，不启动深度流。由于没有深度，`center` 只是为了兼容 v1 JSON 字段而保留的固定距离近似值；当前主测试脚本应主要使用 `normal` 和 `imu`。

## 目录结构

```text
face_pose_module_v2/
  README.md
  config.yaml
  environment.yml
  requirements.txt
  main.py
  camera_realsense_rgb.py
  face_detector.py
  three_ddfa_pose.py
  pose_types.py
  filters.py
  udp_sender.py
  pose_file_writer.py
  visualizer.py
  scripts/
    bootstrap_3ddfa_v2.py
  tests/
    import_smoke_test.py
    offline_image_pose_test.py
    live_visual_snapshot_test.py
    realsense_rgb_imu_smoke_test.py
    full_pipeline_smoke_test.py
    udp_receiver_test.py
  third_party/
    3DDFA_V2/
    3DDFA_V2_COMMIT.txt
```

## 环境准备

v1 的 `screen_arm` 环境没有 `torch/torchvision`。为了避免影响 v1，推荐为 v2 单独创建环境：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
conda env create -f environment.yml
conda activate screen_arm_v2
```

如果环境已存在，也可以直接安装依赖：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
conda activate screen_arm_v2
python -m pip install -r requirements.txt
```

## 3DDFA_V2 依赖

本目录已放入官方 3DDFA_V2 代码，位置为：

```text
face_pose_module_v2/third_party/3DDFA_V2
```

如果需要重新拉取：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
python scripts\bootstrap_3ddfa_v2.py
```

当前默认使用官方 PyTorch 权重：

```text
third_party/3DDFA_V2/configs/mb1_120x120.yml
third_party/3DDFA_V2/weights/mb1_120x120.pth
third_party/3DDFA_V2/configs/bfm_noneck_v3.pkl
```

说明：官方 `FaceBoxes` 在 Windows 上通常需要编译 `cpu_nms`。本模块在
`third_party/3DDFA_V2/FaceBoxes/utils/nms_wrapper.py` 中加入了最小 fallback，
当编译版 NMS 不存在时使用官方仓库自带的 `py_cpu_nms.py`。

## 运行可视化

连接 D435i 后运行：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
conda activate screen_arm_v2
python main.py --config config.yaml --no-udp
```

窗口打开后按 `q` 退出。

如果要同时写出最新姿态 JSON，供 MATLAB 或其他程序轮询：

```powershell
python main.py --config config.yaml --pose-file "$env:TEMP\screen_arm_face_pose_live_v2.json"
```

如果只跑固定帧数的无窗口冒烟测试：

```powershell
python main.py --config config.yaml --no-window --no-udp --max-frames 90
```

## UDP / JSON 输出

默认 UDP：

```text
IP: 127.0.0.1
Port: 5005
Format: JSON
```

输出字段兼容 v1 主字段。这里约定 `normal` 就是人脸法向量，也是人脸所面向的方向；OpenCV 可视化中的红色箭头画的就是这个 `normal`。

```json
{
  "t": 1718000000.123,
  "valid": true,
  "center": [0.01, -0.03, 0.65],
  "normal": [0.10, -0.02, -0.99],
  "x_axis": [0.99, 0.01, 0.10],
  "imu": {
    "accel": [0.40, -9.48, 0.92],
    "gyro": [-0.006, -0.001, -0.001]
  },
  "angles_deg": {
    "yaw": 8.2,
    "pitch": -2.4,
    "roll": 1.1
  },
  "status": "valid_filtered"
}
```

`normal` 使用 RealSense 彩色相机坐标约定：`+X` 图像右，`+Y` 图像下，`+Z` 从相机指向用户。默认 `normal_towards_camera: true`，正脸看向相机时通常接近 `[0, 0, -1]`。
`x_axis` 只是兼容和调试用的脸部横向辅助轴，不代表视线方向。

## 测试脚本

导入和模型加载测试：

```powershell
python tests\import_smoke_test.py --load-model
```

离线图片姿态测试，会生成 `outputs/offline_image_pose.jpg`：

```powershell
python tests\offline_image_pose_test.py
```

D435i RGB + IMU 冒烟测试：

```powershell
python tests\realsense_rgb_imu_smoke_test.py --frames 60
```

保存一帧实时相机可视化结果到 `outputs/live_visual_snapshot.jpg`：

```powershell
python tests\live_visual_snapshot_test.py --frames 120
```

如果希望无人脸时测试失败，可以加：

```powershell
python tests\live_visual_snapshot_test.py --frames 120 --require-valid
```

完整链路无窗口测试：

```powershell
python tests\full_pipeline_smoke_test.py --frames 90
```

UDP 接收测试：

```powershell
python tests\udp_receiver_test.py --seconds 30
```

## 关键配置

`config.yaml` 中最重要的参数：

- `camera.enable_depth: false`：v2 不启动深度流。
- `camera.enable_imu: true`：保留 D435i IMU，用于 MATLAB 侧相机姿态补偿。
- `three_ddfa.detect_interval`：每隔多少帧重新用检测器校正一次人脸框。
- `three_ddfa.assumed_face_distance_m`：仅用于 `center` 兼容字段和可视化箭头投影。
- `three_ddfa.model_normal_axis`：3DDFA 模型坐标到相机法向的默认轴映射；如果实测法向反了，优先改这个或 `normal_towards_camera`。
- `three_ddfa.flip_output_x`：修正 3DDFA 姿态到 RealSense 图像坐标的水平符号。默认 `true`，使人脸朝画面左侧时红色法向箭头也朝画面左侧。
- `three_ddfa.flip_output_y`：预留给上下方向符号修正，默认 `false`。
- `visualization.draw_axes`：绘制红色人脸法向箭头，也就是人脸面向方向。
- `visualization.draw_auxiliary_x_axis`：绘制蓝色辅助横向轴，默认开启；它用于观察脸部横向姿态，不代表视线方向。

## 已知限制

- v2 的真实三维位置不可靠，主输出应看 `normal`、`angles_deg` 和 `imu`。
- 默认使用官方 FaceBoxes 做初始化和丢失后重捕获；大角度连续跟踪主要依赖 3DDFA 上一帧 landmarks。
- 首次加载 PyTorch 模型会比较慢，后续帧才进入正常实时速度。
- 如果 `normal` 在 MATLAB 中方向相反，先在 `config.yaml` 调整 `three_ddfa.normal_towards_camera` 或 `three_ddfa.model_normal_axis`，不要改 MATLAB 运动学模块。
