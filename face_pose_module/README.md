# 人脸位姿建模模块

本模块用于从 Intel RealSense D435i 的 RGB-D 视频流中估计人脸平面在相机坐标系下的位姿，并通过实时窗口检查结果，也可以通过 UDP/JSON 发送给 MATLAB。

## 功能

- 启动 D435i 彩色流和深度流。
- 将 `depth_frame` 对齐到 `color_frame`。
- 使用 MediaPipe Face Landmarker 检测单人脸关键点。
- 读取关键点对应深度并反投影到相机坐标系三维点。
- 使用多个人脸三维点拟合人脸平面。
- 输出 `center`、`normal`、`x_axis`、`valid`、`timestamp`。
- 对位姿结果进行离群过滤、平面残差检查、死区和滤波稳定化。
- 法向量使用独立角度死区和更强的低通滤波，减少可视化箭头的小幅颤抖。
- 在 OpenCV 窗口中显示彩色图、关键点、中心点、法向箭头、坐标、`valid` 和 FPS。
- 通过 UDP 向 `127.0.0.1:5005` 发送 JSON 数据。

## 目录结构

```text
face_pose_module/
  README.md
  requirements.txt
  config.yaml
  main.py
  camera_realsense.py
  face_landmarker.py
  depth_projector.py
  face_geometry.py
  filters.py
  udp_sender.py
  visualizer.py
  assets/
    README.md
```

## 环境准备

推荐使用已经验证通过的 conda 环境：

```powershell
cd E:\robotics\final_project\ws\face_pose_module
conda activate screen_arm
```

如果需要重新创建环境：

```powershell
cd E:\robotics\final_project\ws\face_pose_module
conda env create -f environment.yml
conda activate screen_arm
```

也可以手动创建：

```powershell
cd E:\robotics\final_project\ws\face_pose_module
conda create -n screen_arm python=3.11 pip -y
conda activate screen_arm
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`pyrealsense2` 和 `mediapipe` 对 Python 版本、操作系统和硬件平台比较敏感。本项目已在 `screen_arm` conda 环境的 Python `3.11.15` 下验证通过。不要使用当前系统 Python `3.13` 运行 RealSense 链路，因为 `pyrealsense2` 在该解释器下没有可用 wheel。

## MediaPipe 模型文件

需要手动下载 `face_landmarker.task`，放到：

```text
face_pose_module/assets/face_landmarker.task
```

可使用下面命令下载：

```powershell
Invoke-WebRequest `
  -Uri "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task" `
  -OutFile ".\assets\face_landmarker.task"
```

## 默认相机参数

当前按“高一点”的初始方案配置：

```text
depth: 1280x720 @ 30fps, format z16
color: 1280x720 @ 30fps, format bgr8
align depth to color: true
infrared: false
IMU: true
accel: 63Hz
gyro: 200Hz
```

如果运行时帧率不稳定，可以先把 `config.yaml` 中的 `camera.color_width`、`camera.color_height` 改成 `640x480`，或把 `camera.depth_width`、`camera.depth_height` 改成 `848x480`。

## 运行

连接 D435i 后运行：

```powershell
cd E:\robotics\final_project\ws\face_pose_module
conda activate screen_arm
python main.py --config config.yaml
```

窗口打开后按 `q` 退出。

如果只检查可视化、不发送 UDP：

```powershell
python main.py --config config.yaml --no-udp
```

如果只发送 UDP、不显示窗口：

```powershell
python main.py --config config.yaml --no-window
```

自动处理固定帧数后退出，适合冒烟测试：

```powershell
python main.py --config config.yaml --no-window --no-udp --max-frames 60
```

## UDP 输出

默认发送到：

```text
IP: 127.0.0.1
Port: 5005
Protocol: UDP
Format: JSON
```

数据格式：

```json
{
  "t": 1718000000.123,
  "valid": true,
  "center": [0.02, -0.04, 0.75],
  "normal": [0.10, -0.05, -0.99],
  "x_axis": [0.99, 0.02, 0.01],
  "imu": {
    "accel": [0.40, -9.48, 0.92],
    "gyro": [-0.006, -0.001, -0.001]
  }
}
```

坐标系使用 RealSense 相机坐标系，单位为 m。默认 `normal` 指向相机方向，也就是用户大致看向屏幕时通常接近 `[0, 0, -1]`。如果 MATLAB 侧需要相反方向，可以在 `config.yaml` 中修改 `geometry.normal_towards_camera`。`imu` 为可选字段，当相机返回运动数据时包含最新 `accel` 和 `gyro`。

## infrared 和 IMU

`infrared` 是 D435i 左右红外灰度图像流，主要用于查看深度计算输入、弱纹理场景调试、相机标定或后续额外视觉算法。当前人脸建模主链路使用 `color + aligned depth`，不直接使用 infrared。开启 infrared 会增加 USB 带宽占用，因此默认关闭。

`IMU` 输出 D435i 的加速度计和陀螺仪数据。当前第一版人脸平面拟合不把 IMU 纳入位姿计算，但采集层已经默认开启，并在 `RealSenseFrame.imu` 中保留 `accel` 和 `gyro` 最新样本，便于后续做相机姿态补偿或和 MATLAB 运动学模块融合。

## 测试脚本

不依赖相机的离线测试：

```powershell
cd E:\robotics\final_project\ws\face_pose_module
python tests\run_offline_tests.py
```

D435i 彩色、深度和 IMU 冒烟测试：

```powershell
python tests\realsense_smoke_test.py --frames 60
```

完整链路无窗口冒烟测试：

```powershell
python tests\full_pipeline_smoke_test.py --frames 90
```

MediaPipe 模型加载测试：

```powershell
python tests\landmarker_model_test.py
```

UDP 接收测试可以在另一个终端运行：

```powershell
python tests\udp_receiver_test.py --seconds 30
```

## 关键点方案

当前使用 Face Landmarker 输出的单人脸 478 个关键点中的稳定区域：

- 左右眼区域：用于计算眼中心和 `x_axis`。
- 鼻梁、鼻尖附近点：用于平面中心区域约束。
- 嘴角、下巴、脸颊、额头附近点：用于拟合整体人脸平面。

人脸中心点优先使用左右眼三维中心点的中点。平面法向量由多个人脸三维点拟合得到，不使用单个鼻尖点代表整个人脸位姿。

## 已知限制

- 首次运行前必须放入 `assets/face_landmarker.task`。
- 当前没有在无 RealSense 硬件环境下进行实机验证。
- D435i 高分辨率模式对 USB 带宽和主机性能有要求，实际帧率可能低于 30fps。
- `normal` 的方向约定需要和 MATLAB 侧末端坐标系定义保持一致。
