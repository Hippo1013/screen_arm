# 开发记录

## 当前项目目标

构建一个基于 Intel RealSense D435i、MediaPipe Face Landmarker、OpenCV、NumPy、pyrealsense2 的 Python 端“人脸位姿建模模块”。该模块从 D435i RGB-D 视频流中估计人脸平面在相机坐标系下的位姿，并为后续 MATLAB 机器人运动学模块提供 UDP/JSON 输出。

当前阶段只进行工作区探查、技术方案整理和参数确认，不编写核心采集与建模代码。

## 工作区检查

- 当前工作区：`E:\robotics\final_project\ws`
- 已确认存在：`screen_arm/`
- 处理原则：`screen_arm/` 为已有六自由度机械臂模型目录，当前阶段不移动、不重命名、不修改。
- 当前尚未创建：`face_pose_module/`

## 待确认的 D435i 相机启动参数

建议候选方案：

- `depth`: `848x480 @ 30fps`, format `z16`
- `color`: 优先 `640x480 @ 30fps`, format `bgr8`
- `color` 备选：`1280x720 @ 30fps`, format `bgr8`
- `align_depth_to_color`: `true`
- `infrared`: `false`
- `IMU`: `false`

需要用户确认：

- depth 分辨率
- depth 帧率
- color 分辨率
- color 帧率
- depth 格式
- color 格式
- 是否启用 infrared
- 是否启用 IMU
- 是否将 depth 对齐到 color

## 待确认的 MediaPipe Face Landmarker 方案

建议候选方案：

- 使用 `MediaPipe Face Landmarker`，不使用旧版 `Face Mesh`
- `num_faces = 1`
- 使用 `.task` 模型文件：`face_landmarker.task`
- 模型文件建议放置在后续目录：`face_pose_module/assets/face_landmarker.task`
- 启用 `output_facial_transformation_matrixes`
- `output_face_blendshapes` 默认关闭，除非后续需要表情信息
- 人脸中心点优先使用左右眼中心点的中点
- 人脸平面拟合使用多个人脸三维关键点，不只使用鼻尖单点
- 平面法向量由多个人脸三维关键点拟合得到
- `x_axis` 由左右眼方向确定，用于后续 MATLAB 构造完整旋转矩阵

需要用户确认：

- 使用 `Face Landmarker` 还是旧版 `Face Mesh`
- 使用哪个 `.task` 模型文件
- 是否只检测单人脸
- 是否开启 `facial transformation matrix`
- 是否开启 `blendshapes`
- 用哪些关键点建模人脸平面
- 如何定义人脸中心点
- 如何定义人脸法向量方向

## 建议的人脸建模算法流程

1. 从 D435i 获取 `color_frame` 和 `depth_frame`。
2. 将 `depth_frame` 对齐到 `color_frame`。
3. 使用 MediaPipe Face Landmarker 在彩色图像中检测人脸关键点。
4. 将归一化关键点转换为二维像素坐标 `(u, v)`。
5. 在对齐后的深度图中读取关键点深度 `Z`。
6. 使用 `rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth)` 反投影得到相机坐标系下三维点。
7. 过滤无效深度、越界点和离群三维点。
8. 使用最小二乘平面拟合或 RANSAC 拟合人脸平面。
9. 使用左右眼中心点的中点作为 `face_center_camera`。
10. 使用拟合平面法向量作为 `face_normal_camera`。
11. 使用左右眼方向构造 `x_axis`。
12. 对中心点、法向量和横向轴做稳定化处理。
13. 输出 `valid`、`timestamp`、`center`、`normal`、`x_axis`。
14. 在调试窗口中叠加显示彩色图、关键点、中心点、法向箭头、坐标、`valid` 和 FPS。
15. 通过 UDP 将 JSON 数据发送到 `127.0.0.1:5005`，供 MATLAB 后续接收。

## 稳定化方案

建议实现以下机制，并将阈值写入后续 `config.yaml`：

- 深度无效值过滤
- 关键点离群值过滤
- 最小可用三维点数量检查
- 平面拟合残差阈值
- 滑动平均滤波
- 指数平滑滤波
- 中心点死区阈值
- 法向量角度死区阈值
- 检测失败后的短时保持上一帧有效结果
- `valid` 状态输出

## 建议项目结构

确认方案后再创建：

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
    face_landmarker.task
```

## 已创建或修改的文件

- `AGENT.md`：记录当前项目目标、探查结果、待确认参数、建议技术方案、后续开发记录。

## 当前运行方式

当前阶段尚未创建运行程序。待 D435i 参数和 MediaPipe 方案确认后，再创建 `face_pose_module/` 并提供运行命令。

## 已测试的命令

- `Get-ChildItem -Force`
- `git status --short`

## 已知问题

- 当前工作区不是 Git 仓库，`git status --short` 返回 `fatal: not a git repository`。
- 当前机器环境尚未验证是否已安装 `pyrealsense2`、`mediapipe`、`opencv-python`、`numpy`、`scipy`、`pyyaml`。
- 当前未检测 RealSense 设备连接状态。
- `face_landmarker.task` 模型文件尚未放入项目。

## 后续待办

1. 等待用户确认 D435i 启动参数。
2. 等待用户确认 MediaPipe Face Landmarker 使用方式和关键点方案。
3. 创建 `face_pose_module/` 目录。
4. 编写模块化 Python 代码。
5. 编写 `README.md`、`requirements.txt`、`config.yaml`。
6. 实现实时可视化检查窗口。
7. 实现或预留 UDP JSON 输出。
8. 运行可用性检查并更新本文档。

## 2026-06-10 阶段更新：确认并实现第一版模块

用户确认：相机启动参数初步“都设高一点”，MediaPipe 参数采用建议方案。

已落地的相机参数：

- `depth`: `1280x720 @ 30fps`, format `z16`
- `color`: `1280x720 @ 30fps`, format `bgr8`
- `align_depth_to_color`: `true`
- `infrared`: `false`
- `IMU`: `false`

已落地的 MediaPipe 参数：

- 使用 `MediaPipe Face Landmarker`
- `num_faces = 1`
- 模型文件路径：`face_pose_module/assets/face_landmarker.task`
- `output_facial_transformation_matrixes = true`
- `output_face_blendshapes = false`
- 人脸中心：左右眼三维中心点的中点
- 人脸法向：多个人脸三维关键点拟合平面得到
- `x_axis`：左右眼方向，按图像横坐标排序后指向相机坐标系 `+X`

新增文件：

- `face_pose_module/README.md`：中文说明、安装、模型下载、运行方式、UDP 格式和注意事项。
- `face_pose_module/requirements.txt`：Python 依赖列表。
- `face_pose_module/config.yaml`：相机、MediaPipe、几何拟合、滤波、UDP 和可视化参数。
- `face_pose_module/main.py`：主程序入口，串联采集、检测、建模、滤波、UDP 和可视化。
- `face_pose_module/camera_realsense.py`：D435i 启动、帧读取、depth 对齐到 color。
- `face_pose_module/face_landmarker.py`：MediaPipe Face Landmarker 初始化和逐帧检测。
- `face_pose_module/depth_projector.py`：深度窗口中值过滤和 RealSense 反投影。
- `face_pose_module/face_geometry.py`：关键点三维化、人脸中心、平面拟合、法向量和 `x_axis` 计算。
- `face_pose_module/filters.py`：EMA、滑动平均、死区和短时丢失保持。
- `face_pose_module/udp_sender.py`：UDP/JSON 输出。
- `face_pose_module/visualizer.py`：OpenCV 实时调试窗口叠加显示。
- `face_pose_module/assets/README.md`：模型文件放置说明。

核心算法流程已实现：

1. D435i 采集 color/depth。
2. depth 对齐到 color。
3. Face Landmarker 检测单人脸关键点。
4. 关键点像素坐标读取对齐深度。
5. 使用 RealSense 内参反投影到相机坐标系三维点。
6. 过滤无效深度和离群点。
7. 使用 RANSAC 优先、SVD 兜底拟合人脸平面。
8. 用左右眼三维中心点中点作为人脸中心。
9. 法向量默认指向相机方向。
10. 使用滤波和死区稳定输出。
11. 可视化窗口显示调试信息。
12. UDP 输出 JSON 给 MATLAB。

后续待办更新：

1. 下载 `face_landmarker.task` 到 `face_pose_module/assets/`。
2. 安装依赖并连接 D435i 实机测试。
3. 根据实测帧率决定是否从 `1280x720` 降到 `848x480` 或 `640x480`。
4. 和 MATLAB 侧确认 `normal` 方向约定。

## 2026-06-10 阶段更新：IMU 与测试脚本

用户补充：

- 询问 `infrared` 参数作用。
- 明确 `IMU` 可以开启，后续建模可能会用到。
- 完成标准调整为：构建好人脸建模模块完整版以及测试脚本，用户会连接相机供实测。

处理结果：

- `camera.enable_imu` 已改为 `true`。
- 新增 `camera.imu_accel_fps = 63`。
- 新增 `camera.imu_gyro_fps = 200`。
- `infrared` 仍默认关闭，因为当前 MediaPipe 和深度反投影链路不消费红外图像，开启会增加 USB 带宽占用。
- `RealSenseFrame` 新增 `imu` 字段，用于保留 `accel` 和 `gyro` 样本。
- 新增测试脚本：
  - `face_pose_module/tests/run_offline_tests.py`
  - `face_pose_module/tests/realsense_smoke_test.py`
  - `face_pose_module/tests/udp_receiver_test.py`
  - `face_pose_module/tests/landmarker_model_test.py`
  - `face_pose_module/tests/full_pipeline_smoke_test.py`

更新后的测试命令：

- 离线测试：`python tests\run_offline_tests.py`
- D435i 冒烟测试：`python tests\realsense_smoke_test.py --frames 60`
- MediaPipe 模型测试：`python tests\landmarker_model_test.py`
- UDP 接收测试：`python tests\udp_receiver_test.py --seconds 30`
- 完整链路无窗口测试：`python tests\full_pipeline_smoke_test.py --frames 90`

## 2026-06-10 阶段更新：conda 环境与实机冒烟测试

用户要求：使用 conda 创建新环境 `screen_arm`，用于本任务全部环境，并继续完成模块和测试脚本。

已完成：

- 创建 conda 环境：`screen_arm`
- 环境路径：`D:\Anaconda\envs\screen_arm`
- Python 版本：`3.11.15`
- 已安装依赖：
  - `pyrealsense2 2.58.1.10581`
  - `mediapipe 0.10.35`
  - `opencv-python 4.13.0`
  - `numpy 2.4.6`
  - `scipy 1.17.1`
  - `pyyaml 6.0.3`
- 已下载模型文件：`face_pose_module/assets/face_landmarker.task`
- 模型文件大小：`3758596` 字节
- 新增环境文件：`face_pose_module/environment.yml`
- `requirements.txt` 已固定为实测通过版本。

采集层更新：

- `enable_imu=true` 时改用 RealSense callback。
- color/depth 视频帧集进入 `frame_queue`。
- `accel` 和 `gyro` 运动帧保存为最新 IMU 样本。
- `FacePose` 和 UDP payload 已支持可选 `imu` 字段。
- `main.py` 新增 `--max-frames`，便于自动化冒烟测试。

已通过测试：

- 依赖导入检查：通过。
- 编译检查：`D:\Anaconda\envs\screen_arm\python.exe -m compileall face_pose_module` 通过。
- 离线测试：`D:\Anaconda\envs\screen_arm\python.exe tests\run_offline_tests.py` 通过。
- MediaPipe 模型测试：`D:\Anaconda\envs\screen_arm\python.exe tests\landmarker_model_test.py` 通过。
- D435i 冒烟测试：`D:\Anaconda\envs\screen_arm\python.exe tests\realsense_smoke_test.py --frames 60` 通过。
- 完整链路无窗口测试：`D:\Anaconda\envs\screen_arm\python.exe tests\full_pipeline_smoke_test.py --frames 90` 通过。
- 主程序固定帧数测试：`D:\Anaconda\envs\screen_arm\python.exe main.py --config config.yaml --no-window --no-udp --max-frames 30` 通过。

D435i 冒烟测试结果：

- color: `1280x720`
- depth intrinsics: `1280x720`
- frames: `60`
- avg fps: 约 `16.48`
- IMU: 已读取到 `gyro` 和 `accel`

完整链路无窗口测试结果：

- frames: `90`
- avg fps: 约 `11.12`
- valid poses: `0`
- frames with imu: `90`
- statuses: `{'no_face': 90}`
- 说明：测试时画面中未检测到人脸，但完整链路没有崩溃，且每帧均读取到 IMU。后续需要用户正对相机运行可视化主程序，确认有效人脸位姿输出。

主程序固定帧数测试结果：

- 命令：`D:\Anaconda\envs\screen_arm\python.exe main.py --config config.yaml --no-window --no-udp --max-frames 30`
- 结果：正常启动 RealSense，相机采集、MediaPipe 初始化和主循环均可运行并自动退出。

注意事项：

- `1280x720 @ 30fps` 同时开启 IMU 后，当前实测平均帧率约 `16.5 FPS`，后续如果实时性不足，建议降到 `848x480` 或 `640x480`。
- `conda run` 在 Windows GBK 输出环境下可能因为 RealSense/MediaPipe 输出字符触发编码问题，建议直接使用 `D:\Anaconda\envs\screen_arm\python.exe` 或先 `conda activate screen_arm` 后运行命令。
- MediaPipe `VIDEO` 模式要求输入时间戳严格递增，`FaceLandmarkerDetector.detect()` 已做单调递增保护。

## 2026-06-10 阶段更新：降低法向量箭头抖动

用户反馈：可视化窗口中的法向量箭头有抖动，希望方向判断不那么敏感。

处理结果：

- `filters.py` 中的死区逻辑已从整体判断改为分量独立判断。
- `center`、`normal`、`x_axis` 分别使用自己的死区；中心点轻微变化时，不再强制刷新 `normal`。
- `normal` 和 `x_axis` 在进入死区判断前会先和上一帧方向对齐，避免单位向量符号翻转影响角度判断。
- `config.yaml` 调整：
  - `ema_alpha_axis`: `0.35` -> `0.20`
  - `moving_average_window`: `3` -> `5`
  - `normal_deadband_deg`: `1.0` -> `3.0`
  - `x_axis_deadband_deg`: `1.0` -> `2.0`

预期效果：

- 法向量箭头对 3 度以内的小角度变化保持上一帧方向。
- 小幅噪声由 EMA 和滑动平均进一步压低。
- 方向响应会比之前慢一些，但可视化更稳。

验证结果：

- 编译检查通过：`D:\Anaconda\envs\screen_arm\python.exe -m compileall face_pose_module`
- 离线测试通过：`D:\Anaconda\envs\screen_arm\python.exe tests\run_offline_tests.py`
- 已用新配置重启主程序，PID：`3704`

## 2026-06-11 阶段更新：桌子与深度相机模型加入 `screen_arm`

本阶段围绕 `screen_arm` 的 MATLAB/URDF 可视化模型做了桌面环境和深度相机示意模型扩展，目标是让后续机械臂、人脸建模和 RealSense 深度相机坐标关系更接近真实设计。

### 办公桌模型

新增 FreeCAD 生成脚本：

- `screen_arm/scripts/generate_office_desk.py`

生成文件：

- `screen_arm/generated/office_desk.FCStd`
- `screen_arm/generated/step/office_desk.step`
- `screen_arm/generated/meshes/office_desk.stl`
- `screen_arm/generated/visuals/office_desk.stl`

桌子建模约定：

- 单位：FreeCAD/STL/STEP 使用 `mm`，URDF 中 mesh 使用 `scale="0.001 0.001 0.001"` 转成 `m`。
- 桌面尺寸约为 `1400 x 750 x 740 mm`。
- 坐标约定：桌子前后深度沿 `X`，桌面宽度沿 `Y`，桌子正方向/用户侧为 `+X`。
- 桌脚最低点为 `Z=0`，MATLAB/URDF 中 `desk_link` 作为根节点时四个脚贴在仿真地面。
- 桌面上表面高度按 `Z=0.740 m` 使用。

### 桌面固连机械臂

`screen_arm/generated/urdf/face_screen_support_arm.urdf` 已直接覆盖为“桌子 + 机械臂”模型，不再使用单独的桌面固连 URDF 作为主入口。

URDF 中新增：

- `desk_link`
- `desk_to_arm_base` fixed joint

关键固连关系：

```xml
<joint name="desk_to_arm_base" type="fixed">
  <origin xyz="-0.150 0.000 0.740" rpy="0 0 0"/>
  <parent link="desk_link"/><child link="base_link"/>
</joint>
```

含义：

- `base_link` 底面最低点为 `Z=0`，所以 joint 的 `Z=0.740` 让机械臂基座底面贴合桌面。
- `rpy="0 0 0"`，保持基座原来的 `+X` 正方向与桌子 `+X` 正方向一致。

### 关节限制更新

已同步更新 URDF 和 `screen_arm/test/demo_face_screen_arm_joint_sliders.m` 中的滑块范围：

- `J2 joint2_shoulder_pitch`: `[-200 deg, 20 deg]`
- `J3 joint3_elbow_pitch`: `[-170 deg, 170 deg]`

MATLAB 验证过 `importrobot` 后 `PositionLimits` 与上述角度一致。

### 深度相机示意模型

新增 FreeCAD 生成脚本：

- `screen_arm/scripts/generate_depth_camera.py`

生成文件：

- `screen_arm/generated/depth_camera.FCStd`
- `screen_arm/generated/step/depth_camera.step`
- `screen_arm/generated/meshes/depth_camera.stl`
- `screen_arm/generated/visuals/depth_camera.stl`
- `screen_arm/generated/visuals/depth_camera_body.stl`
- `screen_arm/generated/visuals/depth_camera_lens.stl`

深度相机局部模型约定：

- 主体尺寸：`100 x 20 x 24 mm`
- 黑色镜头层尺寸：`70 x 13 x 1.2 mm`
- 镜头层居中贴在相机局部 `+Z` 上表面。
- 整体含镜头层高度：`25.2 mm`
- 局部 mesh 边界：`X[-50, 50] mm`、`Y[-10, 10] mm`、`Z[0, 25.2] mm`。

### 深度相机安装版 URDF

新增 URDF 生成脚本：

- `screen_arm/scripts/generate_depth_camera_mounted_urdf.py`

新增 URDF：

- `screen_arm/generated/urdf/face_screen_support_arm_depth_camera.urdf`

用途：

- 这是“桌子 + 机械臂 + 深度相机”的新版本 URDF。
- 它基于 `face_screen_support_arm.urdf` 生成。
- 在 `base_link` 中移除原来的 `base_front_ref` 小长方体 visual。
- 增加 `depth_camera_body` 和 `depth_camera_lens` 两个 visual。

安装约束：

- 第二层圆柱 `mast` 半径为 `38 mm`。
- 相机底面/背面，即相机局部 `Z=0` 平面，贴在第二层圆柱前侧切平面 `world X=38 mm`。
- 相机镜头面法向，即相机局部 `+Z`，旋转到世界 `+X`。
- 相机长边沿世界 `Y` 横向展开。

当前 URDF visual 位姿：

```xml
<origin xyz="0.038 0.000 0.060" rpy="1.570796 0 1.570796"/>
```

验证结果：

- `local +Z -> world +X`
- `local Z=0 plane -> world X=38.0 mm`
- 相机安装后世界边界约为 `X[38.0, 63.2] mm`、`Y[-50.0, 50.0] mm`、`Z[50.0, 70.0] mm`。

### MATLAB 滑块 Demo 更新

`screen_arm/test/demo_face_screen_arm_joint_sliders.m` 当前导入新模型：

```matlab
urdfPath = fullfile(projectRoot, "generated", "urdf", "face_screen_support_arm_depth_camera.urdf");
```

其它功能保持不变：

- 6 个可动关节滑块仍对应原机械臂关节。
- `homeConfiguration(robot)` 长度仍为 `6`。
- 已优化拖动滑块时相机视角复位问题：重绘前保存 axes 相机参数，重绘后恢复。

### 验证命令记录

已使用以下方式做过检查：

- `F:\FreeCAD\bin\freecadcmd.exe` 运行 FreeCAD Python 生成脚本。
- MATLAB `importrobot` 导入 `face_screen_support_arm.urdf` 和 `face_screen_support_arm_depth_camera.urdf`。
- MATLAB `show(robot, homeConfiguration(robot), ...)` 无窗口显示验证。
- MATLAB `checkcode('screen_arm/test/demo_face_screen_arm_joint_sliders.m')` 无输出问题。

## 2026-06-11 阶段更新：固定人脸中心的视线方向 IK 轨迹测试

本阶段围绕“用户头部中心固定、视线方向变化、屏幕保持合适距离并正对人脸”的仿真验证，新增了根目录测试脚本：

- `test/demo_face_view_target_ik_trajectory.m`

脚本用途：

- 导入当前带办公桌和深度相机的新模型：`screen_arm/generated/urdf/face_screen_support_arm_depth_camera.urdf`。
- 固定测试人脸中心点：`[0.65, 0.00, 1.00] m`。
- 使用 UI 控制人脸法向量方向：
  - `Yaw about world Z`：左右偏摆。
  - `Pitch about world Y`：上下俯仰。
- 人脸法向量在图中仅作为可视化箭头，显示长度为 `0.15 m`。
- 点击 `Plan + Move` 后再规划运动，不在拖动滑块时连续追踪。

目标位姿约定：

- 默认目标距离为 `0.45 m`。
- 可接受距离容忍区间为 `[0.35, 0.55] m`。
- 规划逻辑必须先尝试 `0.45 m` 标称点；只有标称点不可达时，才退化到 `[0.35, 0.55] m` 距离带内搜索可达目标。
- 目标屏幕中心计算方式：

```text
p_screen_target = p_face_center + distance * n_face
```

- 屏幕朝向约束：`screen_center` 局部 `+X` 轴指向人脸中心，使屏幕平面垂直于人脸视线方向。
- 不使用 Roll 控制；脚本通过世界 `Z` 轴投影构造屏幕姿态，使屏幕尽量保持竖直。

逆运动学与轨迹规划实现：

- 使用 MATLAB Robotics System Toolbox 的 `inverseKinematics`。
- 当前 MATLAB 环境中默认求解器算法为 `BFGSGradientProjection`。
- IK 目标末端为 `screen_center`。
- IK 权重：`[0.7, 0.7, 0.7, 1, 1, 1]`。
- 每次 IK 初值使用当前关节角 `state.q`，使相邻目标更倾向于得到连续解。
- 可达判定主要看：
  - 屏幕中心位置误差 `<= 0.025 m`。
  - 屏幕法向误差 `<= 8 deg`。
- 轨迹动画目前为关节空间三次平滑插值：

```text
s = 3 * t^2 - 2 * t^3
q = q_start + (q_goal - q_start) * s
```

- 当前轨迹动画使用 `90` 帧，每帧约 `0.025 s`。

不可达诊断：

- 如果标称距离和容忍距离带内都不可达，UI 状态区会显示最佳失败目标的距离、位置误差、法向误差。
- 脚本额外创建一个 `ikLoose`，其 `EnforceJointLimits = false`，仅用于诊断，不用于真实运动。
- 诊断方式：用不强制关节限位的 IK 估计如果硬要达到该位姿，哪个关节最明显越界。
- UI 会尝试指出：
  - 哪个关节超限。
  - 需要值与关节限制值。
  - 主因更偏向 `yaw/pan` 偏摆限制、`pitch` 俯仰限制，还是 `telescopic` 伸缩距离限制。

验证记录：

- `matlab -batch "issues = checkcode('test/demo_face_view_target_ik_trajectory.m'); disp(issues)"`：无输出问题。
- `matlab -batch "addpath('test'); demo_face_view_target_ik_trajectory('normal'); close all force"`：脚本可导入模型并完成初始绘图。
- 默认方向规划：`0.45 m` 标称距离可达。
- 极端 `[60, 35]` 方向：标称距离不可达时可退化到 `0.35 m` 并成功。
- 极端 `[-60, -35]` 方向：触发不可达诊断，示例诊断为 `J3 Elbow pitch` 超过上限，主因归类为俯仰侧关节限制。

## 2026-06-11 提交记录

已将当前阶段变更提交并推送到 `origin/main`：

- `84ae786 Add face view IK trajectory demo`
  - 包含 `AGENT.md` 入库。
  - 包含办公桌、深度相机、新 URDF、生成脚本和 `test/demo_face_view_target_ik_trajectory.m`。
- `0299457 Update project README`
  - 新增根目录 `README.md`，简要说明项目组成：`face_pose_module`、`screen_arm` 和 `test`。

当前已知状态：

- 以上为本次 `AGENT.md` 记录更新前已经完成并推送的提交记录。
- 后续提交状态以 `git log` 和 `git status -sb` 为准。

## 2026-06-12 阶段更新：人脸建模模块与机械臂仿真的联合跟随测试

本阶段新增根目录联合测试脚本：

- `test/demo_face_pose_screen_arm_live_follow.m`

脚本用途：

- 同时启动 Python 人脸建模模块和 MATLAB 机械臂仿真窗口。
- 仿真中仍固定人脸中心点：`[0.65, 0.00, 1.00] m`。
- 人脸方向不再由 UI 滑块输入，而是读取 Python 人脸建模模块输出的人脸法向量。
- UI 中提供 `开始跟随` / `暂停跟随` 按钮；只有进入跟随状态后，机械臂才会规划并运动到目标位姿。
- 目标屏幕距离仍遵循：优先 `0.45 m`，不可达时退化搜索 `[0.35, 0.55] m`。
- 继承旧脚本的不可达诊断逻辑：使用 `ikLoose` 估计是哪类关节限制主导不可达。

Python 侧接口增强：

- 新增 `face_pose_module/pose_file_writer.py`，用于把最新 `FacePose` 以 JSON 形式原子写入临时文件。
- `face_pose_module/main.py` 新增参数：
  - `--pose-file`
  - `--window-x`
  - `--window-y`
  - `--window-width`
  - `--window-height`
- `face_pose_module/visualizer.py` 支持通过配置/命令行指定 OpenCV 窗口位置和大小。
- 原有 UDP 输出仍保留；联合 demo 默认通过 JSON 姿态文件轮询，避免依赖 MATLAB 的 UDP/Instrument Control Toolbox。

坐标变换约定：

- RealSense 相机坐标使用常规约定：`+X` 图像右方，`+Y` 图像下方，`+Z` 光轴向前。
- 仿真模型中深度相机示意模型默认朝向：相机 `+Z` 对齐世界 `+X`，相机 `+X` 对齐世界 `-Y`。
- 联合脚本使用深度相机示意模型几何中心作为相机坐标系原点，当前计算值约为：

```text
camera center world = [-0.099, 0.000, 0.800] m
```

- D435i 当前输出的是原始 `accel` 和 `gyro`，不是融合后的绝对姿态。
- 因此联合脚本采用“默认相机偏航由摆放位置/仿真安装确定，IMU 加速度重力方向修正镜头上仰和侧倾”的方式估计相机姿态。
- 偏航角不能仅由 D435i IMU 绝对确定；后续如果需要更严谨的相机外参，应增加手眼标定、棋盘格/AprilTag 标定，或引入外部姿态传感器。

跟随抖动抑制：

- Python 人脸模块仍使用原有 EMA、滑动平均和角度死区稳定化。
- MATLAB 联合 demo 额外增加目标变化阈值：
  - 标称目标点变化小于 `0.035 m` 且法向变化小于 `6 deg` 时，不重新规划。
  - 两次运动之间至少间隔 `0.40 s`。
- 轨迹动画相比旧 demo 加快：根据关节变化量使用约 `24-60` 帧，每帧约 `0.012 s`。

可视化：

- 人脸中心、红色人脸法向量、目标屏幕中心、目标位姿三轴箭头沿用旧 demo 风格。
- 人脸中心到目标屏幕中心的中间虚线改为黄色，便于和旧脚本区分。
- OpenCV 人脸建模窗口由 MATLAB 启动时放到屏幕左上角并缩小。
- MATLAB 仿真窗口放到屏幕右下侧并占据较大区域。

验证记录：

- `D:\Anaconda\envs\screen_arm\python.exe -m compileall face_pose_module`：通过。
- `matlab -batch "issues = checkcode('test/demo_face_pose_screen_arm_live_follow.m'); disp(issues)"`：无输出问题。
- `matlab -batch "addpath('test'); demo_face_pose_screen_arm_live_follow('normal', false); pause(1.0); close all force"`：可导入 URDF、打开仿真窗口并建立 timer。
- 默认法向目标触发 `开始跟随`：IK `success`，`0.45 m` 标称距离可达。
- 模拟姿态文件输入 `normal=[0,0,-1]`、`accel=[0,1,0]` 后，MATLAB 转换得到 `faceNormalWorld=[-1,0,0]`，符合当前仿真坐标约定。

### 2026-06-12 追加约定：相机姿态只修正俯仰角

用户确认联合测试阶段的相机姿态约定如下：

- 相机偏摆 `yaw` 默认端正，即镜头面朝世界坐标系 `+X` 方向。
- 相机横滚 `roll` 默认端正，即不考虑镜头画面左右倾斜。
- 相机俯仰 `pitch` 由 D435i IMU 返回信息估计，用于表达现实中相机镜头略微上仰/下俯。

脚本实现更新：

- `test/demo_face_pose_screen_arm_live_follow.m` 中相机姿态计算改为：

```text
R_world_camera = R_nominal * Rx(pitch)
```

- `R_nominal` 仍表示仿真默认安装：相机 `+Z` 光轴对齐世界 `+X`。
- `pitch` 当前由 `imu.accel` 的重力方向估计，只取俯仰分量，不再整体对齐重力向量，因此不会引入 roll 修正。
- 当前 `imu.gyro` 是角速度，不是绝对角度；若后续需要严格使用陀螺仪积分得到俯仰角，需要增加初始零点、时间积分和漂移校正逻辑。

### 2026-06-12 排查增强：人脸建模进程日志与退出检测

用户反馈联合 demo 中相机画面刚打开就关闭。当前本机复查结果：

- `python main.py --config config.yaml --no-window --no-udp --max-frames 30`：通过。
- 带 OpenCV 窗口、窗口位置和尺寸参数运行 `90` 帧：通过。
- 由 MATLAB 联合脚本启动 Python 后等待 `5 s`：`pythonProcess.HasExited = false`，进程未退出。

为便于定位用户现场问题，新增诊断能力：

- `face_pose_module/main.py` 新增 `--log-file` 参数，将 Python 主程序 stdout/stderr 写入日志。
- `test/demo_face_pose_screen_arm_live_follow.m` 启动 Python 时传入日志路径：

```text
%TEMP%/screen_arm_face_pose_live.log
```

- MATLAB timer 会检测 `state.pythonProcess.HasExited`。
- 如果 Python 人脸模块提前退出，MATLAB UI 状态区会显示退出码和日志尾部，同时命令行打印日志路径。

排查建议：

- 用户再次运行 `demo_face_pose_screen_arm_live_follow` 后，如果相机窗口仍立即关闭，优先查看 MATLAB UI 状态区显示的退出码与日志尾部。
- 也可直接打开：

```text
C:\Users\lenovo\AppData\Local\Temp\screen_arm_face_pose_live.log
```

- 若日志为空或没有捕获到进口级异常，再单独在 PowerShell 中运行 Python 主程序复现。

根因定位与修复：

- 日志捕获到 Python 端异常：

```text
[runtime error] [WinError 5] 拒绝访问:
'...screen_arm_face_pose_live.json.tmp' -> '...screen_arm_face_pose_live.json'
```

- 原因是 Windows 下 MATLAB 轮询读取 JSON 文件时，可能短暂占用目标文件，导致 Python `os.replace()` 原子替换失败。
- `face_pose_module/pose_file_writer.py` 已更新：
  - 写 JSON 后执行 `os.replace()`。
  - 如果遇到 `PermissionError`，最多重试 `5` 次，每次间隔 `0.002 s`。
  - 如果仍失败，只跳过当前帧并节流打印 warning，不再让主程序异常退出。
- 修复后验证：
  - `D:\Anaconda\envs\screen_arm\python.exe -m compileall face_pose_module`：通过。
  - 带 `--pose-file`、`--log-file`、`--no-window`、`--no-udp`、`--max-frames 30` 的短运行：通过，姿态 JSON 文件正常生成。

## 2026-06-12 阶段更新：新增 3DDFA_V2 第二版人脸姿态模块

用户提出 v1 的 MediaPipe Face Landmarker 在大角度偏摆下识别困难，因此要求在项目根目录新增独立实验分支：

- `face_pose_module_v2/`

约束：

- 本阶段所有新增代码、文档、配置和测试脚本均放在 `face_pose_module_v2/` 内。
- 不修改 `face_pose_module/` 第一版模块。
- 不破坏 MATLAB 仿真模块。
- 不移动已有 `screen_arm` 机械臂模型。

### v2 总体链路

v2 采用 3DDFA_V2 单目 RGB 头部姿态估计路线：

```text
D435i RGB 视频流 + IMU
  ↓
FaceBoxes 人脸框初始化 / 重捕获
  ↓
3DDFA_V2 三维人脸对齐与头部姿态估计
  ↓
输出 normal / yaw / pitch / roll / imu
  ↓
OpenCV 可视化测试、JSON 文件、UDP
```

关键区别：

- v1 是 `MediaPipe + RGB-D depth`，使用深度反投影三维点并拟合人脸平面。
- v2 是 `FaceBoxes + 3DDFA_V2`，只使用 RGB 回归 3DMM 姿态参数。
- v2 不启动 D435i 深度流。
- v2 仍启动 D435i IMU，保留 `accel` 和 `gyro` 供 MATLAB 侧相机姿态补偿。
- v2 的真实人脸三维位置不可靠，`center` 只是兼容字段和可视化代理点；控制逻辑应主要使用 `normal`。

### 新增目录和主要文件

新增目录：

- `face_pose_module_v2/`

主要文件：

- `face_pose_module_v2/README.md`：中文说明、运行方式、输出字段、测试命令和已知限制。
- `face_pose_module_v2/config.yaml`：RGB 相机、FaceBoxes、3DDFA、滤波、UDP 和可视化参数。
- `face_pose_module_v2/environment.yml`：独立 conda 环境 `screen_arm_v2`。
- `face_pose_module_v2/requirements.txt`：v2 Python 依赖。
- `face_pose_module_v2/main.py`：主入口，串联采集、检测、3DDFA 姿态、滤波、输出和可视化。
- `face_pose_module_v2/camera_realsense_rgb.py`：只启动 RGB + IMU 的 D435i 采集层。
- `face_pose_module_v2/face_detector.py`：FaceBoxes 检测器封装，同时保留 OpenCV Haar 检测器实现。
- `face_pose_module_v2/three_ddfa_pose.py`：3DDFA_V2 适配器，输出 `normal`、`x_axis`、`angles_deg` 和代理 `center`。
- `face_pose_module_v2/pose_types.py`：`FacePose` 数据结构和 JSON/UDP 字段定义。
- `face_pose_module_v2/filters.py`：EMA、滑动平均、角度死区和短时丢失保持。
- `face_pose_module_v2/udp_sender.py`：UDP/JSON 输出。
- `face_pose_module_v2/pose_file_writer.py`：最新姿态 JSON 原子写入。
- `face_pose_module_v2/visualizer.py`：OpenCV 可视化窗口，显示 bbox、landmarks、红色法向轴和蓝色辅助横向轴。
- `face_pose_module_v2/scripts/bootstrap_3ddfa_v2.py`：重新拉取官方 3DDFA_V2 的脚本。

测试脚本：

- `face_pose_module_v2/tests/import_smoke_test.py`
- `face_pose_module_v2/tests/offline_image_pose_test.py`
- `face_pose_module_v2/tests/realsense_rgb_imu_smoke_test.py`
- `face_pose_module_v2/tests/full_pipeline_smoke_test.py`
- `face_pose_module_v2/tests/live_visual_snapshot_test.py`
- `face_pose_module_v2/tests/udp_receiver_test.py`

第三方代码：

- `face_pose_module_v2/third_party/3DDFA_V2/`
- `face_pose_module_v2/third_party/3DDFA_V2_COMMIT.txt`
- `face_pose_module_v2/third_party/README.md`

官方 3DDFA_V2 当前记录提交：

```text
1b6c67601abffc1e9f248b291708aef0e43b55ae
```

### 环境

已创建独立 conda 环境：

- 环境名：`screen_arm_v2`
- 路径：`D:\Anaconda\envs\screen_arm_v2`
- Python：`3.11`

已安装并验证：

- `pyrealsense2==2.58.1.10581`
- `opencv-python==4.13.0.92`
- `numpy==2.4.6`
- `scipy==1.17.1`
- `pyyaml==6.0.3`
- `torch==2.12.0+cpu`
- `torchvision==0.27.0+cpu`
- `matplotlib`

说明：

- v1 的 `screen_arm` 环境不包含 `torch/torchvision`。
- 为避免影响 v1，v2 使用 `screen_arm_v2`。

### 3DDFA_V2 与 FaceBoxes 适配

当前默认配置：

```yaml
detector:
  type: faceboxes

three_ddfa:
  use_onnx: false
  config_path: configs/mb1_120x120.yml
```

使用官方 PyTorch 权重：

- `third_party/3DDFA_V2/weights/mb1_120x120.pth`
- `third_party/3DDFA_V2/configs/bfm_noneck_v3.pkl`

Windows 兼容补丁：

- 官方 `FaceBoxes` 默认导入编译版 `cpu_nms`。
- 当前 Windows 环境没有编译 `cpu_nms` 扩展。
- 已在 `third_party/3DDFA_V2/FaceBoxes/utils/nms_wrapper.py` 加入 fallback：
  - 如果编译版 `cpu_nms` 不存在，则退回官方仓库自带的 `py_cpu_nms.py`。

### 输出字段与最重要内容

v2 输出字段兼容 v1 主字段：

```json
{
  "t": 1781266352.65,
  "valid": true,
  "center": [-0.03, -0.08, 0.65],
  "normal": [-0.83, 0.17, -0.54],
  "x_axis": [0.54, -0.02, -0.84],
  "imu": {
    "accel": [0.08, -9.30, 2.31],
    "gyro": [-0.002, 0.001, 0.003]
  },
  "angles_deg": {
    "yaw": 57.0,
    "pitch": 7.5,
    "roll": 2.5
  },
  "status": "valid_filtered"
}
```

字段含义：

- `t`：时间戳，单位秒。
- `valid`：当前人脸姿态是否有效。
- `center`：代理人脸中心点，单位米；v2 没有深度，因此不是精确三维位置。
- `normal`：最重要字段，人脸法向量/人脸面向方向/红色轴方向，单位向量。
- `x_axis`：蓝色辅助横向轴，用于观察脸部横向姿态，不代表视线方向。
- `imu`：D435i 加速度计和陀螺仪最新样本。
- `angles_deg`：3DDFA 输出的 yaw/pitch/roll，单位度，主要用于调试。
- `status`：状态文本，如 `valid_filtered`、`no_face`、`lost_hold`。

控制层建议优先使用：

1. `valid`
2. `normal`
3. `imu`
4. `angles_deg` 和 `status` 用于调试
5. `center` 仅用于兼容和可视化

### 人脸方向向量起点

v2 中红色人脸方向向量的起点为代理 `center`，不是深度测得的真实人脸中心。

计算逻辑：

1. 3DDFA 输出 68 个人脸关键点。
2. 取左右眼关键点均值：
   - 左眼：`36:42`
   - 右眼：`42:48`
3. 左右眼中心点取中点，得到二维像素 `center_px`。
4. 使用 `assumed_face_distance_m = 0.65` 作为假定深度。
5. 结合 D435i 彩色相机内参反投影：

```text
x = (u - ppx) / fx * 0.65
y = (v - ppy) / fy * 0.65
z = 0.65
```

该三维点用于：

- OpenCV 画红色/蓝色箭头。
- 保持 `center` 字段与 v1 输出兼容。

不建议用于机器人精确位置控制。

### 坐标轴与方向约定

RealSense 彩色相机坐标约定：

- `+X`：图像右方。
- `+Y`：图像下方。
- `+Z`：相机光轴向前，从相机指向用户。

人脸方向约定：

- `normal` 就是人脸法向量。
- `normal` 也是人脸所面向的方向。
- OpenCV 可视化中的红色箭头画的就是 `normal`。
- 正脸看向相机时，`normal` 通常接近 `[0, 0, -1]`。

蓝色轴：

- `x_axis` 为脸部横向辅助参考轴。
- 当前 `visualization.draw_auxiliary_x_axis = true`，因此默认重新显示蓝色轴。
- 蓝色轴不代表视线方向。

### X 方向反向问题与修复

用户实测反馈：

- 头向一边转时，红色箭头会向另一边转。

排查结论：

- 离线样例也复现：人脸明显朝画面左侧，但红色箭头朝画面右侧。
- 原因是 3DDFA 姿态输出到 RealSense/OpenCV 图像坐标时，水平 `X` 分量符号与项目约定相反。
- 这不是单纯的“摄像头画面未镜像”主观问题，而是输出轴映射需要修正。

修复：

- `three_ddfa_pose.py` 新增输出轴修正。
- `config.yaml` 当前默认：

```yaml
flip_output_x: true
flip_output_y: false
```

修复效果：

- 人脸朝画面左侧时，`normal.x < 0`。
- 红色法向箭头也朝画面左侧。
- 此修正作用于姿态输出层，因此 OpenCV 可视化、JSON、UDP 和 MATLAB 后续消费的 `normal` 都同步修正。

### 可视化约定

OpenCV 可视化窗口显示：

- 人脸检测框。
- 3DDFA 68 点 landmarks。
- 人脸中心代理点。
- 红色箭头：`normal`，人脸面向方向。
- 蓝色箭头：`x_axis`，辅助横向轴。
- 文本信息：`valid`、`status`、`FPS`、`yaw/pitch/roll`、`face_dir/red normal`、`center proxy`、`imu`。

当前配置：

```yaml
visualization:
  draw_bbox: true
  draw_landmarks: true
  draw_axes: true
  draw_auxiliary_x_axis: true
```

### 已验证命令

语法检查：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe -m compileall -q face_pose_module_v2
```

导入和模型加载：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe tests\import_smoke_test.py --load-model
```

离线图片测试：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe tests\offline_image_pose_test.py
```

D435i RGB + IMU 冒烟测试：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe tests\realsense_rgb_imu_smoke_test.py --frames 60
```

完整链路无窗口测试：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe tests\full_pipeline_smoke_test.py --frames 20
```

实时可视化快照：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe tests\live_visual_snapshot_test.py --frames 60
```

带姿态 JSON 的主程序固定帧测试：

```powershell
D:\Anaconda\envs\screen_arm_v2\python.exe main.py --config config.yaml --no-window --no-udp --max-frames 60 --pose-file outputs\live_smoke_pose_faceboxes.json
```

### 实测结果

RGB + IMU 冒烟测试：

- color intrinsics: `1280x720`
- `fx ≈ 912.02`
- `fy ≈ 912.15`
- 60 帧读取约 `18.0 FPS`
- 已读取到 `accel` 和 `gyro`

离线图片测试：

- 3DDFA 模型加载成功。
- FaceBoxes 检测成功。
- 姿态输出有效。
- 已生成可视化图片：
  - `face_pose_module_v2/outputs/offline_image_pose.jpg`
  - `face_pose_module_v2/outputs/offline_image_pose_faceboxes.jpg`
  - `face_pose_module_v2/outputs/offline_image_pose_axis_fixed.jpg`
- X 方向修复后，离线样例中人脸朝画面左侧时红色箭头也朝画面左侧。

实时相机测试：

- 当前相机视野中无真实人脸时，FaceBoxes 输出 `valid=false`、`status=no_face`。
- 这比原先 OpenCV Haar 检测器误检右侧物体更合理。
- 已生成实时快照：
  - `face_pose_module_v2/outputs/live_visual_snapshot.jpg`
  - `face_pose_module_v2/outputs/live_visual_snapshot_faceboxes.jpg`

完整链路：

- 无窗口固定帧运行通过。
- `outputs/live_smoke_pose_faceboxes.json` 可正常生成。
- 画面无人脸时 JSON 为 `valid=false`、`status=no_face`，但 RGB、IMU、检测和输出链路没有崩溃。

### 运行方式

可视化主程序：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
conda activate screen_arm_v2
python main.py --config config.yaml --no-udp
```

保存最新姿态 JSON：

```powershell
python main.py --config config.yaml --pose-file "$env:TEMP\screen_arm_face_pose_live_v2.json"
```

固定帧无窗口冒烟测试：

```powershell
python main.py --config config.yaml --no-window --no-udp --max-frames 90
```

保存一帧实时可视化：

```powershell
python tests\live_visual_snapshot_test.py --frames 120
```

无人脸时要求测试失败：

```powershell
python tests\live_visual_snapshot_test.py --frames 120 --require-valid
```

### 已知限制和后续建议

- v2 没有真实深度，`center` 不应作为核心控制依据。
- `normal` 是最重要输出，后续 MATLAB 或机器人控制应优先消费该字段。
- 当前默认 `1280x720 @ 30fps`，实测 RGB + IMU 约 `18 FPS`；若实时性不足，可考虑降到 `848x480` 或 `640x480`。
- 如果后续接入 MATLAB v2 联合 demo，应只改启动路径和环境到 `face_pose_module_v2/main.py` 与 `screen_arm_v2`，不要改机械臂模型。
- 如果实测上下俯仰方向也反，再考虑开启或调整 `flip_output_y`；当前未默认翻转 Y。
