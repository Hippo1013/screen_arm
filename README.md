# 自动跟随用户视线的电脑支架

本项目用于验证“人脸姿态建模 + 屏幕支撑机械臂仿真”的联合链路：RealSense D435i 采集用户脸部姿态，MATLAB 仿真根据人脸方向向量计算屏幕目标位姿，并驱动机械臂让屏幕尽量正对用户。

## 当前模块

```text
face_pose_module/      第一版人脸平面建模模块，使用 RGB-D + MediaPipe
face_pose_module_v2/   第二版人脸姿态模块，使用 RGB + IMU + 3DDFA_V2
screen_arm/            桌面屏幕支撑机械臂模型、URDF、CoppeliaSim 场景和 MATLAB 分析脚本
test/                  第一版联合测试和固定人头目标位姿测试脚本
test_v2/               第二版人脸模块 UDP 联合测试脚本
test_v3/               带 Steve 小人可视化和跟随预览优化的第二版联合测试脚本
```

## 推荐入口

当前联调优先使用 v2 人脸姿态模块和 `test_v3` 仿真脚本：

```matlab
addpath('test_v3', '-begin')
demo_face_pose_screen_arm_live_follow_udp_avatar
```

无相机调试模式：

```matlab
addpath('test_v3', '-begin')
demo_face_pose_screen_arm_live_follow_udp_avatar("large", false)
```

该脚本会使用 UDP 接收 `face_pose_module_v2` 的人脸方向向量和 IMU 信息，并在仿真中绘制：

- 固定人头/人脸参考点；
- 人脸方向箭头；
- 屏幕目标位姿箭头；
- Steve 风格坐姿人物模型；
- 机械臂末端到目标位姿的跟随过程。

## 人脸姿态模块

第一版模块：

```powershell
cd E:\robotics\final_project\ws\face_pose_module
conda activate screen_arm
python main.py --config config.yaml
```

第二版模块：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
conda activate screen_arm_v2
python main.py --config config.yaml --no-udp
```

`test_v2` 和 `test_v3` 联合测试会按脚本配置启动或接收 v2 UDP 数据。v2 输出中，当前 MATLAB 联合控制主要使用：

```text
t
valid
normal
imu.accel
```

其中 `normal` 表示相机坐标系下的人脸方向，`imu.accel` 用于估计相机俯仰角。v2 的 `center` 只是兼容字段，不作为当前机械臂控制的真实人头位置。

## 机械臂模型

机械臂模型在 `screen_arm/` 中，当前仿真使用带桌面和深度相机示意模型的 URDF：

```text
screen_arm/generated/urdf/face_screen_support_arm_depth_camera.urdf
```

常用 MATLAB 模型检查脚本：

```matlab
addpath('screen_arm/test', '-begin')
demo_face_screen_arm_joint_sliders
```

固定人头、手动调节人脸方向并测试 IK/轨迹规划：

```matlab
addpath('test', '-begin')
demo_face_view_target_ik_trajectory
```

## 关键测试脚本

```text
test/demo_face_view_target_ik_trajectory.m
  固定人头位置，通过 UI 调节人脸方向，测试屏幕目标位姿、IK 可达性和轨迹规划。

test/demo_face_pose_screen_arm_live_follow_udp.m
  第一版人脸模块 + 机械臂仿真的 UDP 联合测试。

test_v2/demo_face_pose_screen_arm_live_follow_udp.m
  第二版人脸姿态模块 + 机械臂仿真的 UDP 联合测试。

test_v2/demo_face_pose_screen_arm_live_follow_udp_avatar.m
  在 v2 UDP 联合测试基础上加入 Steve 小人头部可视化。

test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m
  当前推荐脚本；保留 Steve 小人，运动期间继续更新人脸预览，并加入软视角保护。
```

## 关键参数与后续方向

- [关键参数.md](关键参数.md)：记录相机示意模型位置、固定人头参考点、初始末端位置和现实测试摆放距离。
- [后续开发方向.md](后续开发方向.md)：记录从当前“离散触发式跟随”升级到“实时视觉伺服式连续跟随”的设计方向。
- [AGENT.md](AGENT.md)：记录项目开发过程、调试结论和后续 agent 需要知道的实现细节。

## 当前重要约定

- 固定人头/人脸参考点暂用 `state.faceCenter = [0.65, 0.00, 1.00]`。
- 默认屏幕观看距离优先尝试 `0.45 m`。
- 当前可接受人脸到屏幕中心距离范围为 `[0.30, 0.60] m`。
- 仿真中的相机位置固定为机械臂基座上的深度相机示意模型中心。
- 相机 `yaw` 和 `roll` 默认端正，`pitch` 由 D435i IMU 的加速度方向估计。
- 当前 `test_v3` 仍是“离散触发式跟随”，不是完整实时连续伺服。

## 常用验证命令

MATLAB 静态检查：

```powershell
matlab -batch "issues=[checkcode('test_v2/demo_face_pose_screen_arm_live_follow_udp_avatar.m'); checkcode('test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m')]; if isempty(issues), disp('checkcode clean'); else, disp(issues); error('checkcode reported issues'); end"
```

v2 人脸模块导入和模型加载检查：

```powershell
cd E:\robotics\final_project\ws\face_pose_module_v2
conda activate screen_arm_v2
python tests\import_smoke_test.py --load-model
```
