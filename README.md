# Face Screen Support Arm

这是一个面向桌面场景的屏幕支撑机械臂项目，包含 CoppeliaSim 场景、机器人模型、URDF、网格/CAD 文件，以及用于生成、导入、修正和分析机械臂的脚本。

## 推荐入口

优先打开以下 CoppeliaSim 场景文件：

```text
coppeliasim/face_screen_support_arm_scene.ttt
```

该场景已包含桌面、坐姿用户参考、屏幕支撑机械臂模型和 UI 控制面板。

## 项目内容

```text
coppeliasim/face_screen_support_arm_scene.ttt  最终 CoppeliaSim 场景
coppeliasim/face_screen_support_arm.ttm        机器人模型文件
generated/urdf/face_screen_support_arm.urdf    URDF 模型
generated/meshes/*.stl                         连杆网格文件
generated/step/*.step                          CAD STEP 文件
generated/face_screen_arm.FCStd                FreeCAD 源文件
scripts/                                       生成、导入和修正脚本
test/*.m                                       MATLAB 分析与演示脚本
```

## 初始位姿

默认初始位姿如下：

```text
J2 = -120 deg
J3 = 120 deg
J4 = 30 mm
```

## 使用方式

### CoppeliaSim

1. 打开 `coppeliasim/face_screen_support_arm_scene.ttt`。
2. 运行场景。
3. 使用场景内 UI 控制面板调整机械臂关节。

### MATLAB

项目提供了多个 MATLAB 演示与分析脚本，统一放在 `test/` 目录中。

打开 URDF 并显示机械臂：

```matlab
addpath("test")
open_face_screen_arm_matlab("home")
```

可用位姿包括：

```text
home
left
right
near
far
```

其他常用脚本：

```text
test/demo_face_screen_arm_joint_sliders.m
test/demo_face_screen_arm_motion_loop.m
test/demo_face_screen_arm_target_pose.m
test/analyze_face_screen_arm_workspace.m
test/analyze_face_screen_arm_face_shell.m
```

### FreeCAD 生成模型

模型生成脚本位于：

```text
scripts/generate_face_screen_arm.py
```

该脚本会生成 `generated/` 下的网格、STEP、URDF 和 FreeCAD 文件。运行前需要具备可用的 FreeCAD Python 环境。

## 注意事项

- `generated/` 目录中的文件是当前项目可直接使用的生成结果，已纳入仓库。
- `matlab_open_face_screen_arm.log` 等运行日志不会提交到仓库。
- CoppeliaSim 场景中可见机器人底座相对桌面有约 `1 mm` 的轻微预压，用于避免渲染间隙。
- MATLAB 脚本依赖 `importrobot` 等机器人相关功能，通常需要 Robotics System Toolbox。
