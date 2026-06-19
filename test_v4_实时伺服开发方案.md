# test_v4 实时人脸跟随（视觉伺服）开发方案

> **本文件的读者是负责实现该功能的编程 agent。** 你不需要看过任何历史对话即可据此实现。
> 实现目标：在 `test_v4/` 下新建一个**连续视觉伺服**版本的人脸跟随演示，取代 `test_v3` 的「离散触发式跟随」。
>
> **约束（务必遵守）：**
> 1. **不得修改 `test_v3/`**，它是稳定对照组，必须保持可运行。
> 2. v4 是**自包含单文件**（沿用本项目每个 `test_vN` 各自独立的风格），所有要复用的 v3 函数**复制**进 v4，不抽公共库（公共库重构是另一个独立议题，不在本任务范围）。
> 3. 第 5 节「不变量」与第 9 节「参数总表」是**硬契约**，代码骨架（第 8 节）是参考实现，可按需调整，但不得违反不变量。
> 4. 涉及方向/单位/行序的地方，先按第 5 节核对再写，这些是本任务最容易出 bug 的点。

记录日期：2026-06-19

---

## 1. 前置知识与环境

- **平台**：Windows 11 + MATLAB（含 Robotics System Toolbox，需要 `importrobot` / `inverseKinematics` / `getTransform` / `geometricJacobian` / `show`）。
- **机器人**：URDF 在 [screen_arm/generated/urdf/face_screen_support_arm_depth_camera.urdf](screen_arm/generated/urdf/face_screen_support_arm_depth_camera.urdf)。末端坐标系名为 `screen_center`（屏幕中心）。
- **感知**：Python 模块 [face_pose_module_v2/](face_pose_module_v2/)（3DDFA_V2 人脸姿态），通过 UDP/JSON 发到 `127.0.0.1:5005`。无需改 Python 端。
- **现有最佳实现**：[test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m)（下文简称 **v3**，所有行号引用均指该文件）。**先通读 v3 再动手**，v4 的 80% 基础设施直接复用它。
- **背景文档**：[后续开发方向.md](后续开发方向.md)（提出本目标）、[关键参数.md](关键参数.md)、[AGENT.md](AGENT.md)（开发日志）。

---

## 2. 背景：v3 是什么，v4 要变成什么

**v3（离散触发式）**：读最新人脸帧 → 判断变化是否超阈值且臂空闲 → 锁定这一帧 → 完整逆运动学求一个目标关节角 → 关节空间插值轨迹走完 → 到达后再看下一帧。
- 优点：稳、好诊断、能滤抖。
- 缺点：动作「一顿一顿」；运动途中的新姿态只刷新箭头，不改当前轨迹。

**v4（连续视觉伺服式）**：每个控制周期都读最新人脸方向 → 滤波得到目标位姿 → **用差分逆运动学只走一小步** → 下一周期再根据最新目标修正。
- 屏幕末端随人脸方向**连续**调整位置与姿态；
- 始终尽量让屏幕正对人脸并保持合适距离；
- 靠**关节限速**而非完整重规划来抑制抖动。

---

## 3. 已锁定的设计决策（不要擅自更改）

| 决策点 | 选定方案 | 理由 |
|---|---|---|
| **差分 IK 方法** | **雅可比阻尼最小二乘（DLS）速度伺服** 为核心。完整 `inverseKinematics` 仅用于「接管时粗到位」和「看门狗恢复」 | 每拍跑完整 IK 太慢（10–50ms）撑不起 25Hz；DLS 每拍只算一次雅可比 + 一个 6×6 解算（毫秒级），λ 天然处理奇异、可直接限速 |
| **控制频率** | 单 timer，周期 **0.04s（25Hz）** | MATLAB timer 在 <0.03s 不可靠；25Hz 是稳妥起点 |
| **渲染频率** | 机械臂 `show(...)` **每 2 拍一次（≈12Hz）**；人物头部/目标坐标轴每拍更新 | `show(robot)` 是最大开销，降频；轻量 overlay 可每拍 |
| **控制/渲染解耦** | **单一 timer + 计数器降频**，**不开第二个 timer** | MATLAB 单线程，双 timer 会放大重入问题（v3 已被预览重入坑过） |
| **滤波** | **分层轻滤波**：Python 端主滤波保留；MATLAB 端只做①目标法向量 slerp 限速 + 死区，②IMU 俯仰角低通；**真正的防抖保证是控制层的关节限速** | 避免双重重滤波导致跟随发「黏」 |
| **接管策略** | **仅接管瞬间「粗到位」**（离散 IK + 平滑轨迹送到附近），之后**纯伺服**跟随。看门狗仅在「持续失稳」时触发一次粗到位恢复（安全网，非常规大跳变处理） | 纯小步伺服从远处起步太慢；不限步会跳。粗→精是标准做法 |
| **屏幕滚转(roll)** | **严格正立**：用 v3 的 `buildTargetTformFromNormal` 完整目标位姿帧，姿态各分量权重均等，不放松绕视线轴的 roll | 屏幕显示的人脸不应歪；只在可达性吃紧时才考虑放松（本期不做） |
| **距离调节** | v4 第一版固定标称 **0.45m**，不可达时由 DLS 自动「尽量靠近」并 UI 标记；**距离沿视线浮动（在 `[0.30,0.60]`）作为低速二级回退，列为 Phase 5 增强** | 每拍重跑 11 点距离搜索（v3 做法）不可能实时；DLS 残差天然「尽量靠近」 |

---

## 4. 总体架构

```text
                    ┌─────────────── 单一 control timer（Period=0.04s, BusyMode=drop）──────────────┐
 face_pose_module_v2 │  每拍（轻量）：                                                                │
   ──UDP/JSON──▶     │   1. 读 UDP 最新帧（复用 readLatestUdpPose，已自动丢旧包只留最新）              │
 127.0.0.1:5005      │   2. 有效则：相机系→世界系法向量 + IMU 俯仰角（复用 faceNormalWorldFromPose）   │
                     │   3. 滤波：法向量 slerp 限速 + 死区；俯仰角低通（v4 新增）                       │
                     │   4. 目标位姿 T_target（复用 buildTargetTformFromNormal，严格正立）             │
                     │   5. 【仅 servo 模式】差分 IK 一步：e=pose_error → DLS → qdot → 限速/限位 → q   │
                     │   6. 人物头部朝向 + 目标坐标轴（复用 ensure/update/refresh，轻量）              │
                     │                                                                              │
                     │  每 2 拍（重）：show(robot, q, FastUpdate) 重绘机械臂（复用 redrawRobot）       │
                     └──────────────────────────────────────────────────────────────────────────┘
```

**核心理念**：控制是每拍的便宜操作；重绘机械臂是隔拍才做的昂贵操作；用 `state.busy` 串行化整个 tick，避免重入。

---

## 5. 必须遵守的不变量（correctness-critical，先核对再写代码）

### 5.1 关节结构（混合 6 自由度）
机器人是 **6 自由度混合链**：关节 1/2/3/5/6 是**旋转**（单位 rad），**关节 4 是棱柱/平移**（单位 m）。证据见 v3 的 `displayPoseToConfig`（[行 1652-1660](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1652)）：`q(4)=displayValues(4)/1000`（mm→m），其余 `deg2rad`。
- `q` 是 **6×1 列向量**（`robot.DataFormat="column"`）。Home 位姿 = `displayPoseToConfig([0,-120,120,30,0,0]).'`。
- **不要硬编码哪个关节是棱柱**：运行时用 `movingJointInfo(robot)`（[行 1618](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1618)）取 `names/types/lower/upper`，按 `types=="prismatic"` 判定，对该关节用**米/秒**的限速，对旋转关节用**弧度/秒**。
- 关节限位 `lower/upper` 可能含 `±Inf`（连续旋转关节），`min/max` clamp 对 `Inf` 仍正确。

### 5.2 末端与目标位姿帧（严格正立）
- 末端：`endEffector = "screen_center"`。当前末端位姿 `T = getTransform(robot, q, "screen_center")`（4×4，世界系）。
- 目标位姿用 v3 的 `buildTargetTformFromNormal(faceCenter, distance, faceNormalWorld)`（[行 1329](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1329)），它定义：
  - 屏幕本地 **+X 轴指向人脸**（`xAxis = faceCenter - targetPoint`，归一化）；
  - 屏幕本地 **+Z 轴 = 世界上方在垂直于 X 平面内的投影**（保证画面正立）；
  - `targetPoint = faceCenter + distance * normal`。
- **严格正立 = 直接用这个完整 4×4 目标帧，姿态三分量权重均等，不要放松 roll。**

### 5.3 坐标系与方向（沿用 v3，不要改）
- 固定人脸参考点 `faceCenter = [0.65, 0.00, 1.00]`（世界系，米）。
- 相机位置 `depthCameraCenterWorld()`（[行 1731](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1731)）。
- 相机系→世界系标称旋转 `cameraNominalRotationWorldFromCamera()`（[行 1739](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1739)）= `[0 0 1; -1 0 0; 0 -1 0]`（RealSense：+X 右、+Y 下、+Z 前；映射到世界 +Z_cam→+X_world）。
- 相机 yaw 端正（朝世界 +X）、roll=0，**pitch 由 IMU 加速度计估计**（`cameraRotationFromImu`，[行 1400](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1400)）。
- `faceNormalWorld = R_worldFromCamera * normalCamera`，归一化后**保留符号**（v2 的 normal 已是「人脸朝向」方向）。默认 `[-1,0,0]`（人看向机械臂方向）。
- 距离带 `distanceRange = [0.30, 0.60]`，标称 `viewDistance = 0.45`。

### 5.4 UDP 协议（机器人控制只依赖这几个字段）
复用 `readLatestUdpPose`（[行 1447](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1447)）+ `extractRequiredFacePosePayload`（[行 1480](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1480)）。每个数据报是一条 UTF-8 JSON：
```json
{ "t": 1234.567, "valid": true, "normal": [nx, ny, nz], "imu": { "accel": [ax, ay, az] } }
```
- `t`：单调递增时间戳（秒）。**只处理 `t > lastPoseTimestamp` 的新帧**；旧帧丢弃。
- `valid`：布尔。无效帧不更新目标（见 8.2 丢帧保持）。
- `normal`：**相机系**人脸方向向量（3 元）。
- `imu.accel`：可选，相机系加速度（m/s²，重力在相机系约为 +Y）。
- `readLatestUdpPose` 已经把 socket 缓冲区抽干、只返回**最新**一帧——「只保留最新 UDP 帧」的要求已满足，无需再写。

### 5.5 雅可比的行序（最易踩坑，务必看）
MATLAB `J = geometricJacobian(robot, q, "screen_center")` 返回 **6×6** 矩阵，在**基座（世界）系**下表达：
- **前 3 行 = 角速度项**，**后 3 行 = 线速度项**。
- 因此位姿误差 6 维向量必须排成 `e = [e_角(3); e_线(3)]`，顺序排反会让整个伺服失控。
- 误差与雅可比都在**世界系**，必须一致（`getTransform`、`buildTargetTformFromNormal` 都是世界系，OK）。

---

## 6. 文件组织与「复用 / 新写」映射

新建：`test_v4/demo_face_pose_screen_arm_realtime_servo_udp_avatar.m`，单文件。运行方式与 v3 一致：
```matlab
addpath('test_v4', '-begin')
demo_face_pose_screen_arm_realtime_servo_udp_avatar              % 默认 large 工作空间，自动起相机
demo_face_pose_screen_arm_realtime_servo_udp_avatar("normal")
demo_face_pose_screen_arm_realtime_servo_udp_avatar("normal", false)  % 不起相机进程（配合合成发包器测试）
```

### 6.1 从 v3 直接复制（几乎照搬，按函数名+行号）

| 类别 | 函数（v3 行号） |
|---|---|
| 机器人/IK 初始化 | 主函数 [行 25-36](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L25)（`importrobot`、`DataFormat="column"`、`ik`、`ikLoose`） |
| 窗口/坐标轴/布局 | `makeWindowLayout`([1777]) · `setupAxes`([1699]) · `workspaceLimits`([1712]) · `captureAxesCamera`([1673]) · `restoreAxesCamera`([1687]) |
| UDP | `createUdpPoseReceiver`([1745]) · `closeUdpPoseReceiver`([1766]) · `readLatestUdpPose`([1447]) · `extractRequiredFacePosePayload`([1480]) · `isUdpReceiveTimeout`([1506]) |
| 坐标变换 | `faceNormalWorldFromPose`([1368]) · `cameraRotationFromImu`([1400]) · `rotxLocal`([1426]) · `buildTargetTformFromNormal`([1329]) · `numericVectorField`([1432]) · `numericVector`([1440]) · `depthCameraCenterWorld`([1731]) · `cameraNominalRotationWorldFromCamera`([1739]) · `vectorAngle`([1725]) |
| 误差度量 | `poseErrors`([1355])（用于诊断 UI；伺服用自己的 6 维误差，见 8.5） |
| 渲染（建一次小人 + 头部 hgtransform + 轻量 overlay） | `redrawRobot`([709]) · `ensurePreviewStatics`([768]) · `updateHeadOrientation`([802]) · `refreshTargetOverlay`([810]) · `emptyAvatarHandles`([874]) · `avatarStaticsValid`([885]) · `deleteAvatarHandles`([892]) · 全部 `drawSteve*`/`drawCuboid`/`steveHead*`/`avatarTransform`([905-1320]) · `deleteGraphics`([1321]) |
| 关节/导出/状态 | `movingJointInfo`([1618]) · `displayPoseToConfig`([1652]) · `setStatus`([1662]) · `assignTargetToBase`([1643]) · `exportState`([1600]) · `updatePoseUi`([1564]) |
| 进程/生命周期 | `startPythonFaceModule`([1805]) · `pythonProcessExited`([1511]) · `tailTextFile`([1545]) · `closeDemo`([1840]，需按 8.1/8.9 调整清理项) |
| 接管粗到位用 | `solveTargetWithDistanceFallback`([426]) · `solveSingleDistance`([455]) · `candidateDistances`([495]) · `animateJointTrajectory`([668]) · `smoothStep`([704]) |

### 6.2 v4 新写 / 改写

| 函数 | 职责 | 章节 |
|---|---|---|
| `controlTick(fig)` | 单 timer 主回调：感知→滤波→目标→（servo 模式）伺服一步→渲染（降频）。**替代** v3 的 `timerTick`/`processLatestFacePose`/`planAndMove` 那套触发逻辑 | 8.1 |
| `ingestLatestPose(fig)` | 读 UDP 最新帧 + 有效性 + 丢帧保持，更新滤波后法向量与俯仰角 | 8.2 |
| `filterTargetNormal(...)` / `slerpVector(...)` | 法向量 slerp 限速 + 死区 | 8.3 |
| `lowpassPitch(...)` | IMU 俯仰角 EMA 低通 | 8.3 |
| `servoOneStep(fig, T_target)` | 差分 IK 内核（DLS + 行序 + 奇异阻尼），输出 qdot | 8.5 |
| `limitJointStep(...)` | 关节限速（分旋转/棱柱单位）+ 限位 clamp | 8.6 |
| `engageFollow(fig)` | 接管：粗到位（离散 IK + 平滑轨迹）→ 切 servo | 8.8 |
| `servoWatchdog(...)` | 持续失稳检测 → 触发一次粗到位恢复 | 8.8 |
| `renderScene(fig, forceRobot)` | 统一渲染（头/箭头每拍，机械臂降频），假定调用方已持 `state.busy` | 8.9 |
| `toggleFollow(fig)` | 改为切换 servo 模式（调用 `engageFollow` / 停止） | 8.8 |
| `updateServoUi(fig, ...)` | 伺服诊断面板（误差/速度/可操作度/实测频率/可达性灯） | 8.10 |
| `createControls(fig,panel)` | 在 v3 基础上增删按钮与诊断文本 | 8.10 |
| `clearBusy(fig)` | 配合 `onCleanup` 释放 `state.busy` | 8.1 |

---

## 7. state 结构（在 v3 setup 基础上增删）

保留 v3 [行 75-134](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L75) 的大部分字段（robot/ik/ikLoose/endEffector/weights/jointInfo/ax/q/faceCenter/viewDistance/targetDistance/distanceRange/箭头长度/steveHeadSize/cameraPositionWorld/cameraNominalRotation/udp*/python*/avatar/targetHandles/lastTargetTform/lastTargetPoint/poseText/imuText/statusText/followButton/windowLayout/clock 等）。

**移除**（v3 离散触发专用，v4 不再需要）：
`positionTolerance`/`normalTolerance` 仅保留给诊断；`motionPositionThreshold`/`motionNormalThreshold`/`minMoveIntervalSeconds`/`lastCommandFaceNormal`/`lastCommandTargetPoint`/`lastMoveSeconds`/`isMoving`/`previewBusy`/`previewUpdateIntervalSeconds`/`lastPreviewUpdateSeconds` → 删除或被下列替换。

**新增字段（v4）**：
```matlab
state.mode               = "idle";        % "idle" | "engaging" | "servo"
state.busy               = false;         % 串行锁：controlTick 与按钮回调互斥
state.controlPeriod      = 0.04;          % s（timer Period）
state.renderEvery        = 2;             % 每 N 拍重绘机械臂
state.renderCounter      = 0;

% 滤波
state.normalFilt         = [-1,0,0];      % 滤波后世界系法向量（伺服真正跟随的量）
state.pitchFilt          = 0;             % 滤波后 IMU 俯仰角(rad)
state.pitchFiltInit      = false;
state.normalDeadbandRad  = deg2rad(1.0);  % 法向量死区
state.normalSlerpAlphaMin= 0.15;          % 小变化：弱跟随抑抖
state.normalSlerpAlphaMax= 0.55;          % 大变化：强跟随
state.normalSlerpAngleFull = deg2rad(20); % 达到该夹角时用 alphaMax
state.normalMaxRateRad   = deg2rad(120);  % 目标法向量最大角速度(rad/s)
state.pitchLpAlpha       = 0.20;          % 俯仰角 EMA 系数

% 伺服增益与限幅
state.servoKpOri         = 1.5;           % 1/s，姿态比例增益
state.servoKpPos         = 2.0;           % 1/s，位置比例增益
state.lambdaMin          = 1e-3;          % DLS 阻尼下限
state.lambdaMax          = 0.05;          % DLS 阻尼上限（奇异附近）
state.manipW0            = [];            % 可操作度阈值，运行时标定（见 8.5）
state.qdotMaxRev         = deg2rad(70);   % 旋转关节限速(rad/s)
state.qdotMaxPris        = 0.15;          % 棱柱关节限速(m/s)
state.qdotMax            = [];            % 6×1，setup 时按 jointInfo.types 组装

% 丢帧/失稳
state.lastValidPoseTime  = -Inf;          % toc(clock) 时刻
state.holdTimeoutSec     = 0.5;           % 超过则 UI 标"信号暂失，保持"
state.watchPosErr        = Inf;           % 上一拍位置误差(用于看门狗)
state.watchStuckTicks    = 0;
state.watchStuckLimit    = 25;            % ≈1s @25Hz：持续不收敛→恢复

% 诊断
state.lastTickTime       = -Inf;          % 实测控制频率
state.measRateHz         = 0;
state.servoText          = gobjects(1);   % 新增诊断文本控件
```
`state.qdotMax` 在 setup 时组装：
```matlab
qdotMax = zeros(numel(state.jointInfo.types),1);
for i=1:numel(qdotMax)
    if state.jointInfo.types(i)=="prismatic", qdotMax(i)=state.qdotMaxPris;
    else, qdotMax(i)=state.qdotMaxRev; end
end
state.qdotMax = qdotMax;
```

**setup 顺序保持 v3 的关键约定**（[行 137-142](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L137)）：先 `redrawRobot`（hold off，让 `show` 安装场景灯光），**再** `hold(ax,"on")`，再首帧渲染 overlay。**这条顺序不能动**，否则桌子/机械臂会丢光影（v3 曾因此回归）。

---

## 8. 模块实现规范

### 8.1 主控制循环 `controlTick`

并发模型：MATLAB 单线程协作式。timer（`BusyMode="drop"`）+ `state.busy` 串行锁防止 tick 与按钮回调在 `drawnow` 处互相打断重入。所有会改 `q`/渲染的入口（`controlTick`/`engageFollow`/`resetHome`）都遵循「入口检查 busy → 置 busy → `onCleanup` 释放」。

```matlab
function controlTick(fig)
if ~isvalid(fig), return, end
state = guidata(fig);
if state.startFaceModule && pythonProcessExited(fig), return, end
if state.busy, return, end                 % 串行：有正在进行的 tick/回调则丢弃本拍
state.busy = true; guidata(fig, state);
guard = onCleanup(@() clearBusy(fig));      % 任何退出(含报错)都释放锁

% 实测控制频率
state = guidata(fig);
nowT = toc(state.clock);
if isfinite(state.lastTickTime)
    dtMeas = nowT - state.lastTickTime;
    if dtMeas > 0, state.measRateHz = 0.9*state.measRateHz + 0.1*(1/dtMeas); end
end
state.lastTickTime = nowT; guidata(fig, state);

% 1) 感知 + 滤波（每拍，轻）
[hasValid, holding] = ingestLatestPose(fig);

% 2) 目标位姿（每拍）
state = guidata(fig);
[T_target, targetPoint, faceNormal] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.normalFilt);
state.lastTargetTform = T_target; state.lastTargetPoint = targetPoint;
guidata(fig, state);

% 3) 控制（仅 servo 模式）
state = guidata(fig);
if state.mode == "servo"
    servoOneStep(fig, T_target);           % 内部更新 state.q 并跑看门狗
end

% 4) 渲染（头/箭头每拍；机械臂每 renderEvery 拍）
state = guidata(fig);
state.renderCounter = state.renderCounter + 1;
forceRobot = (state.mode=="servo") && (mod(state.renderCounter, state.renderEvery)==0);
guidata(fig, state);
renderScene(fig, forceRobot);

% 5) UI + 导出
updateServoUi(fig, hasValid, holding);
updatePoseUi(fig, hasValid, holdingInfoText(holding));   % 复用 v3 文本，holding 文案自定
assignTargetToBase(fig);
drawnow limitrate
end

function clearBusy(fig)
if ~isvalid(fig), return, end
s = guidata(fig); s.busy = false; guidata(fig, s);
end
```
- `engageFollow`/`resetHome` 在自己执行期间会阻塞，并应同样持 `busy`，使 timer 本拍被丢弃。
- timer 创建：复用 v3 [行 155-161](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L155)，把 `Period` 设 `state.controlPeriod`、`TimerFcn=@(~,~)controlTick(fig)`。

### 8.2 感知 + 丢帧保持 `ingestLatestPose`

```matlab
function [hasValid, holding] = ingestLatestPose(fig)
hasValid = false; holding = false;
state = guidata(fig);
pose = readLatestUdpPose(state.udpPoseReceiver);   % 已抽干缓冲，只剩最新
if isempty(pose) || ~isfield(pose,"t") || double(pose.t) <= state.lastPoseTimestamp
    % 没有新帧：看是否超时进入"保持"
    holding = (toc(state.clock) - state.lastValidPoseTime) > state.holdTimeoutSec;
    return
end
state.lastPoseTimestamp = double(pose.t);
state.latestPose = pose;

if ~isfield(pose,"valid") || ~logical(pose.valid)
    guidata(fig, state);
    holding = (toc(state.clock) - state.lastValidPoseTime) > state.holdTimeoutSec;
    return                                  % 无效帧：保持上一个滤波目标，不清零
end

[normalWorldRaw, imuInfo] = faceNormalWorldFromPose(pose, state);  % 复用 v3（含 IMU pitch）
if isempty(normalWorldRaw)
    guidata(fig, state); return
end

% 滤波：法向量 slerp 限速 + 死区（俯仰角已在 faceNormalWorldFromPose 内部用过；
%       若需要单独平滑俯仰，改造 cameraRotationFromImu 接收 state.pitchFilt，见 8.3）
state.normalFilt = filterTargetNormal(state.normalFilt, normalWorldRaw, state);
state.faceNormalWorld = state.normalFilt;          % 与 v3 字段对齐，供 UI/导出
state.lastValidPoseTime = toc(state.clock);
state.imuInfoText = imuInfo;
guidata(fig, state);
hasValid = true;
end
```
> 实现注意：v3 的 `faceNormalWorldFromPose` 每帧硬算 IMU 俯仰角。要让俯仰角也平滑，见 8.3 的「方案 B」对 `cameraRotationFromImu` 做小改造（推荐），否则至少保证法向量 slerp 已能抑制大部分末端抖动。

### 8.3 滤波层

**法向量 slerp 限速 + 死区**：
```matlab
function nf = filterTargetNormal(nPrev, nNew, state)
nPrev = nPrev(:)/norm(nPrev); nNew = nNew(:)/norm(nNew);
ang = acos(max(-1,min(1,dot(nPrev,nNew))));
if ang < state.normalDeadbandRad           % 死区：忽略微小抖动
    nf = nPrev.'; return
end
% 自适应 alpha：小变化弱跟随、大变化强跟随
a = state.normalSlerpAlphaMin + (state.normalSlerpAlphaMax-state.normalSlerpAlphaMin) ...
        * min(1, ang/state.normalSlerpAngleFull);
% 角速度上限：限制本拍步进角不超过 maxRate*dt
maxStep = state.normalMaxRateRad * state.controlPeriod;
if a*ang > maxStep, a = maxStep/ang; end
nf = slerpVector(nPrev, nNew, a).';
end

function v = slerpVector(v0, v1, alpha)
v0=v0(:)/norm(v0); v1=v1(:)/norm(v1);
d = max(-1,min(1,dot(v0,v1))); th = acos(d);
if th < 1e-6, v = v1.'; return, end
v = (sin((1-alpha)*th)*v0 + sin(alpha*th)*v1)/sin(th);
v = (v/norm(v)).';
end
```

**IMU 俯仰角低通**（方案 B，推荐）：把 v3 的 `cameraRotationFromImu` 改造成接收并更新 `state.pitchFilt`——在算出瞬时 `pitchAngle` 后做 EMA：
```matlab
% 在 cameraRotationFromImu 内，得到瞬时 pitchAngle 后：
if ~state.pitchFiltInit
    state.pitchFilt = pitchAngle; state.pitchFiltInit = true;
else
    state.pitchFilt = (1-state.pitchLpAlpha)*state.pitchFilt + state.pitchLpAlpha*pitchAngle;
end
rotationWorldFromCamera = rotationNominal * rotxLocal(state.pitchFilt);
```
（因该函数当前是无状态的纯函数，方案 B 需要把 `state` 读写进来，或把平滑后的 pitch 在 `ingestLatestPose` 里单独做。二选一即可，关键是**俯仰角必须平滑**，否则 accel 噪声会让末端上下抖。）

### 8.4 目标位姿（复用，严格正立）
直接调用 `buildTargetTformFromNormal(state.faceCenter, state.targetDistance, state.normalFilt)`（5.2）。**不做任何 roll 放松**。`targetDistance` 第一版恒为 `viewDistance=0.45`（不可达退化见 8.7）。

### 8.5 差分 IK 内核 `servoOneStep`（★ 核心）

```matlab
function servoOneStep(fig, T_target)
state = guidata(fig);
q  = state.q;                               % 6×1 列
ee = state.endEffector;

% 当前末端位姿
T_cur = getTransform(state.robot, q, ee);
p_cur = T_cur(1:3,4);  R_cur = T_cur(1:3,1:3);
p_tar = T_target(1:3,4); R_tar = T_target(1:3,1:3);

% 6 维位姿误差，世界系，顺序 = [角; 线]（与 geometricJacobian 行序一致！见 5.5）
e_pos = p_tar - p_cur;                                  % 线
Rerr  = R_tar * R_cur.';                                % 世界系旋转误差
axang = rotm2axang(Rerr);                               % [ax ay az theta]
e_ori = (axang(1:3).') * axang(4);                      % 角(旋转矢量)
e = [e_ori; e_pos];                                     % 6×1 [角(3); 线(3)]

% 期望空间速度（比例控制）
vDes = [state.servoKpOri*e_ori; state.servoKpPos*e_pos];

% 雅可比 + 自适应阻尼最小二乘
J = geometricJacobian(state.robot, q, ee);              % 6×6，基座系，[角;线]
w = sqrt(max(0, det(J*J.')));                           % 可操作度
if isempty(state.manipW0)                               % 首次：自标定阈值
    state.manipW0 = max(w*0.15, 1e-6);
end
if w < state.manipW0
    lambda2 = state.lambdaMax^2 * (1 - w/state.manipW0)^2 + state.lambdaMin^2;
else
    lambda2 = state.lambdaMin^2;
end
qdot = J.' * ((J*J.' + lambda2*eye(6)) \ vDes);         % 6×1 关节速度(混合单位)

% 限速 + 限位 → 更新 q
[dq, qdot] = limitJointStep(qdot, q, state);
qNew = q + dq;

state.q = qNew;
guidata(fig, state);

% 看门狗
servoWatchdog(fig, norm(e_pos), max(abs(qdot)./state.qdotMax));
end
```
要点：
- **行序**（5.5）：`e=[e_ori; e_pos]`、`vDes` 同序、`J` 天然 `[角;线]`，三者必须一致。
- `det(J*J')` 对 6×6 即 `det(J)^2`；`w` 的量纲依机器人尺度而定，**`manipW0` 自标定**为「正常扫掠时 w 的约 15%」。建议实现时把 `w` 打印/记录，跑一遍正常扫掠后再固化阈值。
- DLS 在**任务空间**加阻尼，关节单位差异由 `J` 各列自带 → `qdot` 各分量单位正确（旋转 rad/s、棱柱 m/s）。

### 8.6 限幅安全 `limitJointStep`（★）

```matlab
function [dq, qdot] = limitJointStep(qdot, q, state)
qdot = qdot(:);
% 1) 速度限幅：整体等比缩放以保持任务空间方向（优于逐分量硬切）
ratio = abs(qdot) ./ state.qdotMax;        % 旋转用 rad/s、棱柱用 m/s
m = max(ratio);
if m > 1, qdot = qdot / m; end
% 2) 逐分量硬上限（数值兜底）
qdot = max(min(qdot, state.qdotMax), -state.qdotMax);
% 3) 转成本拍增量
dq = qdot * state.controlPeriod;
% 4) 关节限位 clamp（lower/upper 可能含 ±Inf，min/max 仍正确）
qNew = q + dq;
qNew = min(max(qNew, state.jointInfo.lower(:)), state.jointInfo.upper(:));
dq = qNew - q;
end
```
（可选增强：限制相邻拍 `qdot` 变化量做加速度/抖动限幅，本期非必须。）

### 8.7 不可达退化与距离浮动
- **第一版（必做）**：`targetDistance` 固定 0.45。DLS 在目标不可达时**自动收敛到最近可达点**（残差持续存在，不会报错/急停）——这是相对 v3 离散 IK 的优势。
- 检测：若 `norm(e_pos)` 连续 `holdTimeoutSec` 内 > 某阈值（如 `2*positionTolerance`），UI 亮「低可达」灯（见 8.10）。
- **Phase 5 增强（选做）**：低速二级回退——当持续低可达时，按小步把 `targetDistance` 沿视线在 `[0.30,0.60]` 内外推/内缩（每 ~0.5s 调一次），找回可达；带内仍不可达则保持并标记。**不要每拍做 11 点搜索**。

### 8.8 接管：粗到位→精伺服 + 看门狗

`toggleFollow`：
```matlab
function toggleFollow(fig)
state = guidata(fig);
if state.mode == "servo"                    % 当前在跟随 → 停
    state.mode = "idle";
    state.followButton.String = "开始跟随";
    guidata(fig, state);
    setStatus(fig, "已暂停。预览继续刷新。", [0.10,0.10,0.10]);
else                                        % 开始跟随 → 先粗到位再切 servo
    state.followButton.String = "暂停跟随";
    guidata(fig, state);
    engageFollow(fig);
end
end
```

`engageFollow`（粗到位，复用 v3 离散 IK + 平滑轨迹）：
```matlab
function engageFollow(fig)
state = guidata(fig);
if state.busy, return, end
state.busy = true; state.mode = "engaging"; guidata(fig, state);
guard = onCleanup(@() clearBusy(fig));

state = guidata(fig);
solveResult = solveTargetWithDistanceFallback(state);   % 复用 v3([426])
if solveResult.reachable
    setStatus(fig, "接管：平滑到位中…", [0.05,0.35,0.12]);
    qNew = animateJointTrajectory(fig, state.q, solveResult.q);  % 复用 v3([668])
    state = guidata(fig); state.q = qNew; guidata(fig, state);
else
    % 不可达：直接进入伺服，让 DLS 尽量靠近
    setStatus(fig, "接管：目标当前低可达，进入伺服尽量靠近。", [0.70,0.40,0.05]);
end
state = guidata(fig);
state.mode = "servo";
state.watchStuckTicks = 0; state.watchPosErr = Inf;
guidata(fig, state);
end
```
> 注意：`animateJointTrajectory`（v3）内部自带 `pause`/`drawnow` 循环且会调 `processLatestFacePose(fig,false)`——v4 没有该函数。**改造**：把它内部的 `processLatestFacePose(fig,false)` 换成「仅刷新预览」的轻调用（`renderScene(fig,false)` 或 `ingestLatestPose`+`renderScene`），并确认它在 `busy` 已持有的前提下不再二次置锁。

`servoWatchdog`（仅失稳安全网，非常规大跳变处理）：
```matlab
function servoWatchdog(fig, posErr, qdotSat)
state = guidata(fig);
notImproving = posErr > 1.2*state.positionTolerance && posErr >= state.watchPosErr - 1e-4;
saturated    = qdotSat >= 0.999;
if notImproving && saturated
    state.watchStuckTicks = state.watchStuckTicks + 1;
else
    state.watchStuckTicks = max(0, state.watchStuckTicks - 1);
end
state.watchPosErr = posErr;
trigger = state.watchStuckTicks >= state.watchStuckLimit;
guidata(fig, state);
if trigger
    setStatus(fig, "伺服持续不收敛，执行一次粗到位恢复…", [0.70,0.40,0.05]);
    engageFollow(fig);                      % 触发一次重新粗到位
end
end
```

### 8.9 渲染复用 `renderScene`
```matlab
function renderScene(fig, forceRobot)
% 假定调用方已持有 state.busy
state = guidata(fig);
if ~isgraphics(state.ax), return, end
[T_target, targetPoint, faceNormal] = deal(state.lastTargetTform, state.lastTargetPoint, ...
    state.normalFilt/norm(state.normalFilt));
ensurePreviewStatics(fig);                  % 复用 v3：小人/相机标记建一次
updateHeadOrientation(fig, faceNormal);     % 复用 v3：头部仅换 hgtransform Matrix
refreshTargetOverlay(fig, T_target, targetPoint, faceNormal);  % 复用 v3：轻量箭头/坐标轴
if forceRobot
    redrawRobot(fig);                       % 复用 v3：show(robot,...,FastUpdate,true)，内部已保存/恢复相机
end
end
```
- **删除** v3 的 `previewBusy`/`updateTargetPreview` 那套独立预览锁——v4 用单一 `state.busy` 串行化整个 tick，更简单，不会再有预览重入。`resetHome` 改为：置 `busy`→设 home q→`renderScene(fig,true)`→释放。
- `redrawRobot` 内部已 `captureAxesCamera`/`restoreAxesCamera`，不要在 tick 外层再包一层。
- `closeDemo`（[行 1840](test_v3/demo_face_pose_screen_arm_live_follow_udp_avatar.m#L1840)）：保留停 timer、杀 Python、`deleteAvatarHandles`、`deleteGraphics(targetHandles)`、`closeUdpPoseReceiver` 的清理；移除对已删字段的引用。

### 8.10 UI 诊断面板 `createControls` / `updateServoUi`
在 v3 面板基础上：
- **保留**：固定点/距离说明文本、`poseText`、`imuText`、`statusText`、「开始/暂停跟随」按钮、Reset Home、Export、Close。
- **新增** `state.servoText`（诊断），每拍由 `updateServoUi` 刷新：
  - 位置误差 `mm`、姿态误差 `deg`（用 `poseErrors` 复用算）；
  - 最大关节速度占限速百分比 `max(|qdot|./qdotMax)*100%`；
  - 可操作度 `w` 与当前 `lambda`；
  - **实测控制频率** `state.measRateHz`；
  - 可达性灯：OK（绿）/ 低可达（红），及当前 `targetDistance`。
- **替换**：v3 的「Plan Once」（离散）按钮删除，或改为「Snap（单次粗到位）」调用 `engageFollow` 便于调试。
- 「开始跟随」语义改为「接管 → 粗到位 → 进入连续伺服」。

---

## 9. 参数总表（起始值，按需整定）

| 参数 | 起始值 | 单位 | 含义 / 整定提示 |
|---|---|---|---|
| `controlPeriod` | 0.04 | s | timer 周期（25Hz）。想更快可试 0.03；MATLAB timer <0.03 不可靠 |
| `renderEvery` | 2 | 拍 | 机械臂重绘降频（≈12Hz）。卡顿就调大到 3–4 |
| `servoKpOri` | 1.5 | 1/s | 姿态比例增益。大→快但易超调 |
| `servoKpPos` | 2.0 | 1/s | 位置比例增益 |
| `qdotMaxRev` | `deg2rad(70)` | rad/s | 旋转关节限速（防抖主保证） |
| `qdotMaxPris` | 0.15 | m/s | 棱柱关节（关节4）限速 |
| `lambdaMin` | 1e-3 | – | DLS 阻尼下限（数值安全） |
| `lambdaMax` | 0.05 | – | 奇异附近最大阻尼。抖/震就调大 |
| `manipW0` | 自标定 | – | 可操作度阈值≈正常扫掠 w 的 15%。**先记录 w 再固化** |
| `normalDeadbandRad` | `deg2rad(1.0)` | rad | 法向量死区，抑制微抖 |
| `normalSlerpAlphaMin/Max` | 0.15 / 0.55 | – | 自适应 slerp 系数 |
| `normalSlerpAngleFull` | `deg2rad(20)` | rad | 达此夹角用 alphaMax |
| `normalMaxRateRad` | `deg2rad(120)` | rad/s | 目标法向量最大角速度 |
| `pitchLpAlpha` | 0.20 | – | IMU 俯仰角 EMA 系数 |
| `holdTimeoutSec` | 0.5 | s | 丢帧保持/低可达判定时长 |
| `watchStuckLimit` | 25 | 拍 | ≈1s 持续失稳→粗到位恢复 |
| `viewDistance` | 0.45 | m | 标称屏-脸距离 |
| `distanceRange` | [0.30,0.60] | m | 允许距离带（Phase 5 浮动用） |
| `faceCenter` | [0.65,0,1.00] | m | 固定人脸参考点（沿用 v3） |

---

## 10. 验证与测试（不依赖真实相机）

用 `startFaceModule=false` 启动 v4，再用**合成 UDP 发包器**注入可控姿态。发包器示例（Python，发到 `127.0.0.1:5005`）：

```python
# tools/fake_pose_sender.py  —— 仅测试用，可放 test_v4/ 下
import socket, json, time, math, sys
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = ("127.0.0.1", 5005)
mode = sys.argv[1] if len(sys.argv) > 1 else "sweep"   # sweep|step|jitter|invalid|farfield
t0 = time.time()
while True:
    t = time.time() - t0
    a = 0.0
    if mode == "sweep":   a = math.radians(25) * math.sin(0.2 * t)      # 慢扫 ±25°
    elif mode == "step":  a = math.radians(35) if (int(t) % 6) < 3 else math.radians(-35)  # 阶跃
    elif mode == "jitter":a = math.radians(25)*math.sin(0.2*t) + math.radians(3)*math.sin(40*t)  # 慢扫+高频抖
    # 相机系法向量：人正对相机 nc=[0,0,-1]；绕相机Y(偏航)扫掠
    nc = [math.sin(a), 0.0, -math.cos(a)]
    pose = {"t": t, "valid": (mode != "invalid" or (int(t) % 4 != 0)),
            "normal": nc, "imu": {"accel": [0.0, 9.81, 0.0]}}  # 重力沿相机+Y → 俯仰0
    if mode == "farfield":
        # 让目标落在臂工作空间外，测试不可达退化（按需把 faceCenter/距离推远）
        pose["normal"] = [0.0, 0.0, -1.0]
    sock.sendto(json.dumps(pose).encode("utf-8"), addr)
    time.sleep(0.02)   # 50Hz
```

| 场景 | 模式 | 验收标准 |
|---|---|---|
| 慢速扫掠 | `sweep` | 末端姿态/位置误差始终小；跟随连续、无「顿挫」；视图不被重置 |
| 行序自检 | `sweep` | 纯偏航变化主要驱动姿态项；人为给纯平移目标时只动线性项（确认 `[角;线]` 没排反） |
| 快速阶跃 | `step` | 关节速度被限在 `qdotMax` 内、不超调、平滑追上；`measRateHz` 接近 25 |
| 抖动注入 | `jitter` | 末端几乎不抖（验证死区 + slerp 限速 + 关节限速 + 俯仰低通） |
| 越界目标 | `farfield` | 不报错/不急停；收敛到最近可达点；UI 亮「低可达」；（Phase 5）距离在带内浮动 |
| 丢帧/无效 | `invalid` | 保持上一目标不跳变；UI 标「信号暂失，保持」；恢复后继续 |
| 接管 | 任意 | 点「开始跟随」先平滑粗到位再进伺服，不从远处慢爬、不瞬跳 |
| 看门狗 | `farfield` 持续 | 持续不收敛约 1s 后触发一次粗到位恢复，不死锁 |

**客观记录**（可临时 `assignin`/打印到 base）：每拍记录位置误差、姿态误差、`max(|qdot|./qdotMax)`、可操作度 `w`、`measRateHz`、是否出现 `NaN/Inf`。跑完确认：速度有界、误差收敛、频率达标、无 NaN。

---

## 11. 实施里程碑（建议顺序）

1. **脚手架**：复制 v3 → v4，去掉离散触发（`planAndMove`/`targetChangeNeedsMove`/`processLatestFacePose` 触发段），保留加载/UI/渲染/坐标变换/UDP。先确保能开窗、显示、收 UDP、Reset Home。提交。
2. **差分 IK 内核**：实现 `servoOneStep` + `limitJointStep`，用 `sweep` 离线验证收敛与**行序**（场景「慢扫」「行序自检」）。
3. **主循环**：把 `servoOneStep` 接入单 timer + 计数器降频渲染，跑通连续跟随（`sweep`/`jitter`）。
4. **滤波层**：加 slerp 限速 + 死区 + 俯仰低通，调 `jitter`。
5. **安全/退化**：限速限位、奇异阻尼、不可达退化 + 看门狗 + UI 灯（`step`/`farfield`/`invalid`）。
6. **接管体验**：`engageFollow` 粗到位→精伺服；改造 `animateJointTrajectory` 的内部回调。
7. **真机联调**：接真实相机，整定 `Kp`/`lambda*`/`qdotMax`/`manipW0`。
8. **收尾**：见第 13 节文档/记忆更新。

每个里程碑结束跑一次 `checkcode` 确保无警告，并做对应场景烟测。

---

## 12. 风险与回退

| 风险 | 缓解 |
|---|---|
| MATLAB timer 在 <0.03s 不准、`drawnow` 抢线程 | 控制周期 0.04s 起步；渲染降频；以 `measRateHz` 实测为准，别假设 25/50Hz |
| 棱柱关节单位混用导致限速/DLS 错误 | `qdotMax` 按 `jointInfo.types` 分类型组装；单独验证关节4（给纯沿屏法向位移目标看它是否平移） |
| 雅可比行序排反（角/线） | 「行序自检」场景必跑：纯平移只动线性、纯转动只动角度 |
| `Kp`/`lambda` 整定不当→震荡或太慢 | 先保守（小 Kp、稍大 lambda）求稳，再加快；参数集中在 setup 顶部便于调 |
| 伺服失稳/发散 | 看门狗（8.8）：持续不收敛 + 速度触顶 → 自动粗到位恢复 |
| 重入/竞态（v3 曾踩） | 单 `state.busy` 串行锁 + `onCleanup` 释放；timer `BusyMode="drop"`；删除 v3 独立预览锁 |
| 改坏 v3 | **v4 完全独立**；v3 不动，随时可切回稳定演示 |

**最终回退保证**：`test_v3/` 原封不动，任何时刻都能切回稳定版。

---

## 13. 完成定义（DoD）与收尾清单

**功能 DoD**：
- [ ] `test_v4/demo_face_pose_screen_arm_realtime_servo_udp_avatar.m` 可独立运行（large/normal/wide；起/不起相机两种）。
- [ ] 第 10 节全部场景通过；`checkcode` 无警告。
- [ ] 连续跟随平滑、无「顿挫」；关节速度恒在限内；丢帧保持、越界退化、接管粗到位、看门狗均按预期。
- [ ] 桌子/机械臂光影正常（核对 setup 的「先 show（hold off）再 hold on」顺序，见第 7 节末）。
- [ ] `test_v3/` 未被修改。

**文档/记忆收尾**（实现完成后执行）：
- [ ] [AGENT.md](AGENT.md)：新增一节，记录 v4 的架构（单 timer + DLS 伺服 + 分层滤波 + 限幅 + 粗到位接管）、与 v3 的分工、关键非显然点（雅可比行序、棱柱关节限速、单 busy 锁取代预览锁、`manipW0` 自标定）。
- [ ] [README.md](README.md)：补 test_v4 的用途与运行方式（「v4：实时视觉伺服式跟随，验证连续输入下的动态响应」），与 v3「离散触发，验证目标位姿/可达性/距离 fallback/UI 诊断」并列。
- [ ] [关键参数.md](关键参数.md)：补 v4 控制参数（`controlPeriod`/`Kp`/`qdotMax`/`lambda`/滤波系数）的整定结果。
- [ ] [后续开发方向.md](后续开发方向.md)：把「真正实时跟随」标记为已实现，指向 test_v4。
- [ ] 合成发包器若保留，放 `test_v4/tools/` 并在 README 注明仅测试用。

---

## 附录 A. MATLAB API 与公式速查

**雅可比（基座系，行序 [角(3); 线(3)]）**
```matlab
J = geometricJacobian(robot, q, "screen_center");   % 6×6
```

**6 维位姿误差（世界系，与 J 同序）**
```matlab
T_cur = getTransform(robot, q, "screen_center");
e_pos = T_target(1:3,4) - T_cur(1:3,4);                 % 线
Rerr  = T_target(1:3,1:3) * T_cur(1:3,1:3).';           % 世界系旋转误差
ax    = rotm2axang(Rerr);                               % [axis(1:3), theta]
e_ori = ax(1:3).' * ax(4);                              % 角(旋转矢量)
e     = [e_ori; e_pos];                                 % 6×1
```

**阻尼最小二乘（resolved-rate）**
```matlab
vDes    = [Kp_ori*e_ori; Kp_pos*e_pos];                 % 期望空间速度
qdot    = J.' * ((J*J.' + lambda^2*eye(6)) \ vDes);     % 关节速度（混合单位）
dq      = qdot * dt;                                    % 本拍增量
qNew    = q + dq;                                       % 限速/限位见 8.6
```

**可操作度与自适应阻尼**
```matlab
w = sqrt(max(0, det(J*J.')));                           % 越小越接近奇异
% w<w0 时 lambda^2 = lambdaMax^2*(1-w/w0)^2 + lambdaMin^2; 否则 lambdaMin^2
```

**球面线性插值（单位向量）** —— 见 8.3 `slerpVector`。

> 关键提醒（再说一遍）：**误差 `e`、增益 `vDes`、雅可比 `J` 三者的「角在前、线在后」顺序必须一致**，这是本任务最容易、也最致命的 bug 源。
