function demo_face_pose_screen_arm_realtime_servo_udp_avatar(workspaceMode, startFaceModule)
%DEMO_FACE_POSE_SCREEN_ARM_REALTIME_SERVO_UDP_AVATAR Continuous visual-servo face-follow demo.
%
% Usage:
%   addpath('test_v4', '-begin')
%   demo_face_pose_screen_arm_realtime_servo_udp_avatar
%   demo_face_pose_screen_arm_realtime_servo_udp_avatar("normal")
%   demo_face_pose_screen_arm_realtime_servo_udp_avatar("normal", false)  % no camera process

if nargin < 1 || strlength(string(workspaceMode)) == 0
    workspaceMode = "large";
else
    workspaceMode = lower(string(workspaceMode));
end

if nargin < 2
    startFaceModule = true;
end
startFaceModule = logical(startFaceModule);

projectRoot = fileparts(fileparts(mfilename("fullpath")));
urdfPath = fullfile(projectRoot, "screen_arm", "generated", "urdf", ...
    "face_screen_support_arm_depth_camera.urdf");

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

ik = inverseKinematics("RigidBodyTree", robot);
ik.SolverParameters.MaxIterations = 1200;
ik.SolverParameters.MaxTime = 1.0;

ikLoose = inverseKinematics("RigidBodyTree", robot);
ikLoose.SolverParameters.MaxIterations = 800;
ikLoose.SolverParameters.MaxTime = 0.6;
ikLoose.SolverParameters.EnforceJointLimits = false;

layout = makeWindowLayout();

fig = figure( ...
    "Name", "Realtime Face Pose Screen Arm Servo UDP v4 Avatar", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "MenuBar", "none", ...
    "ToolBar", "figure", ...
    "Visible", "on", ...
    "Units", "pixels", ...
    "Position", layout.simPosition, ...
    "CloseRequestFcn", @(src, ~) closeDemo(src));

ax = axes( ...
    "Parent", fig, ...
    "Units", "normalized", ...
    "Position", [0.045, 0.060, 0.660, 0.820]);
view(ax, 135, 25)
camproj(ax, "perspective")
rotate3d(fig, "on")

panel = uipanel( ...
    "Parent", fig, ...
    "Title", "Realtime Servo v4", ...
    "Units", "normalized", ...
    "Position", [0.735, 0.050, 0.240, 0.750], ...
    "BackgroundColor", "w");

udpIp = "127.0.0.1";
udpPort = 5005;
udpPoseReceiver = createUdpPoseReceiver(udpIp, udpPort);

pythonLogFile = fullfile(tempdir, "screen_arm_face_pose_realtime_servo_v4.log");
if isfile(pythonLogFile)
    delete(pythonLogFile);
end

state = struct;
state.projectRoot = projectRoot;
state.faceModuleDir = fullfile(projectRoot, "face_pose_module_v2");
state.faceModuleLabel = "face_pose_module_v2 / 3DDFA_V2";
state.robot = robot;
state.ik = ik;
state.ikLoose = ikLoose;
state.endEffector = "screen_center";
state.weights = [0.7, 0.7, 0.7, 1, 1, 1];
state.jointInfo = movingJointInfo(robot);
state.workspaceMode = workspaceMode;
state.ax = ax;
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
state.faceCenter = [0.65, 0.00, 1.00];
state.viewDistance = 0.45;
state.targetDistance = state.viewDistance;
state.distanceRange = [0.30, 0.60];
state.faceNormalArrowLength = 0.18;
state.targetAxisArrowLength = 0.12;
state.steveHeadSize = 0.22;
state.positionTolerance = 0.025;
state.normalTolerance = deg2rad(8);
state.faceNormalWorld = [-1, 0, 0];
state.normalFilt = [-1, 0, 0];
state.cameraPositionWorld = depthCameraCenterWorld();
state.cameraNominalRotationWorldFromCamera = cameraNominalRotationWorldFromCamera();
state.latestPose = [];
state.lastPoseTimestamp = -Inf;
state.clock = tic;
state.mode = "idle";
state.busy = false;
state.controlPeriod = 0.04;
state.renderEvery = 2;
state.renderCounter = 0;
state.pitchFilt = 0;
state.pitchFiltInit = false;
state.normalDeadbandRad = deg2rad(1.0);
state.normalSlerpAlphaMin = 0.15;
state.normalSlerpAlphaMax = 0.55;
state.normalSlerpAngleFull = deg2rad(20);
state.normalMaxRateRad = deg2rad(120);
state.pitchLpAlpha = 0.20;
state.servoKpOri = 1.5;
state.servoKpPos = 2.0;
state.lambdaMin = 1e-3;
state.lambdaMax = 0.05;
state.manipW0 = [];
state.qdotMaxRev = deg2rad(70);
state.qdotMaxPris = 0.15;
state.qdotMax = jointVelocityLimits(state.jointInfo, state.qdotMaxRev, state.qdotMaxPris);
state.lastValidPoseTime = -Inf;
state.holdTimeoutSec = 0.5;
state.watchPosErr = Inf;
state.watchStuckTicks = 0;
state.watchStuckLimit = 25;
state.watchdogRecoveryPending = false;
state.lastTickTime = -Inf;
state.measRateHz = 0;
state.servoLastPosErr = NaN;
state.servoLastOriErr = NaN;
state.servoLastQdotSat = 0;
state.servoLastManipW = NaN;
state.servoLastLambda = NaN;
state.servoLastHolding = false;
state.targetReachable = true;
state.udpIp = udpIp;
state.udpPort = udpPort;
state.udpPoseReceiver = udpPoseReceiver;
state.pythonLogFile = pythonLogFile;
state.startFaceModule = startFaceModule;
state.pythonProcess = [];
state.pythonExitReported = false;
state.updateTimer = [];
% Persistent avatar/camera graphics are built once; only the head transform
% and the lightweight target overlay change frame-to-frame.
state.avatar = emptyAvatarHandles();
state.targetHandles = gobjects(0);
state.targetOverlayTag = "v4_target_overlay";
state.lastTargetTform = eye(4);
state.lastTargetPoint = zeros(1, 3);
state.poseText = gobjects(1);
state.imuText = gobjects(1);
state.servoText = gobjects(1);
state.statusText = gobjects(1);
state.followButton = gobjects(1);
state.windowLayout = layout;

guidata(fig, state);
createControls(fig, panel);
% First show() runs with hold off so it sets up scene lighting (the desk/arm
% shading). Then hold on for the rest of the session so later show(...,
% "FastUpdate", true) calls keep both the light and the persistent avatar.
redrawRobot(fig);
hold(ax, "on")
state = guidata(fig);
[targetTform, targetPoint] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.normalFilt);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;
guidata(fig, state);
renderScene(fig, false);
assignTargetToBase(fig);

if startFaceModule
    state = guidata(fig);
    state.pythonProcess = startPythonFaceModule(state);
    guidata(fig, state);
    setStatus(fig, sprintf("Face module v2 launched.\nUDP: %s:%d\nLog: %s", udpIp, udpPort, pythonLogFile), [0.10, 0.10, 0.10]);
else
    setStatus(fig, "Camera process disabled for this run.\nWaiting is skipped; preview uses default normal.", [0.10, 0.10, 0.10]);
end

state = guidata(fig);
state.updateTimer = timer( ...
    "ExecutionMode", "fixedSpacing", ...
    "Period", state.controlPeriod, ...
    "BusyMode", "drop", ...
    "TimerFcn", @(~, ~) controlTick(fig));
guidata(fig, state);
start(state.updateTimer);

assignin("base", "robot", robot);
assignin("base", "q", state.q);
assignin("base", "realtimeServoFigure", fig);
fprintf("\nRealtime face-pose servo demo started.\n");
fprintf("Face module:       %s\n", state.faceModuleLabel);
fprintf("Fixed head/face reference: [%.3f %.3f %.3f] m\n", state.faceCenter);
fprintf("Camera center:     [%.3f %.3f %.3f] m\n", state.cameraPositionWorld);
fprintf("UDP input:         %s:%d\n", state.udpIp, state.udpPort);
fprintf("Control period:    %.3f s\n", state.controlPeriod);
fprintf("Variables exported: robot, q, realtimeServoFigure\n\n");
end

function createControls(fig, panel)
state = guidata(fig);

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", sprintf("固定头部/人脸点: [%.2f, %.2f, %.2f] m", state.faceCenter), ...
    "Units", "normalized", ...
    "Position", [0.06, 0.935, 0.88, 0.040], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "目标距离: 伺服标称 0.45 m，接管粗到位可用距离 fallback", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.895, 0.88, 0.035], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", sprintf("控制: %.0f Hz timer，机械臂每 %d 拍重绘", ...
        1 / state.controlPeriod, state.renderEvery), ...
    "Units", "normalized", ...
    "Position", [0.06, 0.855, 0.88, 0.035], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.poseText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "No live face pose yet.", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.680, 0.88, 0.150], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.imuText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "IMU tilt: waiting.", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.545, 0.88, 0.115], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.servoText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "Servo diagnostics: idle.", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.375, 0.88, 0.150], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.statusText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.235, 0.88, 0.120], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.followButton = uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "开始跟随", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.165, 0.88, 0.055], ...
    "Callback", @(~, ~) toggleFollow(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Snap", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.095, 0.42, 0.050], ...
    "Callback", @(~, ~) engageFollow(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Reset Home", ...
    "Units", "normalized", ...
    "Position", [0.52, 0.095, 0.42, 0.050], ...
    "Callback", @(~, ~) resetHome(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Export", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.025, 0.42, 0.050], ...
    "Callback", @(~, ~) exportState(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Close", ...
    "Units", "normalized", ...
    "Position", [0.52, 0.025, 0.42, 0.050], ...
    "Callback", @(~, ~) closeDemo(fig));

guidata(fig, state);
end

function controlTick(fig)
if ~isvalid(fig)
    return
end

state = guidata(fig);
if state.startFaceModule && pythonProcessExited(fig)
    return
end
if state.watchdogRecoveryPending && ~state.busy
    state.watchdogRecoveryPending = false;
    guidata(fig, state);
    engageFollow(fig);
    return
end
if state.busy
    return
end

state.busy = true;
guidata(fig, state);
guard = onCleanup(@() clearBusy(fig));

state = guidata(fig);
nowT = toc(state.clock);
if isfinite(state.lastTickTime)
    dtMeas = nowT - state.lastTickTime;
    if dtMeas > 0
        if state.measRateHz <= 0
            state.measRateHz = 1 / dtMeas;
        else
            state.measRateHz = 0.9 * state.measRateHz + 0.1 * (1 / dtMeas);
        end
    end
end
state.lastTickTime = nowT;
guidata(fig, state);

[~, holding] = ingestLatestPose(fig);

state = guidata(fig);
state.targetDistance = state.viewDistance;
[targetTform, targetPoint] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.normalFilt);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;
guidata(fig, state);

state = guidata(fig);
if state.mode == "servo"
    servoOneStep(fig, targetTform);
end

state = guidata(fig);
state.renderCounter = state.renderCounter + 1;
forceRobot = state.renderCounter >= state.renderEvery;
if forceRobot
    state.renderCounter = 0;
end
guidata(fig, state);

renderScene(fig, forceRobot);
updateServoUi(fig, holding);
assignTargetToBase(fig);
end

function [hasValid, holding] = ingestLatestPose(fig)
hasValid = false;
holding = false;
if ~isvalid(fig)
    return
end

state = guidata(fig);
nowT = toc(state.clock);
pose = readLatestUdpPose(state.udpPoseReceiver);
if isempty(pose)
    holding = isfinite(state.lastValidPoseTime) && nowT - state.lastValidPoseTime > state.holdTimeoutSec;
    state.servoLastHolding = holding;
    guidata(fig, state);
    if holding
        updatePoseUi(fig, false, "信号暂失，保持上一目标。");
    end
    return
end

if ~isfield(pose, "t") || double(pose.t) <= state.lastPoseTimestamp
    holding = isfinite(state.lastValidPoseTime) && nowT - state.lastValidPoseTime > state.holdTimeoutSec;
    state.servoLastHolding = holding;
    guidata(fig, state);
    return
end

state.lastPoseTimestamp = double(pose.t);
state.latestPose = pose;

if ~isfield(pose, "valid") || ~logical(pose.valid)
    holding = isfinite(state.lastValidPoseTime) && nowT - state.lastValidPoseTime > state.holdTimeoutSec;
    state.servoLastHolding = holding;
    guidata(fig, state);
    updatePoseUi(fig, false, "Face pose invalid. Holding previous target.");
    return
end

normalCamera = numericVectorField(pose, "normal");
if isempty(normalCamera)
    holding = isfinite(state.lastValidPoseTime) && nowT - state.lastValidPoseTime > state.holdTimeoutSec;
    state.servoLastHolding = holding;
    guidata(fig, state);
    updatePoseUi(fig, false, "Face normal missing. Holding previous target.");
    return
end

[~, usedImu, accelCamera, pitchAngle] = cameraRotationFromImu(pose, state);
if usedImu
    [state.pitchFilt, state.pitchFiltInit] = lowpassPitch( ...
        state.pitchFilt, state.pitchFiltInit, pitchAngle, state.pitchLpAlpha);
    cameraRotation = state.cameraNominalRotationWorldFromCamera * rotxLocal(state.pitchFilt);
else
    state.pitchFiltInit = false;
    state.pitchFilt = 0;
    cameraRotation = state.cameraNominalRotationWorldFromCamera;
end

normalWorld = cameraRotation * normalCamera(:);
if norm(normalWorld) < 1e-9 || any(~isfinite(normalWorld))
    holding = isfinite(state.lastValidPoseTime) && nowT - state.lastValidPoseTime > state.holdTimeoutSec;
    state.servoLastHolding = holding;
    guidata(fig, state);
    updatePoseUi(fig, false, "Face normal has near-zero length. Holding previous target.");
    return
end

normalWorld = normalWorld / norm(normalWorld);
state.faceNormalWorld = normalWorld.';
state.normalFilt = filterTargetNormal(state.normalFilt, state.faceNormalWorld, state);
state.lastValidPoseTime = nowT;
state.servoLastHolding = false;
forwardWorld = cameraRotation * [0; 0; 1];
imuInfo = poseInfoText(usedImu, accelCamera, state.pitchFilt, forwardWorld);
guidata(fig, state);

updatePoseUi(fig, true, imuInfo);
hasValid = true;
end

function textValue = poseInfoText(usedImu, accelCamera, pitchAngle, forwardWorld)
if usedImu
    textValue = sprintf( ...
        "v2 fields used: t, valid, normal, imu.accel\nIMU pitch filtered: %.1f deg\naccel: [%+.2f %+.2f %+.2f]\nyaw/roll fixed, cam +Z: [%+.2f %+.2f %+.2f]", ...
        rad2deg(pitchAngle), ...
        accelCamera(1), accelCamera(2), accelCamera(3), ...
        forwardWorld(1), forwardWorld(2), forwardWorld(3));
else
    textValue = sprintf( ...
        "v2 fields used: t, valid, normal\nIMU pitch-only: nominal 0.0 deg\nyaw/roll fixed, cam +Z: [%+.2f %+.2f %+.2f]", ...
        forwardWorld(1), forwardWorld(2), forwardWorld(3));
end
end

function nf = filterTargetNormal(nPrev, nNew, state)
nPrev = nPrev(:);
nNew = nNew(:);
if norm(nPrev) < 1e-9 || any(~isfinite(nPrev))
    nPrev = [-1; 0; 0];
end
if norm(nNew) < 1e-9 || any(~isfinite(nNew))
    nf = (nPrev / norm(nPrev)).';
    return
end
nPrev = nPrev / norm(nPrev);
nNew = nNew / norm(nNew);
angle = acos(max(-1, min(1, dot(nPrev, nNew))));
if angle < state.normalDeadbandRad
    nf = nPrev.';
    return
end

alpha = state.normalSlerpAlphaMin + ...
    (state.normalSlerpAlphaMax - state.normalSlerpAlphaMin) * ...
    min(1, angle / state.normalSlerpAngleFull);
maxStep = state.normalMaxRateRad * state.controlPeriod;
if alpha * angle > maxStep
    alpha = maxStep / angle;
end
nf = slerpVector(nPrev, nNew, alpha);
end

function v = slerpVector(v0, v1, alpha)
v0 = v0(:) / norm(v0);
v1 = v1(:) / norm(v1);
d = max(-1, min(1, dot(v0, v1)));
theta = acos(d);
if theta < 1e-6
    v = v1.';
    return
end
if theta > pi - 1e-6
    mixed = (1 - alpha) * v0 + alpha * v1;
    if norm(mixed) < 1e-9
        mixed = v1;
    end
    v = (mixed / norm(mixed)).';
    return
end
v = (sin((1 - alpha) * theta) * v0 + sin(alpha * theta) * v1) / sin(theta);
v = (v / norm(v)).';
end

function [pitchFilt, pitchFiltInit] = lowpassPitch(pitchFilt, pitchFiltInit, pitchAngle, alpha)
if ~pitchFiltInit
    pitchFilt = pitchAngle;
    pitchFiltInit = true;
else
    pitchFilt = (1 - alpha) * pitchFilt + alpha * pitchAngle;
end
end

function servoOneStep(fig, targetTform)
state = guidata(fig);
q = state.q(:);
endEffector = state.endEffector;

currentTform = getTransform(state.robot, q, endEffector);
positionErrorVec = targetTform(1:3, 4) - currentTform(1:3, 4);
rotationError = targetTform(1:3, 1:3) * currentTform(1:3, 1:3).';
axisAngle = rotm2axang(rotationError);
orientationErrorVec = axisAngle(1:3).' * axisAngle(4);
desiredVelocity = [state.servoKpOri * orientationErrorVec; state.servoKpPos * positionErrorVec];

J = geometricJacobian(state.robot, q, endEffector);
manipW = sqrt(max(0, det(J * J.')));
if ~isfinite(manipW)
    manipW = 0;
end
if isempty(state.manipW0) || ~isfinite(state.manipW0)
    state.manipW0 = max(manipW * 0.15, 1e-6);
end
if manipW < state.manipW0
    lambda2 = state.lambdaMax^2 * (1 - manipW / state.manipW0)^2 + state.lambdaMin^2;
else
    lambda2 = state.lambdaMin^2;
end
qdot = J.' * ((J * J.' + lambda2 * eye(6)) \ desiredVelocity);
if any(~isfinite(qdot))
    state.servoLastPosErr = norm(positionErrorVec);
    state.servoLastOriErr = norm(orientationErrorVec);
    state.servoLastQdotSat = NaN;
    state.servoLastManipW = manipW;
    state.servoLastLambda = sqrt(lambda2);
    state.targetReachable = false;
    guidata(fig, state);
    setStatus(fig, "伺服求解出现 NaN/Inf，本拍保持当前关节。", [0.70, 0.05, 0.05]);
    return
end

[dq, qdotLimited] = limitJointStep(qdot, q, state);
qNew = q + dq;
qdotSat = max(abs(qdotLimited) ./ state.qdotMax);
posErr = norm(positionErrorVec);
oriErr = norm(orientationErrorVec);

state.q = qNew;
state.servoLastPosErr = posErr;
state.servoLastOriErr = oriErr;
state.servoLastQdotSat = qdotSat;
state.servoLastManipW = manipW;
state.servoLastLambda = sqrt(lambda2);
state.targetReachable = posErr <= 2 * state.positionTolerance || qdotSat < 0.999;
guidata(fig, state);

servoWatchdog(fig, posErr, qdotSat);
end

function [dq, qdot] = limitJointStep(qdot, q, state)
qdot = qdot(:);
qdotMax = state.qdotMax(:);
ratio = abs(qdot) ./ qdotMax;
limitRatio = max(ratio);
if limitRatio > 1
    qdot = qdot / limitRatio;
end
qdot = max(min(qdot, qdotMax), -qdotMax);
dq = qdot * state.controlPeriod;
qNew = q(:) + dq;
qNew = min(max(qNew, state.jointInfo.lower(:)), state.jointInfo.upper(:));
dq = qNew - q(:);
end

function toggleFollow(fig)
state = guidata(fig);
if state.busy
    setStatus(fig, "控制循环正在更新，稍后再切换跟随。", [0.70, 0.40, 0.05]);
    return
end
if state.mode == "servo" || state.mode == "engaging"
    state.mode = "idle";
    state.watchdogRecoveryPending = false;
    if isgraphics(state.followButton)
        state.followButton.String = "开始跟随";
    end
    guidata(fig, state);
    setStatus(fig, "已暂停。预览继续刷新，目标保持最新滤波法向量。", [0.10, 0.10, 0.10]);
else
    if isgraphics(state.followButton)
        state.followButton.String = "暂停跟随";
    end
    guidata(fig, state);
    engageFollow(fig);
end
end

function engageFollow(fig)
if ~isvalid(fig)
    return
end

state = guidata(fig);
if state.busy
    return
end

state.busy = true;
state.mode = "engaging";
if isgraphics(state.followButton)
    state.followButton.String = "暂停跟随";
end
guidata(fig, state);
guard = onCleanup(@() clearBusy(fig));

ingestLatestPose(fig);
state = guidata(fig);
state.faceNormalWorld = state.normalFilt;
state.targetDistance = state.viewDistance;
guidata(fig, state);

solveResult = solveTargetWithDistanceFallback(state);
fprintf("Engage IK status: %s\n", string(solveResult.status));
fprintf("Engage distance: %.3f m%s\n", ...
    solveResult.distance, ternaryText(solveResult.usedFallback, " (fallback)", ""));

if solveResult.reachable
    state = guidata(fig);
    state.targetDistance = solveResult.distance;
    state.targetReachable = true;
    [targetTform, targetPoint] = buildTargetTformFromNormal( ...
        state.faceCenter, state.targetDistance, state.normalFilt);
    state.lastTargetTform = targetTform;
    state.lastTargetPoint = targetPoint;
    guidata(fig, state);
    setStatus(fig, "接管：平滑粗到位中...", [0.05, 0.35, 0.12]);
    qNew = animateJointTrajectory(fig, state.q, solveResult.q);
    state = guidata(fig);
    state.q = qNew;
    guidata(fig, state);
else
    state = guidata(fig);
    state.targetReachable = false;
    guidata(fig, state);
    setStatus(fig, "接管：目标当前低可达，进入伺服尽量靠近。", [0.70, 0.40, 0.05]);
end

state = guidata(fig);
state.targetDistance = state.viewDistance;
[targetTform, targetPoint] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.normalFilt);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;
state.mode = "servo";
state.watchPosErr = Inf;
state.watchStuckTicks = 0;
state.watchdogRecoveryPending = false;
guidata(fig, state);

renderScene(fig, true);
updateServoUi(fig, false);
assignin("base", "q", state.q);
assignTargetToBase(fig);
if solveResult.reachable
    setStatus(fig, "接管完成，已进入连续视觉伺服。", [0.05, 0.35, 0.12]);
end
end

function servoWatchdog(fig, posErr, qdotSat)
state = guidata(fig);
notImproving = posErr > 1.2 * state.positionTolerance && posErr >= state.watchPosErr - 1e-4;
saturated = qdotSat >= 0.999;
if notImproving && saturated
    state.watchStuckTicks = state.watchStuckTicks + 1;
else
    state.watchStuckTicks = max(0, state.watchStuckTicks - 1);
end
state.watchPosErr = posErr;
if state.watchStuckTicks >= state.watchStuckLimit
    state.watchStuckTicks = 0;
    state.watchdogRecoveryPending = true;
    setStatus(fig, "伺服持续不收敛，将执行一次粗到位恢复。", [0.70, 0.40, 0.05]);
end
guidata(fig, state);
end

function clearBusy(fig)
if ~isvalid(fig)
    return
end
state = guidata(fig);
state.busy = false;
guidata(fig, state);
end

function result = solveTargetWithDistanceFallback(state)
distances = candidateDistances(state.distanceRange, state.viewDistance);
result = solveSingleDistance(state, distances(1));
result.usedFallback = false;

if result.reachable
    return
end

bestFallback = result;
hasReachableFallback = false;

for i = 2:numel(distances)
    candidate = solveSingleDistance(state, distances(i));
    candidate.usedFallback = true;

    if candidate.reachable
        if ~hasReachableFallback || candidate.score < bestFallback.score
            bestFallback = candidate;
            hasReachableFallback = true;
        end
    elseif ~hasReachableFallback && candidate.score < bestFallback.score
        bestFallback = candidate;
    end
end

result = bestFallback;
end

function result = solveSingleDistance(state, distance)
targetTform = buildTargetTformFromNormal(state.faceCenter, distance, state.faceNormalWorld);
[qSolution, solutionInfo] = state.ik( ...
    state.endEffector, targetTform, state.weights, state.q);
[positionError, fullOrientationError, normalError] = poseErrors( ...
    state.robot, qSolution, state.endEffector, targetTform);

actualTform = getTransform(state.robot, qSolution, state.endEffector);
actualDistance = norm(actualTform(1:3, 4).' - state.faceCenter);
distanceInBand = ...
    actualDistance >= state.distanceRange(1) - 1e-6 && ...
    actualDistance <= state.distanceRange(2) + 1e-6;

reachable = ...
    positionError <= state.positionTolerance && ...
    normalError <= state.normalTolerance && ...
    distanceInBand;

rangeHalfWidth = max(eps, diff(state.distanceRange) / 2);
distancePenalty = abs(distance - state.viewDistance) / rangeHalfWidth;
score = ...
    positionError / state.positionTolerance + ...
    normalError / state.normalTolerance + ...
    0.25 * distancePenalty;

result = struct( ...
    "q", qSolution, ...
    "targetTform", targetTform, ...
    "distance", distance, ...
    "actualDistance", actualDistance, ...
    "positionError", positionError, ...
    "fullOrientationError", fullOrientationError, ...
    "normalError", normalError, ...
    "distanceInBand", distanceInBand, ...
    "reachable", reachable, ...
    "usedFallback", false, ...
    "score", score, ...
    "status", string(solutionInfo.Status));
end

function distances = candidateDistances(distanceRange, nominalDistance)
fallbackDistances = linspace(distanceRange(1), distanceRange(2), 11);
fallbackDistances(abs(fallbackDistances - nominalDistance) < 1e-9) = [];
[~, order] = sort(abs(fallbackDistances - nominalDistance));
distances = [nominalDistance, fallbackDistances(order)];
end

function textValue = ternaryText(condition, trueText, falseText)
if condition
    textValue = trueText;
else
    textValue = falseText;
end
end

function q = animateJointTrajectory(fig, qStart, qGoal)
maxDelta = max(abs(qGoal(:) - qStart(:)));
frameCount = max(12, min(36, ceil(14 + 10 * maxDelta)));
secondsPerRobotFrame = 0.040;
previewPauseSeconds = 0.008;
q = qStart;

for frameIndex = 1:frameCount
    if ~isvalid(fig)
        return
    end

    t = frameIndex / frameCount;
    s = smoothStep(t);
    q = qStart + (qGoal - qStart) * s;

    state = guidata(fig);
    state.q = q;
    guidata(fig, state);
    redrawRobot(fig);
    drawnow limitrate

    frameTimer = tic;
    while toc(frameTimer) < secondsPerRobotFrame
        if ~isvalid(fig)
            return
        end
        ingestLatestPose(fig);
        state = guidata(fig);
        [targetTform, targetPoint] = buildTargetTformFromNormal( ...
            state.faceCenter, state.targetDistance, state.normalFilt);
        state.lastTargetTform = targetTform;
        state.lastTargetPoint = targetPoint;
        guidata(fig, state);
        renderScene(fig, false);
        drawnow limitrate
        pause(previewPauseSeconds)
    end
end

q = qGoal;
end

function s = smoothStep(t)
% Cubic interpolation with zero velocity at both ends.
s = 3 * t^2 - 2 * t^3;
end

function redrawRobot(fig)
state = guidata(fig);
cameraState = captureAxesCamera(state.ax);

show(state.robot, state.q, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "Parent", state.ax, ...
    "PreservePlot", false, ...
    "FastUpdate", true);

setupAxes(state.ax, state.workspaceMode);
restoreAxesCamera(state.ax, cameraState);
title(state.ax, "Realtime face-pose visual servo")
drawnow limitrate
end

function renderScene(fig, forceRobot)
if nargin < 2
    forceRobot = false;
end
state = guidata(fig);
if ~isgraphics(state.ax)
    return
end

if forceRobot
    redrawRobot(fig);
    state = guidata(fig);
end

cameraState = captureAxesCamera(state.ax);
targetTform = state.lastTargetTform;
targetPoint = state.lastTargetPoint;
faceNormal = state.normalFilt(:);
if norm(faceNormal) < 1e-9 || any(~isfinite(faceNormal))
    faceNormal = [-1; 0; 0];
end
faceNormal = (faceNormal / norm(faceNormal)).';

% The avatar and camera marker are built once; per update we only re-pose the
% head (an hgtransform Matrix swap) and rebuild the lightweight target overlay.
ensurePreviewStatics(fig);
updateHeadOrientation(fig, faceNormal);
refreshTargetOverlay(fig, targetTform, targetPoint, faceNormal);

restoreAxesCamera(state.ax, cameraState);
drawnow limitrate
end

function ensurePreviewStatics(fig)
% Build the parts that never change frame-to-frame: stool, body, head mesh,
% camera marker, head-point marker and its label. Rebuild only if missing.
state = guidata(fig);
if avatarStaticsValid(state.avatar)
    return
end

deleteAvatarHandles(state.avatar);
ax = state.ax;
if ~isgraphics(ax)
    return
end

headCenter = state.faceCenter;
headSize = state.steveHeadSize;

avatar = emptyAvatarHandles();
avatar.stool = drawSteveStool(ax, headCenter, headSize);
avatar.body = drawSteveBody(ax, headCenter, headSize);
avatar.head = drawSteveHead(ax, headCenter, state.normalFilt, headSize);
avatar.cameraMarker = plot3(ax, ...
    state.cameraPositionWorld(1), state.cameraPositionWorld(2), state.cameraPositionWorld(3), ...
    "^", "MarkerSize", 7, "MarkerFaceColor", [0.20, 0.20, 0.20], "MarkerEdgeColor", "k");
avatar.headMarker = plot3(ax, headCenter(1), headCenter(2), headCenter(3), ...
    "o", "MarkerSize", 9, "MarkerFaceColor", [0.90, 0.10, 0.10], "MarkerEdgeColor", "k");
avatar.headLabel = text(ax, headCenter(1), headCenter(2), headCenter(3) + 0.80 * headSize, ...
    "head/face point", "Color", [0.55, 0.05, 0.05], "FontWeight", "bold");
avatar.built = true;

state.avatar = avatar;
guidata(fig, state);
end

function updateHeadOrientation(fig, faceNormal)
% Re-pose the persistent head by swapping its hgtransform Matrix only.
state = guidata(fig);
if isfield(state, "avatar") && isgraphics(state.avatar.head)
    state.avatar.head.Matrix = avatarTransform(state.faceCenter, faceNormal);
end
end

function refreshTargetOverlay(fig, targetTform, targetPoint, faceNormal)
% Recreate only the cheap, frequently changing primitives (arrows, target
% markers, distance band, labels). The expensive avatar stays untouched.
% The caller owns the single v4 busy lock, so overlay handle bookkeeping does
% not need a second preview-specific lock.
state = guidata(fig);
ax = state.ax;
if ~isgraphics(ax)
    deleteGraphics(state.targetHandles);
    state.targetHandles = gobjects(0);
    guidata(fig, state);
    return
end

clearTargetOverlay(ax, state);
state.targetHandles = gobjects(0);

if state.targetReachable
    targetColor = [0.05, 0.35, 0.90];
else
    targetColor = [0.85, 0.10, 0.10];
end

headPoint = state.faceCenter;
nearPoint = headPoint + state.distanceRange(1) * faceNormal;
farPoint = headPoint + state.distanceRange(2) * faceNormal;
xAxis = targetTform(1:3, 1);
yAxis = targetTform(1:3, 2);
zAxis = targetTform(1:3, 3);
axisLength = state.targetAxisArrowLength;

handles = gobjects(8, 1);
handles(1) = quiver3(ax, headPoint(1), headPoint(2), headPoint(3), ...
    faceNormal(1) * state.faceNormalArrowLength, ...
    faceNormal(2) * state.faceNormalArrowLength, ...
    faceNormal(3) * state.faceNormalArrowLength, ...
    0, "LineWidth", 2.2, "Color", [0.90, 0.10, 0.10], "MaxHeadSize", 0.45);
handles(2) = plot3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    "s", "MarkerSize", 8, "MarkerFaceColor", targetColor, "MarkerEdgeColor", "k");
handles(3) = plot3(ax, ...
    [headPoint(1), targetPoint(1)], ...
    [headPoint(2), targetPoint(2)], ...
    [headPoint(3), targetPoint(3)], ...
    "--", "Color", [0.95, 0.72, 0.05], "LineWidth", 1.6);
handles(4) = plot3(ax, ...
    [nearPoint(1), farPoint(1)], ...
    [nearPoint(2), farPoint(2)], ...
    [nearPoint(3), farPoint(3)], ...
    "-", "Color", [0.15, 0.15, 0.15], "LineWidth", 2.0);
handles(5) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    xAxis(1) * axisLength, xAxis(2) * axisLength, xAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.85, 0.10, 0.10], "MaxHeadSize", 0.8);
handles(6) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    yAxis(1) * axisLength, yAxis(2) * axisLength, yAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.55, 0.16], "MaxHeadSize", 0.8);
handles(7) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    zAxis(1) * axisLength, zAxis(2) * axisLength, zAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.25, 0.90], "MaxHeadSize", 0.8);
handles(8) = text(ax, targetPoint(1), targetPoint(2), targetPoint(3) + 0.05, ...
    sprintf("target %.2f m", state.targetDistance), ...
    "Color", targetColor, "FontWeight", "bold");

tagGraphics(handles, state.targetOverlayTag);
state.targetHandles = handles;
guidata(fig, state);
end

function clearTargetOverlay(ax, state)
deleteGraphics(state.targetHandles);
if isfield(state, "targetOverlayTag") && strlength(state.targetOverlayTag) > 0 && isgraphics(ax)
    deleteGraphics(findall(ax, "Tag", char(state.targetOverlayTag)));
end
end

function tagGraphics(handles, tagValue)
for i = 1:numel(handles)
    if isgraphics(handles(i))
        handles(i).Tag = char(tagValue);
    end
end
end

function avatar = emptyAvatarHandles()
avatar = struct( ...
    "stool", gobjects(1), ...
    "body", gobjects(1), ...
    "head", gobjects(1), ...
    "cameraMarker", gobjects(1), ...
    "headMarker", gobjects(1), ...
    "headLabel", gobjects(1), ...
    "built", false);
end

function tf = avatarStaticsValid(avatar)
tf = isstruct(avatar) && isfield(avatar, "built") && avatar.built && ...
    isgraphics(avatar.stool) && isgraphics(avatar.body) && ...
    isgraphics(avatar.head) && isgraphics(avatar.cameraMarker) && ...
    isgraphics(avatar.headMarker) && isgraphics(avatar.headLabel);
end

function deleteAvatarHandles(avatar)
if ~isstruct(avatar)
    return
end
fields = ["stool", "body", "head", "cameraMarker", "headMarker", "headLabel"];
for i = 1:numel(fields)
    name = fields(i);
    if isfield(avatar, name)
        deleteGraphics(avatar.(name));
    end
end
end

function handle = drawSteveStool(ax, headCenter, headSize)
matrix = avatarTransform(headCenter, [-1; 0; 0]);
handle = hgtransform("Parent", ax, "Matrix", matrix);
handle.Tag = "SteveStoolModel";

scale = headSize / 0.16;
wood = [0.50, 0.27, 0.11];
woodDark = [0.30, 0.14, 0.06];
groundLocal = -headCenter(3);
seatTop = scale * -0.385;
seatThickness = scale * 0.035;
seatCenterZ = seatTop - seatThickness / 2;
seatBottom = seatTop - seatThickness;
legHeight = max(0.05, seatBottom - groundLocal);
legCenterZ = groundLocal + legHeight / 2;

drawCuboid(handle, [0, 0, seatCenterZ], ...
    scale * [0.260, 0.260, 0.035], wood, woodDark);

legSize = [scale * 0.032, scale * 0.032, legHeight];
for x = scale * [-0.100, 0.100]
    for y = scale * [-0.100, 0.100]
        drawCuboid(handle, [x, y, legCenterZ], legSize, wood, woodDark);
    end
end
end

function handle = drawSteveBody(ax, headCenter, headSize)
matrix = avatarTransform(headCenter, [-1; 0; 0]);
handle = hgtransform("Parent", ax, "Matrix", matrix);
handle.Tag = "SteveBodyModel";

shirt = [0.06, 0.47, 0.55];
shirtDark = [0.04, 0.30, 0.38];
skin = [0.72, 0.49, 0.31];
skinDark = [0.58, 0.34, 0.22];
jeans = [0.16, 0.20, 0.55];
jeansDark = [0.09, 0.12, 0.34];
shoe = [0.07, 0.06, 0.08];
scale = headSize / 0.16;
legCenterZ = scale * -0.355;

drawCuboid(handle, scale * [0.000, 0.000, -0.230], scale * [0.090, 0.170, 0.285], shirt, shirtDark);
drawCuboid(handle, scale * [0.000, -0.122, -0.170], scale * [0.080, 0.047, 0.120], shirt, shirtDark);
drawCuboid(handle, scale * [0.000, -0.122, -0.315], scale * [0.080, 0.047, 0.170], skin, skinDark);
drawCuboid(handle, scale * [0.000, 0.122, -0.170], scale * [0.080, 0.047, 0.120], shirt, shirtDark);
drawCuboid(handle, scale * [0.000, 0.122, -0.315], scale * [0.080, 0.047, 0.170], skin, skinDark);
drawCuboid(handle, [scale * 0.155, scale * -0.045, legCenterZ], ...
    scale * [0.310, 0.070, 0.080], jeans, jeansDark);
drawCuboid(handle, [scale * 0.155, scale * 0.045, legCenterZ], ...
    scale * [0.310, 0.070, 0.080], jeans, jeansDark);
drawCuboid(handle, [scale * 0.350, scale * -0.045, legCenterZ - scale * 0.010], ...
    scale * [0.085, 0.075, 0.090], shoe, shoe);
drawCuboid(handle, [scale * 0.350, scale * 0.045, legCenterZ - scale * 0.010], ...
    scale * [0.085, 0.075, 0.090], shoe, shoe);
end

function handle = drawSteveHead(ax, headCenter, faceNormal, headSize)
[vertices, faces, colors] = steveHeadMesh(headSize);
vertices(:, 1) = vertices(:, 1) + headFaceOffset(headSize);
frontAxis = faceNormal(:) / norm(faceNormal);
worldUp = [0; 0; 1];
upAxis = worldUp - dot(worldUp, frontAxis) * frontAxis;
if norm(upAxis) < 1e-6
    worldUp = [0; 1; 0];
    upAxis = worldUp - dot(worldUp, frontAxis) * frontAxis;
end
upAxis = upAxis / norm(upAxis);
rightAxis = cross(upAxis, frontAxis);
rightAxis = rightAxis / norm(rightAxis);
upAxis = cross(frontAxis, rightAxis);
upAxis = upAxis / norm(upAxis);

matrix = eye(4);
matrix(1:3, 1:3) = [frontAxis, rightAxis, upAxis];
matrix(1:3, 4) = headCenter(:);

handle = hgtransform("Parent", ax, "Matrix", matrix);
handle.Tag = "SteveHeadModel";
drawSteveHeadBackfill(handle, headSize, headFaceOffset(headSize));
patch( ...
    "Parent", handle, ...
    "Vertices", vertices, ...
    "Faces", faces, ...
    "FaceVertexCData", colors, ...
    "FaceColor", "flat", ...
    "EdgeColor", "none", ...
    "FaceAlpha", 1.0, ...
    "AmbientStrength", 0.75, ...
    "DiffuseStrength", 0.40);
patch( ...
    "Parent", handle, ...
    "Vertices", steveHeadOutlineVertices(headSize, headFaceOffset(headSize)), ...
    "Faces", steveHeadOutlineFaces(), ...
    "FaceColor", "none", ...
    "EdgeColor", [0.05, 0.04, 0.03], ...
    "FaceAlpha", 1.0, ...
    "LineWidth", 0.7);
end

function offset = headFaceOffset(headSize)
offset = 0.50 * headSize;
end

function drawSteveHeadBackfill(parent, headSize, xOffset)
half = headSize / 2;
skin = [0.72, 0.49, 0.31];
skinDark = [0.58, 0.34, 0.22];
hair = [0.20, 0.11, 0.06];
faceData = {
    [0, -half, -half; 0, half, -half; 0, half, half; 0, -half, half], skin
    [-headSize, half, -half; -headSize, -half, -half; -headSize, -half, half; -headSize, half, half], hair
    [0, -half, -half; -headSize, -half, -half; -headSize, -half, half; 0, -half, half], skinDark
    [-headSize, half, -half; 0, half, -half; 0, half, half; -headSize, half, half], skinDark
    [0, -half, half; 0, half, half; -headSize, half, half; -headSize, -half, half], hair
    [0, -half, -half; -headSize, -half, -half; -headSize, half, -half; 0, half, -half], skinDark
};

for i = 1:size(faceData, 1)
    vertices = faceData{i, 1};
    vertices(:, 1) = vertices(:, 1) + xOffset;
    patch( ...
        "Parent", parent, ...
        "Vertices", vertices, ...
        "Faces", [1 2 3 4], ...
        "FaceColor", faceData{i, 2}, ...
        "EdgeColor", "none", ...
        "FaceAlpha", 1.0, ...
        "AmbientStrength", 0.75, ...
        "DiffuseStrength", 0.40);
end
end

function [vertices, faces, colors] = steveHeadMesh(headSize)
% Geometry depends only on headSize, so memoize it: the 384-quad textured
% head is built at most once per size instead of on every preview update.
persistent cacheSize cacheVertices cacheFaces cacheColors
if ~isempty(cacheSize) && cacheSize == headSize
    vertices = cacheVertices;
    faces = cacheFaces;
    colors = cacheColors;
    return
end

textures = steveHeadTextures();
vertices = zeros(0, 3);
faces = zeros(0, 4);
colors = zeros(0, 3);

[vertices, faces, colors] = appendSteveFace(vertices, faces, colors, textures.front, "front", headSize);
[vertices, faces, colors] = appendSteveFace(vertices, faces, colors, textures.back, "back", headSize);
[vertices, faces, colors] = appendSteveFace(vertices, faces, colors, textures.left, "left", headSize);
[vertices, faces, colors] = appendSteveFace(vertices, faces, colors, textures.right, "right", headSize);
[vertices, faces, colors] = appendSteveFace(vertices, faces, colors, textures.top, "top", headSize);
[vertices, faces, colors] = appendSteveFace(vertices, faces, colors, textures.bottom, "bottom", headSize);

cacheSize = headSize;
cacheVertices = vertices;
cacheFaces = faces;
cacheColors = colors;
end

function [vertices, faces, colors] = appendSteveFace(vertices, faces, colors, texture, faceName, headSize)
half = headSize / 2;
depth = headSize;
step = headSize / 8;
surfaceOffset = 0.001;

for row = 1:8
    for col = 1:8
        a = -half + (col - 1) * step;
        b = -half + col * step;
        top = half - (row - 1) * step;
        bottom = half - row * step;

        switch faceName
            case "front"
                quad = [surfaceOffset, a, top; surfaceOffset, b, top; surfaceOffset, b, bottom; surfaceOffset, a, bottom];
            case "back"
                quad = [-depth - surfaceOffset, b, top; -depth - surfaceOffset, a, top; -depth - surfaceOffset, a, bottom; -depth - surfaceOffset, b, bottom];
            case "left"
                quad = [-b - half, -half - surfaceOffset, top; -a - half, -half - surfaceOffset, top; -a - half, -half - surfaceOffset, bottom; -b - half, -half - surfaceOffset, bottom];
            case "right"
                quad = [-a - half, half + surfaceOffset, top; -b - half, half + surfaceOffset, top; -b - half, half + surfaceOffset, bottom; -a - half, half + surfaceOffset, bottom];
            case "top"
                quad = [-b - half, a, half + surfaceOffset; -a - half, a, half + surfaceOffset; -a - half, b, half + surfaceOffset; -b - half, b, half + surfaceOffset];
            case "bottom"
                quad = [-a - half, a, -half - surfaceOffset; -b - half, a, -half - surfaceOffset; -b - half, b, -half - surfaceOffset; -a - half, b, -half - surfaceOffset];
        end

        first = size(vertices, 1) + 1;
        vertices = [vertices; quad]; %#ok<AGROW>
        faces = [faces; first, first + 1, first + 2, first + 3]; %#ok<AGROW>
        colors = [colors; reshape(texture(row, col, :), 1, 3)]; %#ok<AGROW>
    end
end
end

function textures = steveHeadTextures()
% Texture atlas is constant; build it once and reuse the cached struct.
persistent cachedTextures
if ~isempty(cachedTextures)
    textures = cachedTextures;
    return
end

symbols = "ABCDEF GHIJKLMNPQW";
colors = [
    0.12 0.065 0.025  % A hair nearly black
    0.17 0.095 0.035  % B hair dark brown
    0.23 0.135 0.055  % C hair brown
    0.29 0.175 0.075  % D hair warm brown
    0.36 0.225 0.105  % E hair highlight
    0.43 0.255 0.125  % F hair/side highlight
    0.00 0.000 0.000  % space fallback, unused in texture cells
    0.88 0.640 0.470  % G skin light
    0.80 0.560 0.400  % H skin base
    0.70 0.455 0.305  % I skin shadow
    0.60 0.365 0.235  % J skin dark
    0.48 0.265 0.155  % K beard
    0.33 0.170 0.080  % L beard dark
    0.55 0.315 0.220  % M mouth
    0.18 0.120 0.420  % N iris purple
    0.12 0.245 0.650  % P iris blue edge
    0.73 0.475 0.350  % Q skin side muted
    0.96 0.945 0.925  % W eye white
];

front = textureFromRows([
    "ABBBBBCB"
    "BBBBCCBB"
    "BGGHHGGB"
    "IHHHHHHI"
    "HWNIHPWH"
    "IHHKMIHJ"
    "JKLLLKJJ"
    "JLLKLLIJ"
], symbols, colors);

back = textureFromRows([
    "BBCBCBBA"
    "BACCDBBB"
    "CBCABBBB"
    "BCBBAACB"
    "BCBCABCB"
    "BBCCBBAB"
    "IIBBBBIJ"
    "JIABBBHI"
], symbols, colors);

left = textureFromRows([
    "BBBBCDBB"
    "BBBBCCCB"
    "CBBBBABB"
    "BBBBCHHB"
    "BBBBIHHI"
    "BBBIHHHI"
    "IHHHIIHI"
    "JIIHHHIJ"
], symbols, colors);

right = textureFromRows([
    "CBBBBCBB"
    "BBBDBBBB"
    "BBAABBBB"
    "ABBBBCBB"
    "BBBBHHBB"
    "BBBIHHHB"
    "IIHHIIBB"
    "JIHHHHIJ"
], symbols, colors);

top = textureFromRows([
    "BBCBBCBB"
    "BAABBBAB"
    "BCBBBBCB"
    "ABBABBBB"
    "BBBBBCBB"
    "CBBABBCB"
    "BABBABBB"
    "BBCBBBAB"
], symbols, colors);

bottom = textureFromRows([
    "GHHHHIHH"
    "HHHHHHQI"
    "HIHHHIIH"
    "HHHJHHHH"
    "IHHHHHHI"
    "HHIHHIHH"
    "GHHHHHHH"
    "HHIHHHQH"
], symbols, colors);

textures = struct( ...
    "front", front, ...
    "back", back, ...
    "left", left, ...
    "right", right, ...
    "top", top, ...
    "bottom", bottom);
cachedTextures = textures;
end

function image = textureFromRows(rows, symbols, colors)
rows = char(rows);
image = zeros(8, 8, 3);
for i = 1:strlength(symbols)
    symbol = extractBetween(symbols, i, i);
    mask = rows == char(symbol);
    for channel = 1:3
        plane = image(:, :, channel);
        plane(mask) = colors(i, channel);
        image(:, :, channel) = plane;
    end
end
end

function vertices = steveHeadOutlineVertices(headSize, xOffset)
half = headSize / 2;
vertices = [
    0, -half, -half
    0, half, -half
    0, half, half
    0, -half, half
    -headSize, -half, -half
    -headSize, half, -half
    -headSize, half, half
    -headSize, -half, half
];
vertices(:, 1) = vertices(:, 1) + xOffset;
end

function faces = steveHeadOutlineFaces()
faces = [
    1 2 3 4
    5 8 7 6
    1 5 6 2
    2 6 7 3
    3 7 8 4
    4 8 5 1
];
end

function drawCuboid(parent, center, sizeValue, color, darkColor)
cx = center(1);
cy = center(2);
cz = center(3);
sx = sizeValue(1) / 2;
sy = sizeValue(2) / 2;
sz = sizeValue(3) / 2;

vertices = [
    cx - sx, cy - sy, cz - sz
    cx + sx, cy - sy, cz - sz
    cx + sx, cy + sy, cz - sz
    cx - sx, cy + sy, cz - sz
    cx - sx, cy - sy, cz + sz
    cx + sx, cy - sy, cz + sz
    cx + sx, cy + sy, cz + sz
    cx - sx, cy + sy, cz + sz
];
faces = [
    1 2 3 4
    5 8 7 6
    1 5 6 2
    2 6 7 3
    3 7 8 4
    4 8 5 1
];
faceColors = [
    darkColor
    color
    darkColor
    color
    darkColor
    color
];

patch( ...
    "Parent", parent, ...
    "Vertices", vertices, ...
    "Faces", faces, ...
    "FaceVertexCData", faceColors, ...
    "FaceColor", "flat", ...
    "EdgeColor", [0.04, 0.035, 0.030], ...
    "LineWidth", 0.45, ...
    "FaceAlpha", 1.0, ...
    "AmbientStrength", 0.75, ...
    "DiffuseStrength", 0.40);
end

function matrix = avatarTransform(origin, frontAxis)
frontAxis = frontAxis(:);
if norm(frontAxis) < 1e-9
    frontAxis = [-1; 0; 0];
end
frontAxis = frontAxis / norm(frontAxis);

worldUp = [0; 0; 1];
upAxis = worldUp - dot(worldUp, frontAxis) * frontAxis;
if norm(upAxis) < 1e-6
    worldUp = [0; 1; 0];
    upAxis = worldUp - dot(worldUp, frontAxis) * frontAxis;
end
upAxis = upAxis / norm(upAxis);
rightAxis = cross(upAxis, frontAxis);
rightAxis = rightAxis / norm(rightAxis);
upAxis = cross(frontAxis, rightAxis);
upAxis = upAxis / norm(upAxis);

matrix = eye(4);
matrix(1:3, 1:3) = [frontAxis, rightAxis, upAxis];
matrix(1:3, 4) = origin(:);
end

function deleteGraphics(handles)
for i = 1:numel(handles)
    if isgraphics(handles(i))
        delete(handles(i));
    end
end
end

function [targetTform, targetPoint, faceNormal] = buildTargetTformFromNormal(faceCenter, distance, faceNormalWorld)
faceNormal = faceNormalWorld(:);
faceNormal = faceNormal / norm(faceNormal);
targetPoint = faceCenter(:) + distance * faceNormal;

xAxis = faceCenter(:) - targetPoint(:);
xAxis = xAxis / norm(xAxis);

worldUp = [0; 0; 1];
zAxis = worldUp - dot(worldUp, xAxis) * xAxis;
if norm(zAxis) < 1e-6
    worldUp = [0; 1; 0];
    zAxis = worldUp - dot(worldUp, xAxis) * xAxis;
end
zAxis = zAxis / norm(zAxis);
yAxis = cross(zAxis, xAxis);
yAxis = yAxis / norm(yAxis);
zAxis = cross(xAxis, yAxis);

targetTform = eye(4);
targetTform(1:3, 1:3) = [xAxis, yAxis, zAxis];
targetTform(1:3, 4) = targetPoint(:);
targetPoint = targetPoint.';
faceNormal = faceNormal.';
end

function [positionError, fullOrientationError, normalError] = poseErrors(robot, q, endEffector, targetTform)
actualTform = getTransform(robot, q, endEffector);
positionError = norm(actualTform(1:3, 4) - targetTform(1:3, 4));

rotationError = targetTform(1:3, 1:3).' * actualTform(1:3, 1:3);
axisAngle = rotm2axang(rotationError);
fullOrientationError = abs(axisAngle(4));

actualNormal = actualTform(1:3, 1);
targetNormal = targetTform(1:3, 1);
normalError = acos(max(-1, min(1, dot(actualNormal, targetNormal))));
end

function [rotationWorldFromCamera, usedImu, accelCamera, pitchAngle, forwardWorld] = cameraRotationFromImu(pose, state)
rotationNominal = state.cameraNominalRotationWorldFromCamera;
rotationWorldFromCamera = rotationNominal;
usedImu = false;
accelCamera = [NaN; NaN; NaN];
pitchAngle = 0;

if isfield(pose, "imu") && isstruct(pose.imu) && isfield(pose.imu, "accel")
    accelCamera = numericVector(pose.imu.accel);
    if ~isempty(accelCamera) && norm(accelCamera) > 0.2
        accelCamera = accelCamera(:);
        gravityCamera = accelCamera / norm(accelCamera);
        expectedGravityCamera = [0; 1; 0];
        if dot(gravityCamera, expectedGravityCamera) < 0
            gravityCamera = -gravityCamera;
        end

        pitchAngle = atan2(-gravityCamera(3), gravityCamera(2));
        rotationWorldFromCamera = rotationNominal * rotxLocal(pitchAngle);
        usedImu = true;
    end
end

forwardWorld = rotationWorldFromCamera * [0; 0; 1];
end

function matrix = rotxLocal(angle)
c = cos(angle);
s = sin(angle);
matrix = [1, 0, 0; 0, c, -s; 0, s, c];
end

function vector = numericVectorField(pose, fieldName)
vector = [];
if ~isfield(pose, fieldName)
    return
end
vector = numericVector(pose.(fieldName));
end

function vector = numericVector(value)
vector = [];
if isnumeric(value) && numel(value) == 3 && all(isfinite(value(:)))
    vector = double(value(:));
end
end

function pose = readLatestUdpPose(receiver)
pose = [];
if isempty(receiver) || ~isfield(receiver, "client") || isempty(receiver.client)
    return
end

latestText = "";
try
    while double(receiver.client.Available) > 0
        remoteEndpoint = System.Net.IPEndPoint(System.Net.IPAddress.Any, int32(0));
        bytes = receiver.client.Receive(remoteEndpoint);
        latestText = string(char(System.Text.Encoding.UTF8.GetString(bytes)));
    end
catch udpError
    if ~isUdpReceiveTimeout(udpError)
        warning("UDP receive failed: %s", udpError.message);
    end
    return
end

if strlength(strtrim(latestText)) == 0
    return
end

try
    rawPose = jsondecode(char(latestText));
    pose = extractRequiredFacePosePayload(rawPose);
catch decodeError
    warning("UDP JSON decode failed: %s", decodeError.message);
    pose = [];
end
end

function pose = extractRequiredFacePosePayload(rawPose)
% Keep the robot-control interface independent from v2 debug fields.
pose = [];
if ~isstruct(rawPose)
    return
end
if ~isfield(rawPose, "t") || ~isnumeric(rawPose.t) || ~isfinite(double(rawPose.t))
    return
end

pose = struct;
pose.t = double(rawPose.t);
pose.valid = false;
if isfield(rawPose, "valid") && (islogical(rawPose.valid) || isnumeric(rawPose.valid)) && isscalar(rawPose.valid)
    pose.valid = logical(rawPose.valid);
end

if isfield(rawPose, "normal")
    pose.normal = rawPose.normal;
end

if isfield(rawPose, "imu") && isstruct(rawPose.imu) && isfield(rawPose.imu, "accel")
    pose.imu = struct("accel", rawPose.imu.accel);
end
end

function timeout = isUdpReceiveTimeout(errorValue)
timeout = contains(string(errorValue.message), "timed out", "IgnoreCase", true) || ...
    contains(string(errorValue.message), "超时", "IgnoreCase", true);
end

function exited = pythonProcessExited(fig)
exited = false;
state = guidata(fig);
if state.pythonExitReported || isempty(state.pythonProcess)
    return
end

try
    exited = logical(state.pythonProcess.HasExited);
catch
    return
end

if ~exited
    return
end

exitCode = int32(state.pythonProcess.ExitCode);
logTail = tailTextFile(state.pythonLogFile, 3000);
if strlength(logTail) == 0
    logTail = "(log file is empty)";
end

state.pythonExitReported = true;
state.mode = "idle";
state.watchdogRecoveryPending = false;
if isgraphics(state.followButton)
    state.followButton.String = "开始跟随";
end
guidata(fig, state);

setStatus(fig, sprintf("Face module exited. Exit code: %d\nLog tail:\n%s", exitCode, logTail), [0.70, 0.05, 0.05]);
fprintf("Face module exited. Exit code: %d\nLog file: %s\n%s\n", exitCode, state.pythonLogFile, logTail);
end

function textValue = tailTextFile(path, maxChars)
textValue = "";
if ~isfile(path)
    return
end

try
    text = string(fileread(path));
catch
    return
end

if strlength(text) > maxChars
    textValue = extractAfter(text, strlength(text) - maxChars);
else
    textValue = text;
end
end

function updatePoseUi(fig, validPose, imuInfo)
state = guidata(fig);
if ~isgraphics(state.poseText)
    return
end

if validPose
    normal = state.normalFilt;
    yaw = atan2(normal(2), -normal(1));
    pitch = atan2(normal(3), hypot(normal(1), normal(2)));
    state.poseText.String = sprintf( ...
        "Live pose valid\nfiltered normal(world): [%+.3f %+.3f %+.3f]\nyaw %.1f deg, pitch %.1f deg\ntarget distance %.3f m", ...
        normal(1), normal(2), normal(3), rad2deg(yaw), rad2deg(pitch), state.targetDistance);
else
    state.poseText.String = sprintf("Live pose invalid\n%s", imuInfo);
end

if isgraphics(state.imuText)
    state.imuText.String = imuInfo;
end
guidata(fig, state);
end

function updateServoUi(fig, holding)
if ~isvalid(fig)
    return
end
state = guidata(fig);
if ~isgraphics(state.servoText)
    return
end

positionMm = state.servoLastPosErr * 1000;
orientationDeg = rad2deg(state.servoLastOriErr);
speedPct = state.servoLastQdotSat * 100;
reachText = ternaryText(state.targetReachable, "OK", "低可达");
holdText = ternaryText(holding || state.servoLastHolding, "保持", "实时");
state.servoText.String = sprintf( ...
    "mode: %s / signal: %s\npos %.1f mm, ori %.2f deg\nspeed %.0f%%, w %.3g, lambda %.4f\nrate %.1f Hz, reach %s, d %.3f m", ...
    char(state.mode), char(holdText), positionMm, orientationDeg, speedPct, ...
    state.servoLastManipW, state.servoLastLambda, state.measRateHz, ...
    char(reachText), state.targetDistance);
if state.targetReachable
    state.servoText.ForegroundColor = [0.05, 0.35, 0.12];
else
    state.servoText.ForegroundColor = [0.70, 0.05, 0.05];
end
guidata(fig, state);
end

function resetHome(fig)
state = guidata(fig);
if state.busy
    return
end
state.busy = true;
guidata(fig, state);
guard = onCleanup(@() clearBusy(fig));

state = guidata(fig);
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
state.mode = "idle";
state.targetReachable = true;
state.targetDistance = state.viewDistance;
state.watchdogRecoveryPending = false;
state.watchPosErr = Inf;
state.watchStuckTicks = 0;
if isgraphics(state.followButton)
    state.followButton.String = "开始跟随";
end
[targetTform, targetPoint] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.normalFilt);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;
guidata(fig, state);
renderScene(fig, true);
updateServoUi(fig, false);
assignin("base", "q", state.q);
setStatus(fig, "Robot reset to home pose.", [0.10, 0.10, 0.10]);
end

function exportState(fig)
state = guidata(fig);
assignTargetToBase(fig);
assignin("base", "robot", state.robot);
assignin("base", "q", state.q);
assignin("base", "faceCenter", state.faceCenter);
assignin("base", "faceNormalWorld", state.normalFilt);
assignin("base", "rawFaceNormalWorld", state.faceNormalWorld);
assignin("base", "viewDistance", state.viewDistance);
assignin("base", "targetDistance", state.targetDistance);
assignin("base", "distanceRange", state.distanceRange);
assignin("base", "latestFacePose", state.latestPose);
assignin("base", "cameraPositionWorld", state.cameraPositionWorld);
fprintf("Exported robot, q, faceCenter, faceNormalWorld, viewDistance, targetDistance, targetTform, targetScreenPoint.\n");
fprintf("faceCenter = [%.3f %.3f %.3f]\n", state.faceCenter);
fprintf("faceNormalWorld = [%.3f %.3f %.3f]\n", state.normalFilt);
fprintf("targetDistance = %.3f m\n", state.targetDistance);
end

function jointInfo = movingJointInfo(robot)
names = strings(0, 1);
types = strings(0, 1);
lower = zeros(0, 1);
upper = zeros(0, 1);

for bodyIndex = 1:numel(robot.Bodies)
    joint = robot.Bodies{bodyIndex}.Joint;
    if joint.Type == "fixed"
        continue
    end

    names(end + 1, 1) = string(joint.Name); %#ok<AGROW>
    types(end + 1, 1) = string(joint.Type); %#ok<AGROW>
    lower(end + 1, 1) = joint.PositionLimits(1); %#ok<AGROW>
    upper(end + 1, 1) = joint.PositionLimits(2); %#ok<AGROW>
end

jointInfo = struct( ...
    "names", names, ...
    "types", types, ...
    "lower", lower, ...
    "upper", upper);
end

function qdotMax = jointVelocityLimits(jointInfo, revoluteLimit, prismaticLimit)
qdotMax = zeros(numel(jointInfo.types), 1);
for i = 1:numel(qdotMax)
    if jointInfo.types(i) == "prismatic"
        qdotMax(i) = prismaticLimit;
    else
        qdotMax(i) = revoluteLimit;
    end
end
end

function assignTargetToBase(fig)
state = guidata(fig);
assignin("base", "targetTform", state.lastTargetTform);
assignin("base", "targetScreenPoint", state.lastTargetPoint);
assignin("base", "faceNormalWorld", state.normalFilt);
assignin("base", "rawFaceNormalWorld", state.faceNormalWorld);
assignin("base", "targetDistance", state.targetDistance);
assignin("base", "latestFacePose", state.latestPose);
end

function q = displayPoseToConfig(displayValues)
q = zeros(1, 6);
q(1) = deg2rad(displayValues(1));
q(2) = deg2rad(displayValues(2));
q(3) = deg2rad(displayValues(3));
q(4) = displayValues(4) / 1000;
q(5) = deg2rad(displayValues(5));
q(6) = deg2rad(displayValues(6));
end

function setStatus(fig, message, color)
if ~isvalid(fig)
    return
end
state = guidata(fig);
if isgraphics(state.statusText)
    state.statusText.String = message;
    state.statusText.ForegroundColor = color;
end
end

function cameraState = captureAxesCamera(ax)
cameraState = [];
if isempty(ax) || ~isvalid(ax)
    return
end

cameraState = struct( ...
    "CameraPosition", ax.CameraPosition, ...
    "CameraTarget", ax.CameraTarget, ...
    "CameraUpVector", ax.CameraUpVector, ...
    "CameraViewAngle", ax.CameraViewAngle, ...
    "Projection", ax.Projection);
end

function restoreAxesCamera(ax, cameraState)
if isempty(cameraState) || isempty(ax) || ~isvalid(ax)
    return
end

ax.CameraPosition = cameraState.CameraPosition;
ax.CameraTarget = cameraState.CameraTarget;
ax.CameraUpVector = cameraState.CameraUpVector;
ax.CameraViewAngle = cameraState.CameraViewAngle;
ax.Projection = cameraState.Projection;
end

function setupAxes(ax, workspaceMode)
axis(ax, "equal")
grid(ax, "on")

limits = workspaceLimits(workspaceMode);
xlim(ax, limits(1, :))
ylim(ax, limits(2, :))
zlim(ax, limits(3, :))
xlabel(ax, "X / m")
ylabel(ax, "Y / m")
zlabel(ax, "Z / m")
end

function limits = workspaceLimits(workspaceMode)
switch workspaceMode
    case "normal"
        limits = [-0.75, 0.85; -0.85, 0.85; -0.10, 1.35];
    case "large"
        limits = [-1.20, 1.20; -1.20, 1.20; -0.10, 1.55];
    case "wide"
        limits = [-2.00, 2.00; -2.00, 2.00; -0.10, 1.75];
    otherwise
        error("Unknown workspace mode '%s'. Use normal, large, or wide.", workspaceMode);
end
end

function position = depthCameraCenterWorld()
deskToBase = [-0.150, 0.000, 0.740];
cameraBackPlaneInBase = [0.038, 0.000, 0.060];
cameraDepth = 0.0252;
cameraForwardInBase = [1, 0, 0];
position = deskToBase + cameraBackPlaneInBase + 0.5 * cameraDepth * cameraForwardInBase;
end

function rotation = cameraNominalRotationWorldFromCamera()
% RealSense camera axes: +X image-right, +Y image-down, +Z optical-forward.
% Mounted model axes: +Z_camera -> +X_world, +X_camera -> -Y_world.
rotation = [0, 0, 1; -1, 0, 0; 0, -1, 0];
end

function receiver = createUdpPoseReceiver(ip, port)
receiver = struct( ...
    "ip", string(ip), ...
    "port", double(port), ...
    "client", []);

try
    client = System.Net.Sockets.UdpClient();
    client.ExclusiveAddressUse = false;
    client.Client.SetSocketOption( ...
        System.Net.Sockets.SocketOptionLevel.Socket, ...
        System.Net.Sockets.SocketOptionName.ReuseAddress, true);
    endpoint = System.Net.IPEndPoint(System.Net.IPAddress.Any, int32(port));
    client.Client.Bind(endpoint);
    client.Client.ReceiveTimeout = 5;
    receiver.client = client;
catch udpError
    error("Could not open UDP receiver on %s:%d. %s", string(ip), port, udpError.message);
end
end

function closeUdpPoseReceiver(receiver)
if isempty(receiver) || ~isfield(receiver, "client") || isempty(receiver.client)
    return
end

try
    receiver.client.Close();
catch
end
end

function layout = makeWindowLayout()
screen = get(0, "ScreenSize");
screenWidth = screen(3);
screenHeight = screen(4);

cameraWidth = max(420, round(screenWidth * 0.30));
cameraHeight = max(240, round(screenHeight * 0.28));
margin = 28;
verticalGap = 10;

availableOrthoHeight = screenHeight - cameraHeight - verticalGap - margin;
orthoHeight = min(max(300, availableOrthoHeight), screenHeight - margin);
orthoPosition = [screen(1), screen(2) + margin, cameraWidth, orthoHeight];

simX = screen(1) + cameraWidth + margin;
simY = screen(2) + margin;
simWidth = max(720, screenWidth - cameraWidth - 2 * margin);
simHeight = max(560, screenHeight - 2 * margin);

layout = struct( ...
    "cameraX", 0, ...
    "cameraY", 0, ...
    "cameraWidth", cameraWidth, ...
    "cameraHeight", cameraHeight, ...
    "orthoPosition", orthoPosition, ...
    "simPosition", [simX, simY, simWidth, simHeight]);
end

function process = startPythonFaceModule(state)
process = [];
pythonExe = fullfile("D:\Anaconda", "envs", "screen_arm_v2", "python.exe");
if ~isfile(pythonExe)
    pythonExe = "python";
end

faceModuleDir = state.faceModuleDir;
mainPath = fullfile(faceModuleDir, "main.py");
configPath = fullfile(faceModuleDir, "config.yaml");
layout = state.windowLayout;

arguments = sprintf( ...
    '"%s" --config "%s" --log-file "%s" --window-x %d --window-y %d --window-width %d --window-height %d', ...
    mainPath, configPath, state.pythonLogFile, ...
    layout.cameraX, layout.cameraY, layout.cameraWidth, layout.cameraHeight);

try
    processInfo = System.Diagnostics.ProcessStartInfo(char(pythonExe), char(arguments));
    processInfo.WorkingDirectory = char(faceModuleDir);
    processInfo.UseShellExecute = false;
    processInfo.CreateNoWindow = true;
    process = System.Diagnostics.Process.Start(processInfo);
catch startError
    warning("Could not launch Python face module via .NET: %s", startError.message);
    if ispc
        command = sprintf('start "" "%s" %s', pythonExe, arguments);
        [status, output] = system(command);
        if status ~= 0
            warning("Fallback launch failed: %s", output);
        end
    end
end
end

function closeDemo(fig)
if ~isvalid(fig)
    return
end

state = guidata(fig);
if isfield(state, "updateTimer") && ~isempty(state.updateTimer) && isvalid(state.updateTimer)
    stop(state.updateTimer);
    delete(state.updateTimer);
end

if isfield(state, "pythonProcess") && ~isempty(state.pythonProcess)
    try
        if ~state.pythonProcess.HasExited
            state.pythonProcess.Kill();
        end
    catch
    end
end

if isfield(state, "udpPoseReceiver")
    closeUdpPoseReceiver(state.udpPoseReceiver);
end

if isfield(state, "avatar")
    deleteAvatarHandles(state.avatar);
end
if isfield(state, "targetHandles")
    deleteGraphics(state.targetHandles);
end
delete(fig);
end
