function demo_face_pose_screen_arm_live_follow_udp_avatar(workspaceMode, startFaceModule)
%DEMO_FACE_POSE_SCREEN_ARM_LIVE_FOLLOW_UDP_AVATAR UDP 3DDFA_V2 face-pose driven arm demo with avatar.
%
% Usage:
%   addpath('test_v2', '-begin')
%   demo_face_pose_screen_arm_live_follow_udp_avatar
%   demo_face_pose_screen_arm_live_follow_udp_avatar("normal")
%   demo_face_pose_screen_arm_live_follow_udp_avatar("normal", false)  % no camera process

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
    "Name", "Live Face Pose Screen Arm Follow UDP v2 Avatar", ...
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
    "Title", "Live Follow v2", ...
    "Units", "normalized", ...
    "Position", [0.735, 0.050, 0.240, 0.750], ...
    "BackgroundColor", "w");

udpIp = "127.0.0.1";
udpPort = 5005;
udpPoseReceiver = createUdpPoseReceiver(udpIp, udpPort);

pythonLogFile = fullfile(tempdir, "screen_arm_face_pose_live_udp_v2.log");
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
state.motionPositionThreshold = 0.035;
state.motionNormalThreshold = deg2rad(6);
state.minMoveIntervalSeconds = 0.40;
state.faceNormalWorld = [-1, 0, 0];
state.cameraPositionWorld = depthCameraCenterWorld();
state.cameraNominalRotationWorldFromCamera = cameraNominalRotationWorldFromCamera();
state.latestPose = [];
state.lastPoseTimestamp = -Inf;
state.lastCommandFaceNormal = [NaN, NaN, NaN];
state.lastCommandTargetPoint = [NaN, NaN, NaN];
state.lastMoveSeconds = -Inf;
state.clock = tic;
state.following = false;
state.isMoving = false;
state.targetReachable = true;
state.udpIp = udpIp;
state.udpPort = udpPort;
state.udpPoseReceiver = udpPoseReceiver;
state.pythonLogFile = pythonLogFile;
state.startFaceModule = startFaceModule;
state.pythonProcess = [];
state.pythonExitReported = false;
state.updateTimer = [];
state.graphicsHandles = gobjects(0);
state.lastTargetTform = eye(4);
state.lastTargetPoint = zeros(1, 3);
state.poseText = gobjects(1);
state.imuText = gobjects(1);
state.statusText = gobjects(1);
state.followButton = gobjects(1);
state.windowLayout = layout;

guidata(fig, state);
createControls(fig, panel);
redrawRobot(fig);
updateTargetPreview(fig);
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
    "Period", 0.10, ...
    "BusyMode", "drop", ...
    "TimerFcn", @(~, ~) timerTick(fig));
guidata(fig, state);
start(state.updateTimer);

assignin("base", "robot", robot);
assignin("base", "q", state.q);
assignin("base", "liveFollowFigure", fig);
fprintf("\nLive face-pose follow demo started.\n");
fprintf("Face module:       %s\n", state.faceModuleLabel);
fprintf("Fixed head/face reference: [%.3f %.3f %.3f] m\n", state.faceCenter);
fprintf("Camera center:     [%.3f %.3f %.3f] m\n", state.cameraPositionWorld);
fprintf("UDP input:         %s:%d\n", state.udpIp, state.udpPort);
fprintf("Variables exported: robot, q, liveFollowFigure\n\n");
end

function createControls(fig, panel)
state = guidata(fig);

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", sprintf("Fixed head/face point: [%.2f, %.2f, %.2f] m", state.faceCenter), ...
    "Units", "normalized", ...
    "Position", [0.06, 0.935, 0.88, 0.040], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "Distance: try 0.45 m first, accept 0.30-0.60 m", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.895, 0.88, 0.035], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", sprintf("Move threshold: %.0f mm or %.1f deg", ...
        state.motionPositionThreshold * 1000, rad2deg(state.motionNormalThreshold)), ...
    "Units", "normalized", ...
    "Position", [0.06, 0.855, 0.88, 0.035], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.poseText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "No live face pose yet.", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.665, 0.88, 0.165], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.imuText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "IMU tilt: waiting.", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.530, 0.88, 0.110], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.statusText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.320, 0.88, 0.170], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

state.followButton = uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "开始跟随", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.235, 0.88, 0.060], ...
    "Callback", @(~, ~) toggleFollow(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Plan Once", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.165, 0.42, 0.050], ...
    "Callback", @(~, ~) planAndMove(fig, true));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Reset Home", ...
    "Units", "normalized", ...
    "Position", [0.52, 0.165, 0.42, 0.050], ...
    "Callback", @(~, ~) resetHome(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Export", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.095, 0.42, 0.050], ...
    "Callback", @(~, ~) exportState(fig));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Close", ...
    "Units", "normalized", ...
    "Position", [0.52, 0.095, 0.42, 0.050], ...
    "Callback", @(~, ~) closeDemo(fig));

guidata(fig, state);
end

function timerTick(fig)
if ~isvalid(fig)
    return
end

state = guidata(fig);
if state.isMoving
    return
end

if state.startFaceModule && pythonProcessExited(fig)
    return
end

pose = readLatestUdpPose(state.udpPoseReceiver);
if isempty(pose)
    return
end

if ~isfield(pose, "t") || double(pose.t) <= state.lastPoseTimestamp
    return
end

state.lastPoseTimestamp = double(pose.t);
state.latestPose = pose;

if ~isfield(pose, "valid") || ~logical(pose.valid)
    guidata(fig, state);
    updatePoseUi(fig, false, "Face pose invalid. Waiting for a valid v2 normal.");
    return
end

[faceNormalWorld, imuInfo] = faceNormalWorldFromPose(pose, state);
if isempty(faceNormalWorld)
    guidata(fig, state);
    updatePoseUi(fig, false, "Face normal missing.");
    return
end

state.faceNormalWorld = faceNormalWorld;
state.targetReachable = true;
guidata(fig, state);

updateTargetPreview(fig);
updatePoseUi(fig, true, imuInfo);
assignTargetToBase(fig);

state = guidata(fig);
if state.following && targetChangeNeedsMove(state)
    planAndMove(fig, false);
end
end

function toggleFollow(fig)
state = guidata(fig);
state.following = ~state.following;
if state.following
    state.followButton.String = "暂停跟随";
    message = "Following enabled. Robot will move only after the face normal changes enough.";
else
    state.followButton.String = "开始跟随";
    message = "Following paused. Preview continues to update.";
end
guidata(fig, state);
setStatus(fig, message, [0.10, 0.10, 0.10]);

if state.following
    planAndMove(fig, true);
end
end

function planAndMove(fig, forceMove)
if nargin < 2
    forceMove = false;
end

state = guidata(fig);
if state.isMoving
    return
end
if ~forceMove && ~targetChangeNeedsMove(state)
    return
end

state.isMoving = true;
guidata(fig, state);

solveResult = solveTargetWithDistanceFallback(state);
state = guidata(fig);
state.targetDistance = solveResult.distance;
state.targetReachable = solveResult.reachable;
guidata(fig, state);
updateTargetPreview(fig);

fprintf("IK status: %s\n", string(solveResult.status));
fprintf("Target distance used: %.3f m%s\n", ...
    solveResult.distance, ternaryText(solveResult.usedFallback, " (fallback)", ""));
fprintf("Position error: %.4f m, normal error: %.2f deg, full orientation error: %.2f deg\n", ...
    solveResult.positionError, rad2deg(solveResult.normalError), ...
    rad2deg(solveResult.fullOrientationError));

if solveResult.reachable
    if solveResult.usedFallback
        statusMessage = sprintf( ...
            "Reachable with fallback %.3f m.\nPos %.1f mm, normal %.1f deg.", ...
            solveResult.distance, solveResult.positionError * 1000, ...
            rad2deg(solveResult.normalError));
    else
        statusMessage = sprintf( ...
            "Reachable at nominal 0.45 m.\nPos %.1f mm, normal %.1f deg.", ...
            solveResult.positionError * 1000, rad2deg(solveResult.normalError));
    end
    setStatus(fig, statusMessage, [0.05, 0.35, 0.12]);
    qNew = animateJointTrajectory(fig, state.q, solveResult.q);

    state = guidata(fig);
    state.q = qNew;
    state.targetReachable = true;
    state.targetDistance = solveResult.distance;
    state.lastCommandFaceNormal = state.faceNormalWorld;
    state.lastCommandTargetPoint = nominalTargetPoint(state);
    state.lastMoveSeconds = toc(state.clock);
    state.isMoving = false;
    guidata(fig, state);

    assignin("base", "q", qNew);
    assignTargetToBase(fig);
    setStatus(fig, "Arrived. Waiting for the next meaningful face-normal change.", [0.05, 0.35, 0.12]);
else
    diagnosis = diagnoseUnreachable(state, solveResult);
    state = guidata(fig);
    state.targetReachable = false;
    state.lastCommandFaceNormal = state.faceNormalWorld;
    state.lastCommandTargetPoint = nominalTargetPoint(state);
    state.lastMoveSeconds = toc(state.clock);
    state.isMoving = false;
    guidata(fig, state);
    updateTargetPreview(fig);
    setStatus(fig, sprintf( ...
        "Not reachable in %.2f-%.2f m.\nBest: d %.3f m, pos %.1f mm, normal %.1f deg.\n%s", ...
        state.distanceRange(1), state.distanceRange(2), ...
        solveResult.distance, solveResult.positionError * 1000, ...
        rad2deg(solveResult.normalError), diagnosis), [0.70, 0.05, 0.05]);
end
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

function diagnosis = diagnoseUnreachable(state, solveResult)
[qLoose, ~] = state.ikLoose( ...
    state.endEffector, solveResult.targetTform, state.weights, state.q);
[loosePositionError, ~, looseNormalError] = poseErrors( ...
    state.robot, qLoose, state.endEffector, solveResult.targetTform);

if loosePositionError <= 2 * state.positionTolerance && ...
        looseNormalError <= 1.5 * state.normalTolerance
    [limitLine, causeLine] = jointLimitDiagnosis(state.jointInfo, qLoose);
    if strlength(limitLine) > 0
        diagnosis = sprintf("%s\n%s", limitLine, causeLine);
        fprintf("%s\n", diagnosis);
        return
    end
end

diagnosis = sprintf("Limit: no single joint over-limit found.\n%s", ...
    viewDirectionDiagnosis(state.faceNormalWorld));
fprintf("%s\n", diagnosis);
end

function [limitLine, causeLine] = jointLimitDiagnosis(jointInfo, q)
q = q(:);
lowerLimits = jointInfo.lower(:);
upperLimits = jointInfo.upper(:);

below = lowerLimits - q;
above = q - upperLimits;
excess = max([below, above, zeros(size(q))], [], 2);
violating = find(excess > 1e-5);

if isempty(violating)
    limitLine = "";
    causeLine = "";
    return
end

[~, order] = sort(excess(violating), "descend");
jointIndex = violating(order(1));
name = jointInfo.names(jointIndex);
type = jointInfo.types(jointIndex);

if below(jointIndex) > above(jointIndex)
    directionText = "below min";
    limitValue = lowerLimits(jointIndex);
    overAmount = below(jointIndex);
else
    directionText = "above max";
    limitValue = upperLimits(jointIndex);
    overAmount = above(jointIndex);
end

[neededDisplay, unit] = jointDisplayValue(q(jointIndex), type);
[limitDisplay, ~] = jointDisplayValue(limitValue, type);
[overDisplay, ~] = jointDisplayValue(overAmount, type);

limitLine = sprintf("Limit: %s %s by %.1f %s.", ...
    jointLabel(name), directionText, abs(overDisplay), unit);
limitLine = sprintf("%s Need %.1f, limit %.1f.", ...
    limitLine, neededDisplay, limitDisplay);

category = jointMotionCategory(name);
switch category
    case "pitch"
        causeLine = "Cause: pitch-side joint limit is dominant.";
    case "yaw"
        causeLine = "Cause: yaw/pan joint limit is dominant.";
    case "distance"
        causeLine = "Cause: telescopic distance limit is dominant.";
    otherwise
        causeLine = "Cause: combined joint limit is dominant.";
end
end

function textValue = viewDirectionDiagnosis(faceNormalWorld)
yaw = atan2(faceNormalWorld(2), -faceNormalWorld(1));
pitch = atan2(faceNormalWorld(3), hypot(faceNormalWorld(1), faceNormalWorld(2)));
yawRatio = abs(rad2deg(yaw)) / 60;
pitchRatio = abs(rad2deg(pitch)) / 35;

if yawRatio > pitchRatio + 0.15
    textValue = "Cause: face yaw demand is likely too large.";
elseif pitchRatio > yawRatio + 0.15
    textValue = "Cause: face pitch demand is likely too large.";
elseif yawRatio > 0.7 && pitchRatio > 0.7
    textValue = "Cause: yaw and pitch demands are both near the practical limit.";
else
    textValue = "Cause: yaw/pitch combination or workspace boundary.";
end
end

function category = jointMotionCategory(name)
if contains(name, "yaw") || contains(name, "pan")
    category = "yaw";
elseif contains(name, "pitch")
    category = "pitch";
elseif contains(name, "telescopic")
    category = "distance";
else
    category = "other";
end
end

function label = jointLabel(name)
switch string(name)
    case "joint1_base_yaw"
        label = "J1 Base yaw";
    case "joint2_shoulder_pitch"
        label = "J2 Shoulder pitch";
    case "joint3_elbow_pitch"
        label = "J3 Elbow pitch";
    case "joint4_telescopic"
        label = "J4 Telescopic";
    case "joint5_screen_pan"
        label = "J5 Screen pan";
    case "joint6_screen_pitch"
        label = "J6 Screen pitch";
    otherwise
        label = char(name);
end
end

function [displayValue, unit] = jointDisplayValue(value, jointType)
if jointType == "prismatic"
    displayValue = value * 1000;
    unit = "mm";
else
    displayValue = rad2deg(value);
    unit = "deg";
end
end

function textValue = ternaryText(condition, trueText, falseText)
if condition
    textValue = trueText;
else
    textValue = falseText;
end
end

function needsMove = targetChangeNeedsMove(state)
elapsed = toc(state.clock) - state.lastMoveSeconds;
if elapsed < state.minMoveIntervalSeconds
    needsMove = false;
    return
end

if any(isnan(state.lastCommandFaceNormal)) || any(isnan(state.lastCommandTargetPoint))
    needsMove = true;
    return
end

currentPoint = nominalTargetPoint(state);
positionDelta = norm(currentPoint - state.lastCommandTargetPoint);
normalDelta = vectorAngle(state.faceNormalWorld, state.lastCommandFaceNormal);

needsMove = ...
    positionDelta >= state.motionPositionThreshold || ...
    normalDelta >= state.motionNormalThreshold;
end

function point = nominalTargetPoint(state)
normal = state.faceNormalWorld / norm(state.faceNormalWorld);
point = state.faceCenter + state.viewDistance * normal;
end

function q = animateJointTrajectory(fig, qStart, qGoal)
maxDelta = max(abs(qGoal(:) - qStart(:)));
frameCount = max(24, min(60, ceil(24 + 18 * maxDelta)));
secondsPerFrame = 0.012;
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
    updateTargetPreview(fig);
    drawnow limitrate
    pause(secondsPerFrame)
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
title(state.ax, "Live face-pose target IK and fast joint trajectory")
drawnow limitrate
end

function updateTargetPreview(fig)
state = guidata(fig);
deleteGraphics(state.graphicsHandles);

[targetTform, targetPoint, faceNormal] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.faceNormalWorld);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;

mainHandles = drawTargetPreviewOnAxes(state.ax, state, targetTform, targetPoint, faceNormal, true);
state.graphicsHandles = mainHandles;

guidata(fig, state);
drawnow limitrate
end

function handles = drawTargetPreviewOnAxes(ax, state, targetTform, targetPoint, faceNormal, includeLabels)
handles = gobjects(0);
if ~isgraphics(ax)
    return
end

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

labelCount = 2 * double(includeLabels);
hold(ax, "on")
avatarHandles = drawSteveAvatarOnAxes(ax, headPoint, faceNormal, state.steveHeadSize);
targetHandles = gobjects(9 + labelCount, 1);

targetHandles(1) = plot3(ax, headPoint(1), headPoint(2), headPoint(3), ...
    "o", "MarkerSize", 9, "MarkerFaceColor", [0.90, 0.10, 0.10], "MarkerEdgeColor", "k");
targetHandles(2) = quiver3(ax, headPoint(1), headPoint(2), headPoint(3), ...
    faceNormal(1) * state.faceNormalArrowLength, ...
    faceNormal(2) * state.faceNormalArrowLength, ...
    faceNormal(3) * state.faceNormalArrowLength, ...
    0, "LineWidth", 2.2, "Color", [0.90, 0.10, 0.10], "MaxHeadSize", 0.45);
targetHandles(3) = plot3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    "s", "MarkerSize", 8, "MarkerFaceColor", targetColor, "MarkerEdgeColor", "k");
targetHandles(4) = plot3(ax, ...
    [headPoint(1), targetPoint(1)], ...
    [headPoint(2), targetPoint(2)], ...
    [headPoint(3), targetPoint(3)], ...
    "--", "Color", [0.95, 0.72, 0.05], "LineWidth", 1.6);
targetHandles(5) = plot3(ax, ...
    [nearPoint(1), farPoint(1)], ...
    [nearPoint(2), farPoint(2)], ...
    [nearPoint(3), farPoint(3)], ...
    "-", "Color", [0.15, 0.15, 0.15], "LineWidth", 2.0);
targetHandles(6) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    xAxis(1) * axisLength, xAxis(2) * axisLength, xAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.85, 0.10, 0.10], "MaxHeadSize", 0.8);
targetHandles(7) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    yAxis(1) * axisLength, yAxis(2) * axisLength, yAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.55, 0.16], "MaxHeadSize", 0.8);
targetHandles(8) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    zAxis(1) * axisLength, zAxis(2) * axisLength, zAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.25, 0.90], "MaxHeadSize", 0.8);
targetHandles(9) = plot3(ax, state.cameraPositionWorld(1), state.cameraPositionWorld(2), state.cameraPositionWorld(3), ...
    "^", "MarkerSize", 7, "MarkerFaceColor", [0.20, 0.20, 0.20], "MarkerEdgeColor", "k");
if includeLabels
    targetHandles(10) = text(ax, headPoint(1), headPoint(2), headPoint(3) + 0.80 * state.steveHeadSize, ...
        "head/face point", "Color", [0.55, 0.05, 0.05], "FontWeight", "bold");
    targetHandles(11) = text(ax, targetPoint(1), targetPoint(2), targetPoint(3) + 0.05, ...
        sprintf("target %.2f m", state.targetDistance), ...
        "Color", targetColor, "FontWeight", "bold");
end
hold(ax, "off")
handles = [avatarHandles(:); targetHandles(:)];
end

function handles = drawSteveAvatarOnAxes(ax, headCenter, faceNormal, headSize)
handles = gobjects(3, 1);
handles(1) = drawSteveStool(ax, headCenter, headSize);
handles(2) = drawSteveBody(ax, headCenter, headSize);
handles(3) = drawSteveHead(ax, headCenter, faceNormal, headSize);
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

function [faceNormalWorld, infoText] = faceNormalWorldFromPose(pose, state)
faceNormalWorld = [];
normalCamera = numericVectorField(pose, "normal");
if isempty(normalCamera)
    infoText = "Normal missing.";
    return
end

[cameraRotation, usedImu, accelCamera, pitchAngle, forwardWorld] = cameraRotationFromImu(pose, state);
normalWorld = cameraRotation * normalCamera(:);
if norm(normalWorld) < 1e-9
    infoText = "Normal has near-zero length.";
    return
end
normalWorld = normalWorld / norm(normalWorld);

% v2 defines normal as the face-facing direction in RealSense RGB camera axes.
% Keep its sign here; MATLAB only rotates it into the world frame.
faceNormalWorld = normalWorld.';
if usedImu
    infoText = sprintf( ...
        "v2 fields used: t, valid, normal, imu.accel\nIMU pitch-only: %.1f deg\naccel: [%+.2f %+.2f %+.2f]\nyaw/roll fixed, cam +Z: [%+.2f %+.2f %+.2f]", ...
        rad2deg(pitchAngle), ...
        accelCamera(1), accelCamera(2), accelCamera(3), ...
        forwardWorld(1), forwardWorld(2), forwardWorld(3));
else
    infoText = sprintf( ...
        "v2 fields used: t, valid, normal\nIMU pitch-only: nominal 0.0 deg\nyaw/roll fixed, cam +Z: [%+.2f %+.2f %+.2f]", ...
        forwardWorld(1), forwardWorld(2), forwardWorld(3));
end
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
state.following = false;
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
    normal = state.faceNormalWorld;
    yaw = atan2(normal(2), -normal(1));
    pitch = atan2(normal(3), hypot(normal(1), normal(2)));
    state.poseText.String = sprintf( ...
        "Live pose valid\nnormal(world): [%+.3f %+.3f %+.3f]\nyaw %.1f deg, pitch %.1f deg\ntarget distance %.3f m", ...
        normal(1), normal(2), normal(3), rad2deg(yaw), rad2deg(pitch), state.targetDistance);
else
    state.poseText.String = sprintf("Live pose invalid\n%s", imuInfo);
end

if isgraphics(state.imuText)
    state.imuText.String = imuInfo;
end
guidata(fig, state);
end

function resetHome(fig)
state = guidata(fig);
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
state.targetReachable = true;
state.lastCommandFaceNormal = [NaN, NaN, NaN];
state.lastCommandTargetPoint = [NaN, NaN, NaN];
guidata(fig, state);
redrawRobot(fig);
updateTargetPreview(fig);
assignin("base", "q", state.q);
setStatus(fig, "Robot reset to home pose.", [0.10, 0.10, 0.10]);
end

function exportState(fig)
state = guidata(fig);
assignTargetToBase(fig);
assignin("base", "robot", state.robot);
assignin("base", "q", state.q);
assignin("base", "faceCenter", state.faceCenter);
assignin("base", "faceNormalWorld", state.faceNormalWorld);
assignin("base", "viewDistance", state.viewDistance);
assignin("base", "targetDistance", state.targetDistance);
assignin("base", "distanceRange", state.distanceRange);
assignin("base", "latestFacePose", state.latestPose);
assignin("base", "cameraPositionWorld", state.cameraPositionWorld);
fprintf("Exported robot, q, faceCenter, faceNormalWorld, viewDistance, targetDistance, targetTform, targetScreenPoint.\n");
fprintf("faceCenter = [%.3f %.3f %.3f]\n", state.faceCenter);
fprintf("faceNormalWorld = [%.3f %.3f %.3f]\n", state.faceNormalWorld);
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

function assignTargetToBase(fig)
state = guidata(fig);
assignin("base", "targetTform", state.lastTargetTform);
assignin("base", "targetScreenPoint", state.lastTargetPoint);
assignin("base", "faceNormalWorld", state.faceNormalWorld);
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
if isempty(ax) || ~isvalid(ax) || isempty(ax.Children)
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

function angle = vectorAngle(a, b)
a = a(:) / norm(a);
b = b(:) / norm(b);
angle = acos(max(-1, min(1, dot(a, b))));
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

deleteGraphics(state.graphicsHandles);
delete(fig);
end
