function demo_face_pose_screen_arm_live_follow(workspaceMode, startFaceModule)
%DEMO_FACE_POSE_SCREEN_ARM_LIVE_FOLLOW Live face-pose driven arm demo.
%
% Usage:
%   demo_face_pose_screen_arm_live_follow
%   demo_face_pose_screen_arm_live_follow("normal")
%   demo_face_pose_screen_arm_live_follow("normal", false)  % no camera process

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
    "Name", "Live Face Pose Screen Arm Follow", ...
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

[orthoFig, orthoAxes] = createOrthoViewWindow(layout, fig);

panel = uipanel( ...
    "Parent", fig, ...
    "Title", "Live Follow", ...
    "Units", "normalized", ...
    "Position", [0.735, 0.050, 0.240, 0.750], ...
    "BackgroundColor", "w");

poseFile = fullfile(tempdir, "screen_arm_face_pose_live.json");
if isfile(poseFile)
    delete(poseFile);
end
pythonLogFile = fullfile(tempdir, "screen_arm_face_pose_live.log");
if isfile(pythonLogFile)
    delete(pythonLogFile);
end

state = struct;
state.projectRoot = projectRoot;
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
state.distanceRange = [0.35, 0.55];
state.faceNormalArrowLength = 0.15;
state.targetAxisArrowLength = 0.12;
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
state.poseFile = poseFile;
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
state.orthoFig = orthoFig;
state.orthoAxes = orthoAxes;
state.orthoGraphicsHandles = gobjects(0);

guidata(fig, state);
createControls(fig, panel);
redrawRobot(fig);
updateTargetPreview(fig);
assignTargetToBase(fig);

if startFaceModule
    state = guidata(fig);
    state.pythonProcess = startPythonFaceModule(state);
    guidata(fig, state);
    setStatus(fig, sprintf("Face module launched.\nPose: %s\nLog: %s", poseFile, pythonLogFile), [0.10, 0.10, 0.10]);
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
assignin("base", "liveFollowOrthoFigure", orthoFig);

fprintf("\nLive face-pose follow demo started.\n");
fprintf("Fixed face center: [%.3f %.3f %.3f] m\n", state.faceCenter);
fprintf("Camera center:     [%.3f %.3f %.3f] m\n", state.cameraPositionWorld);
fprintf("Pose file:         %s\n", state.poseFile);
fprintf("Variables exported: robot, q, liveFollowFigure\n\n");
end

function createControls(fig, panel)
state = guidata(fig);

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", sprintf("Fixed face center: [%.2f, %.2f, %.2f] m", state.faceCenter), ...
    "Units", "normalized", ...
    "Position", [0.06, 0.935, 0.88, 0.040], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "Distance: try 0.45 m first, accept 0.35-0.55 m", ...
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

pose = readLatestPose(state.poseFile);
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
    updatePoseUi(fig, false, "Face pose invalid.");
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
        "Not reachable in 0.35-0.55 m.\nBest: d %.3f m, pos %.1f mm, normal %.1f deg.\n%s", ...
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
redrawOrthoRobot(fig);
drawnow limitrate
end

function redrawOrthoRobot(fig)
state = guidata(fig);
if ~isfield(state, "orthoAxes")
    return
end

for axesIndex = 1:numel(state.orthoAxes)
    ax = state.orthoAxes(axesIndex);
    if ~isgraphics(ax)
        continue
    end

    show(state.robot, state.q, ...
        "Visuals", "on", ...
        "Collisions", "off", ...
        "Frames", "off", ...
        "Parent", ax, ...
        "PreservePlot", false, ...
        "FastUpdate", true);

    applyOrthoView(ax, axesIndex, state.workspaceMode);
end
end

function updateTargetPreview(fig)
state = guidata(fig);
deleteGraphics(state.graphicsHandles);
if isfield(state, "orthoGraphicsHandles")
    deleteGraphics(state.orthoGraphicsHandles);
end

[targetTform, targetPoint, faceNormal] = buildTargetTformFromNormal( ...
    state.faceCenter, state.targetDistance, state.faceNormalWorld);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;

mainHandles = drawTargetPreviewOnAxes(state.ax, state, targetTform, targetPoint, faceNormal, true);
orthoHandles = gobjects(0);
if isfield(state, "orthoAxes")
    for axesIndex = 1:numel(state.orthoAxes)
        orthoAx = state.orthoAxes(axesIndex);
        if isgraphics(orthoAx)
            orthoHandles = [orthoHandles; ...
                drawTargetPreviewOnAxes(orthoAx, state, targetTform, targetPoint, faceNormal, false)]; %#ok<AGROW>
            applyOrthoView(orthoAx, axesIndex, state.workspaceMode);
        end
    end
end

state.graphicsHandles = mainHandles;
state.orthoGraphicsHandles = orthoHandles;
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
handles = gobjects(9 + labelCount, 1);

hold(ax, "on")
handles(1) = plot3(ax, headPoint(1), headPoint(2), headPoint(3), ...
    "o", "MarkerSize", 9, "MarkerFaceColor", [0.90, 0.10, 0.10], "MarkerEdgeColor", "k");
handles(2) = quiver3(ax, headPoint(1), headPoint(2), headPoint(3), ...
    faceNormal(1) * state.faceNormalArrowLength, ...
    faceNormal(2) * state.faceNormalArrowLength, ...
    faceNormal(3) * state.faceNormalArrowLength, ...
    0, "LineWidth", 2.2, "Color", [0.90, 0.10, 0.10], "MaxHeadSize", 0.45);
handles(3) = plot3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    "s", "MarkerSize", 8, "MarkerFaceColor", targetColor, "MarkerEdgeColor", "k");
handles(4) = plot3(ax, ...
    [headPoint(1), targetPoint(1)], ...
    [headPoint(2), targetPoint(2)], ...
    [headPoint(3), targetPoint(3)], ...
    "--", "Color", [0.95, 0.72, 0.05], "LineWidth", 1.6);
handles(5) = plot3(ax, ...
    [nearPoint(1), farPoint(1)], ...
    [nearPoint(2), farPoint(2)], ...
    [nearPoint(3), farPoint(3)], ...
    "-", "Color", [0.15, 0.15, 0.15], "LineWidth", 2.0);
handles(6) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    xAxis(1) * axisLength, xAxis(2) * axisLength, xAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.85, 0.10, 0.10], "MaxHeadSize", 0.8);
handles(7) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    yAxis(1) * axisLength, yAxis(2) * axisLength, yAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.55, 0.16], "MaxHeadSize", 0.8);
handles(8) = quiver3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    zAxis(1) * axisLength, zAxis(2) * axisLength, zAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.25, 0.90], "MaxHeadSize", 0.8);
handles(9) = plot3(ax, state.cameraPositionWorld(1), state.cameraPositionWorld(2), state.cameraPositionWorld(3), ...
    "^", "MarkerSize", 7, "MarkerFaceColor", [0.20, 0.20, 0.20], "MarkerEdgeColor", "k");
if includeLabels
    handles(10) = text(ax, headPoint(1), headPoint(2), headPoint(3) + 0.05, ...
        "face center", "Color", [0.55, 0.05, 0.05], "FontWeight", "bold");
    handles(11) = text(ax, targetPoint(1), targetPoint(2), targetPoint(3) + 0.05, ...
        sprintf("target %.2f m", state.targetDistance), ...
        "Color", targetColor, "FontWeight", "bold");
end
hold(ax, "off")
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

toCamera = state.cameraPositionWorld(:) - state.faceCenter(:);
if dot(normalWorld, toCamera) < 0
    normalWorld = -normalWorld;
end

faceNormalWorld = normalWorld.';
if usedImu
    infoText = sprintf( ...
        "IMU pitch-only: %.1f deg\naccel: [%+.2f %+.2f %+.2f]\nyaw/roll fixed, cam +Z: [%+.2f %+.2f %+.2f]", ...
        rad2deg(pitchAngle), ...
        accelCamera(1), accelCamera(2), accelCamera(3), ...
        forwardWorld(1), forwardWorld(2), forwardWorld(3));
else
    infoText = sprintf( ...
        "IMU pitch-only: nominal 0.0 deg\nyaw/roll fixed, cam +Z: [%+.2f %+.2f %+.2f]", ...
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

function pose = readLatestPose(path)
pose = [];
if ~isfile(path)
    return
end

try
    text = strtrim(fileread(path));
    if strlength(text) == 0
        return
    end
    pose = jsondecode(text);
catch
    pose = [];
end
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

function [fig, axesList] = createOrthoViewWindow(layout, mainFig)
fig = figure( ...
    "Name", "Simulation Fixed Views", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "MenuBar", "none", ...
    "ToolBar", "none", ...
    "Visible", "on", ...
    "Units", "pixels", ...
    "Position", layout.orthoPosition, ...
    "CloseRequestFcn", @(src, ~) closeOrthoWindow(mainFig, src));

axesList = gobjects(3, 1);
axesPositions = [
    0.025, 0.675, 0.950, 0.305
    0.025, 0.345, 0.950, 0.305
    0.025, 0.015, 0.950, 0.305];

for axesIndex = 1:3
    axesList(axesIndex) = axes( ...
        "Parent", fig, ...
        "Units", "normalized", ...
        "Position", axesPositions(axesIndex, :));
    axesList(axesIndex).LooseInset = [0.01, 0.01, 0.01, 0.01];
    camproj(axesList(axesIndex), "orthographic")
    grid(axesList(axesIndex), "on")
end
end

function closeOrthoWindow(mainFig, orthoFig)
if isgraphics(mainFig)
    state = guidata(mainFig);
    if isfield(state, "orthoGraphicsHandles")
        deleteGraphics(state.orthoGraphicsHandles);
    end
    state.orthoFig = gobjects(1);
    state.orthoAxes = gobjects(0);
    state.orthoGraphicsHandles = gobjects(0);
    guidata(mainFig, state);
end

if isgraphics(orthoFig)
    delete(orthoFig);
end
end

function applyOrthoView(ax, axesIndex, workspaceMode)
axis(ax, "equal")
axis(ax, "vis3d")
grid(ax, "on")
box(ax, "on")

limits = workspaceLimits(workspaceMode);
xlim(ax, limits(1, :))
ylim(ax, limits(2, :))
zlim(ax, limits(3, :))
camproj(ax, "orthographic")
ax.FontSize = 7;
ax.XLabel.FontSize = 7;
ax.YLabel.FontSize = 7;
ax.ZLabel.FontSize = 7;
ax.Title.FontSize = 9;
ax.LooseInset = [0.01, 0.01, 0.01, 0.01];

switch axesIndex
    case 1
        view(ax, 0, 0)
        title(ax, "正视视角")
        xlabel(ax, "X")
        ylabel(ax, "Y")
        zlabel(ax, "Z")
    case 2
        view(ax, 0, 90)
        title(ax, "俯视视角")
        xlabel(ax, "X")
        ylabel(ax, "Y")
        zlabel(ax, "Z")
    otherwise
        view(ax, 90, 0)
        title(ax, "侧视视角")
        xlabel(ax, "X")
        ylabel(ax, "Y")
        zlabel(ax, "Z")
end
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
pythonExe = fullfile("D:\Anaconda", "envs", "screen_arm", "python.exe");
if ~isfile(pythonExe)
    pythonExe = "python";
end

faceModuleDir = fullfile(state.projectRoot, "face_pose_module");
mainPath = fullfile(faceModuleDir, "main.py");
configPath = fullfile(faceModuleDir, "config.yaml");
layout = state.windowLayout;

arguments = sprintf( ...
    '"%s" --config "%s" --pose-file "%s" --log-file "%s" --window-x %d --window-y %d --window-width %d --window-height %d', ...
    mainPath, configPath, state.poseFile, ...
    state.pythonLogFile, ...
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

deleteGraphics(state.graphicsHandles);
if isfield(state, "orthoGraphicsHandles")
    deleteGraphics(state.orthoGraphicsHandles);
end
if isfield(state, "orthoFig") && isgraphics(state.orthoFig)
    delete(state.orthoFig);
end
delete(fig);
end
