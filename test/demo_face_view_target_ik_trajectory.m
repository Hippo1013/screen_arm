function demo_face_view_target_ik_trajectory(workspaceMode)
%DEMO_FACE_VIEW_TARGET_IK_TRAJECTORY Face-view target IK and trajectory demo.
%
% Usage:
%   demo_face_view_target_ik_trajectory
%   demo_face_view_target_ik_trajectory("wide")
%
% The face center is fixed for this test. The two sliders control the face
% viewing direction. Click "Plan + Move" to solve IK for the target screen
% pose and animate a smooth joint-space trajectory to the result.

if nargin < 1 || strlength(string(workspaceMode)) == 0
    workspaceMode = "large";
else
    workspaceMode = lower(string(workspaceMode));
end

projectRoot = fileparts(fileparts(mfilename("fullpath")));
urdfPath = fullfile(projectRoot, "screen_arm", "generated", "urdf", ...
    "face_screen_support_arm_depth_camera.urdf");

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

ik = inverseKinematics("RigidBodyTree", robot);
ik.SolverParameters.MaxIterations = 1500;
ik.SolverParameters.MaxTime = 1.5;

ikLoose = inverseKinematics("RigidBodyTree", robot);
ikLoose.SolverParameters.MaxIterations = 1000;
ikLoose.SolverParameters.MaxTime = 0.8;
ikLoose.SolverParameters.EnforceJointLimits = false;

fig = figure( ...
    "Name", "Face View IK Trajectory Demo", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "MenuBar", "none", ...
    "ToolBar", "figure", ...
    "Visible", "on", ...
    "Units", "normalized", ...
    "Position", [0.08, 0.10, 0.84, 0.78]);

ax = axes( ...
    "Parent", fig, ...
    "Units", "normalized", ...
    "Position", [0.05, 0.08, 0.62, 0.86]);
view(ax, 135, 25)
camproj(ax, "perspective")
rotate3d(fig, "on")

panel = uipanel( ...
    "Parent", fig, ...
    "Title", "Face Direction", ...
    "Units", "normalized", ...
    "Position", [0.70, 0.08, 0.27, 0.86], ...
    "BackgroundColor", "w");

state = struct;
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
state.viewAngles = [0, 0]; % yaw about world Z, pitch about world Y, degrees.
state.positionTolerance = 0.025;
state.normalTolerance = deg2rad(8);
state.sliders = gobjects(2, 1);
state.edits = gobjects(2, 1);
state.valueTexts = gobjects(2, 1);
state.listeners = cell(2, 1);
state.statusText = gobjects(1);
state.graphicsHandles = gobjects(0);
state.targetReachable = true;
state.isUpdatingUi = false;
state.lastTargetTform = eye(4);
state.lastTargetPoint = zeros(1, 3);
state.lastFaceNormal = [-1, 0, 0];

guidata(fig, state);
createControls(fig, panel);
redrawRobot(fig);
updateTargetPreview(fig);
setStatus(fig, "Adjust yaw/pitch, then click Plan + Move.", [0.10, 0.10, 0.10]);

assignin("base", "robot", robot);
assignin("base", "q", state.q);
assignin("base", "faceViewFigure", fig);

fprintf("\nFace-view IK trajectory demo started.\n");
fprintf("Fixed face center: [%.3f %.3f %.3f] m\n", state.faceCenter);
fprintf("Screen distance:   %.3f m\n", state.viewDistance);
fprintf("Variables exported: robot, q, faceViewFigure\n\n");
end

function createControls(fig, panel)
state = guidata(fig);

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "Fixed face center: [0.65, 0.00, 1.00] m", ...
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

defs = createAngleDefinitions();
top = 0.78;
rowHeight = 0.145;

for i = 1:2
    y = top - (i - 1) * rowHeight;
    def = defs(i);
    value = state.viewAngles(i);

    uicontrol( ...
        "Parent", panel, ...
        "Style", "text", ...
        "String", def.label, ...
        "Units", "normalized", ...
        "Position", [0.06, y + 0.058, 0.52, 0.035], ...
        "HorizontalAlignment", "left", ...
        "BackgroundColor", "w");

    state.valueTexts(i) = uicontrol( ...
        "Parent", panel, ...
        "Style", "text", ...
        "String", sprintf("% .1f deg", value), ...
        "Units", "normalized", ...
        "Position", [0.60, y + 0.058, 0.34, 0.035], ...
        "HorizontalAlignment", "right", ...
        "BackgroundColor", "w");

    state.sliders(i) = uicontrol( ...
        "Parent", panel, ...
        "Style", "slider", ...
        "Min", def.min, ...
        "Max", def.max, ...
        "Value", value, ...
        "SliderStep", normalizedSliderStep(def), ...
        "Units", "normalized", ...
        "Position", [0.06, y + 0.020, 0.60, 0.040], ...
        "Callback", @(src, ~) angleSliderChanged(fig, src));

    state.listeners{i} = addlistener( ...
        state.sliders(i), ...
        "Value", ...
        "PostSet", ...
        @(~, event) angleSliderChanged(fig, event.AffectedObject));

    state.edits(i) = uicontrol( ...
        "Parent", panel, ...
        "Style", "edit", ...
        "String", sprintf("%.1f", value), ...
        "Units", "normalized", ...
        "Position", [0.70, y + 0.014, 0.24, 0.050], ...
        "HorizontalAlignment", "left", ...
        "BackgroundColor", "white", ...
        "Callback", @(src, ~) angleEditChanged(fig, src));
end

state.statusText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.235, 0.88, 0.130], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Plan + Move", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.165, 0.88, 0.055], ...
    "Callback", @(~, ~) planAndMove(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Reset Angles", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.100, 0.42, 0.045], ...
    "Callback", @(~, ~) resetAngles(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Reset Home", ...
    "Units", "normalized", ...
    "Position", [0.52, 0.100, 0.42, 0.045], ...
    "Callback", @(~, ~) resetHome(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Export", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.040, 0.42, 0.045], ...
    "Callback", @(~, ~) exportState(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Close", ...
    "Units", "normalized", ...
    "Position", [0.52, 0.040, 0.42, 0.045], ...
    "Callback", @(~, ~) close(fig));

guidata(fig, state);
end

function defs = createAngleDefinitions()
defs = struct( ...
    "label", {"Yaw about world Z", "Pitch about world Y"}, ...
    "min", {-60, -35}, ...
    "max", {60, 35}, ...
    "smallStep", {1, 1}, ...
    "bigStep", {10, 5});
end

function step = normalizedSliderStep(def)
range = def.max - def.min;
step = [def.smallStep / range, def.bigStep / range];
step = min(max(step, 0.0001), 1);
end

function angleSliderChanged(fig, slider)
if ~isvalid(fig)
    return
end

state = guidata(fig);
if state.isUpdatingUi
    return
end

index = find(state.sliders == slider, 1);
if isempty(index)
    return
end

state.viewAngles(index) = slider.Value;
state = updateAngleUiValue(state, index);
state.targetReachable = true;
state.targetDistance = state.viewDistance;
guidata(fig, state);
updateTargetPreview(fig);
assignTargetToBase(fig);
end

function angleEditChanged(fig, editBox)
state = guidata(fig);
index = find(state.edits == editBox, 1);
if isempty(index)
    return
end

value = str2double(editBox.String);
if isnan(value)
    state = updateAngleUiValue(state, index);
    guidata(fig, state);
    return
end

defs = createAngleDefinitions();
def = defs(index);
value = min(max(value, def.min), def.max);

state.viewAngles(index) = value;
state.targetDistance = state.viewDistance;
state.isUpdatingUi = true;
state.sliders(index).Value = value;
state.isUpdatingUi = false;
state = updateAngleUiValue(state, index);
state.targetReachable = true;
guidata(fig, state);
updateTargetPreview(fig);
assignTargetToBase(fig);
end

function state = updateAngleUiValue(state, index)
value = state.viewAngles(index);
state.valueTexts(index).String = sprintf("% .1f deg", value);
state.edits(index).String = sprintf("%.1f", value);
end

function planAndMove(fig)
state = guidata(fig);
solveResult = solveTargetWithDistanceFallback(state);
qSolution = solveResult.q;

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
            "Reachable with fallback distance %.3f m.\nPos %.1f mm, normal %.1f deg.", ...
            solveResult.distance, solveResult.positionError * 1000, ...
            rad2deg(solveResult.normalError));
    else
        statusMessage = sprintf( ...
            "Reachable at nominal 0.45 m.\nPos %.1f mm, normal %.1f deg.", ...
            solveResult.positionError * 1000, rad2deg(solveResult.normalError));
    end
    setStatus(fig, statusMessage, [0.05, 0.35, 0.12]);
    qNew = animateJointTrajectory(fig, state.q, qSolution);
    state = guidata(fig);
    state.q = qNew;
    state.targetReachable = true;
    state.targetDistance = solveResult.distance;
    guidata(fig, state);
    assignin("base", "q", qNew);
    assignTargetToBase(fig);
    setStatus(fig, "Arrived. Adjust yaw/pitch for the next target.", [0.05, 0.35, 0.12]);
else
    diagnosis = diagnoseUnreachable(state, solveResult);
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
targetTform = buildTargetTform(state.faceCenter, distance, state.viewAngles);
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
    viewAngleDiagnosis(state.viewAngles));
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

function textValue = viewAngleDiagnosis(viewAngles)
yawRatio = abs(viewAngles(1)) / 60;
pitchRatio = abs(viewAngles(2)) / 35;

if yawRatio > pitchRatio + 0.15
    textValue = "Cause: face yaw demand is likely too large.";
elseif pitchRatio > yawRatio + 0.15
    textValue = "Cause: face pitch demand is likely too large.";
elseif yawRatio > 0.7 && pitchRatio > 0.7
    textValue = "Cause: yaw and pitch are both near the test limit.";
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

function resetAngles(fig)
state = guidata(fig);
state.viewAngles = [0, 0];
state.targetDistance = state.viewDistance;
state.targetReachable = true;
state.isUpdatingUi = true;
for i = 1:2
    state.sliders(i).Value = state.viewAngles(i);
    state = updateAngleUiValue(state, i);
end
state.isUpdatingUi = false;
guidata(fig, state);
updateTargetPreview(fig);
assignTargetToBase(fig);
setStatus(fig, "Face direction reset. Click Plan + Move if needed.", [0.10, 0.10, 0.10]);
end

function resetHome(fig)
state = guidata(fig);
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
state.targetReachable = true;
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
assignin("base", "viewAngles", state.viewAngles);
assignin("base", "viewDistance", state.viewDistance);
assignin("base", "targetDistance", state.targetDistance);
assignin("base", "distanceRange", state.distanceRange);
fprintf("Exported robot, q, faceCenter, viewAngles, viewDistance, targetDistance, targetTform, targetScreenPoint.\n");
fprintf("faceCenter = [%.3f %.3f %.3f]\n", state.faceCenter);
fprintf("viewAngles = [%.1f %.1f] deg\n", state.viewAngles);
fprintf("targetDistance = %.3f m\n", state.targetDistance);
end

function q = animateJointTrajectory(fig, qStart, qGoal)
frameCount = 90;
secondsPerFrame = 0.025;
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
    drawnow
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
title(state.ax, "Face-view target IK and joint-space trajectory")
drawnow limitrate
end

function updateTargetPreview(fig)
state = guidata(fig);
deleteGraphics(state.graphicsHandles);

[targetTform, targetPoint, faceNormal] = buildTargetTform( ...
    state.faceCenter, state.targetDistance, state.viewAngles);
state.lastTargetTform = targetTform;
state.lastTargetPoint = targetPoint;
state.lastFaceNormal = faceNormal;

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
axisLength = 0.12;

hold(state.ax, "on")
handles = gobjects(10, 1);
handles(1) = plot3(state.ax, headPoint(1), headPoint(2), headPoint(3), ...
    "o", "MarkerSize", 9, "MarkerFaceColor", [0.90, 0.10, 0.10], "MarkerEdgeColor", "k");
handles(2) = quiver3(state.ax, headPoint(1), headPoint(2), headPoint(3), ...
    faceNormal(1) * state.faceNormalArrowLength, ...
    faceNormal(2) * state.faceNormalArrowLength, ...
    faceNormal(3) * state.faceNormalArrowLength, ...
    0, "LineWidth", 2.2, "Color", [0.90, 0.10, 0.10], "MaxHeadSize", 0.45);
handles(3) = plot3(state.ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    "s", "MarkerSize", 8, "MarkerFaceColor", targetColor, "MarkerEdgeColor", "k");
handles(4) = plot3(state.ax, ...
    [headPoint(1), targetPoint(1)], ...
    [headPoint(2), targetPoint(2)], ...
    [headPoint(3), targetPoint(3)], ...
    "--", "Color", [0.55, 0.10, 0.10], "LineWidth", 1.4);
handles(5) = plot3(state.ax, ...
    [nearPoint(1), farPoint(1)], ...
    [nearPoint(2), farPoint(2)], ...
    [nearPoint(3), farPoint(3)], ...
    "-", "Color", [0.15, 0.15, 0.15], "LineWidth", 2.0);
handles(6) = quiver3(state.ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    xAxis(1) * axisLength, xAxis(2) * axisLength, xAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.85, 0.10, 0.10], "MaxHeadSize", 0.8);
handles(7) = quiver3(state.ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    yAxis(1) * axisLength, yAxis(2) * axisLength, yAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.55, 0.16], "MaxHeadSize", 0.8);
handles(8) = quiver3(state.ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
    zAxis(1) * axisLength, zAxis(2) * axisLength, zAxis(3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.25, 0.90], "MaxHeadSize", 0.8);
handles(9) = text(state.ax, headPoint(1), headPoint(2), headPoint(3) + 0.05, ...
    "face center", "Color", [0.55, 0.05, 0.05], "FontWeight", "bold");
handles(10) = text(state.ax, targetPoint(1), targetPoint(2), targetPoint(3) + 0.05, ...
    sprintf("target %.2f m", state.targetDistance), ...
    "Color", targetColor, "FontWeight", "bold");
hold(state.ax, "off")

state.graphicsHandles = handles;
guidata(fig, state);
drawnow limitrate
end

function deleteGraphics(handles)
for i = 1:numel(handles)
    if isgraphics(handles(i))
        delete(handles(i));
    end
end
end

function [targetTform, targetPoint, faceNormal] = buildTargetTform(faceCenter, distance, viewAngles)
faceNormal = faceNormalFromAngles(viewAngles);
targetPoint = faceCenter + distance * faceNormal;

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
end

function faceNormal = faceNormalFromAngles(viewAngles)
yaw = deg2rad(viewAngles(1));
pitch = deg2rad(viewAngles(2));
baseDirection = [-1; 0; 0];
faceNormal = rotzLocal(yaw) * rotyLocal(pitch) * baseDirection;
faceNormal = (faceNormal / norm(faceNormal)).';
end

function matrix = rotzLocal(angle)
c = cos(angle);
s = sin(angle);
matrix = [c, -s, 0; s, c, 0; 0, 0, 1];
end

function matrix = rotyLocal(angle)
c = cos(angle);
s = sin(angle);
matrix = [c, 0, s; 0, 1, 0; -s, 0, c];
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
assignin("base", "faceNormal", state.lastFaceNormal);
assignin("base", "viewAngles", state.viewAngles);
assignin("base", "targetDistance", state.targetDistance);
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
