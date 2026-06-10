function demo_face_screen_arm_target_pose(workspaceMode)
%DEMO_FACE_SCREEN_ARM_TARGET_POSE Interactive target-pose IK demo.
%
% Usage:
%   demo_face_screen_arm_target_pose
%   demo_face_screen_arm_target_pose("wide")
%
% The right panel controls the target pose of screen_center. The target pose
% is shown as a 3D frame in the simulation view. Move the camera with the
% mouse at any time; redraws preserve the current camera view.

if nargin < 1 || strlength(string(workspaceMode)) == 0
    workspaceMode = "large";
else
    workspaceMode = lower(string(workspaceMode));
end

projectRoot = fileparts(mfilename("fullpath"));
urdfPath = fullfile(projectRoot, "generated", "urdf", "face_screen_support_arm.urdf");

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

ik = inverseKinematics("RigidBodyTree", robot);
ik.SolverParameters.MaxIterations = 1500;
ik.SolverParameters.MaxTime = 2.0;

fig = figure( ...
    "Name", "Face Screen Support Arm Target Pose Demo", ...
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
    "Title", "Target Pose", ...
    "Units", "normalized", ...
    "Position", [0.70, 0.08, 0.27, 0.86], ...
    "BackgroundColor", "w");

state = struct;
state.robot = robot;
state.ik = ik;
state.weights = [0.5, 0.5, 0.5, 1, 1, 1];
state.endEffector = "screen_center";
state.positionTolerance = 0.03;
state.orientationTolerance = deg2rad(12);
state.workspaceMode = workspaceMode;
state.ax = ax;
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
state.targetValues = [0.45, 0.00, 0.50, 0, 0, 0];
state.targetDefs = createTargetDefinitions(workspaceMode);
state.sliders = gobjects(6, 1);
state.edits = gobjects(6, 1);
state.valueTexts = gobjects(6, 1);
state.listeners = cell(6, 1);
state.statusText = gobjects(1);
state.targetHandles = gobjects(0);
state.targetReachable = true;
state.isUpdatingUi = false;

guidata(fig, state);
createControls(fig, panel);
redrawRobot(fig);
updateTargetGraphics(fig);
setStatus(fig, "Adjust target pose, then click Move to Target.", [0.10, 0.10, 0.10]);

assignin("base", "robot", robot);
assignin("base", "q", state.q);
assignin("base", "targetPoseValues", state.targetValues);
assignin("base", "targetPoseFigure", fig);

fprintf("\nTarget-pose slider demo started.\n");
fprintf("Target vector: [x y z yaw pitch roll], meters and degrees.\n");
fprintf("Use the figure toolbar or mouse to rotate, pan, and zoom the camera.\n\n");
end

function defs = createTargetDefinitions(workspaceMode)
limits = workspaceLimits(workspaceMode);
defs = struct( ...
    "label", { ...
        "X position", ...
        "Y position", ...
        "Z position", ...
        "Yaw about Z", ...
        "Pitch about Y", ...
        "Roll about X"}, ...
    "unit", {"m", "m", "m", "deg", "deg", "deg"}, ...
    "min", {limits(1, 1), limits(2, 1), limits(3, 1), -180, -90, -180}, ...
    "max", {limits(1, 2), limits(2, 2), limits(3, 2), 180, 90, 180}, ...
    "smallStep", {0.01, 0.01, 0.01, 1, 1, 1}, ...
    "bigStep", {0.10, 0.10, 0.10, 10, 10, 10});
end

function createControls(fig, panel)
state = guidata(fig);

uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "Target frame: red X, green Y, blue Z", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.945, 0.88, 0.035], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

top = 0.865;
rowHeight = 0.118;

for i = 1:6
    y = top - (i - 1) * rowHeight;
    def = state.targetDefs(i);
    value = state.targetValues(i);

    uicontrol( ...
        "Parent", panel, ...
        "Style", "text", ...
        "String", sprintf("%s / %s", def.label, def.unit), ...
        "Units", "normalized", ...
        "Position", [0.06, y + 0.053, 0.56, 0.035], ...
        "HorizontalAlignment", "left", ...
        "BackgroundColor", "w");

    state.valueTexts(i) = uicontrol( ...
        "Parent", panel, ...
        "Style", "text", ...
        "String", formatTargetValue(value, def.unit), ...
        "Units", "normalized", ...
        "Position", [0.62, y + 0.053, 0.32, 0.035], ...
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
        "Position", [0.06, y + 0.020, 0.60, 0.035], ...
        "Callback", @(src, ~) targetSliderChanged(fig, src));

    state.listeners{i} = addlistener( ...
        state.sliders(i), ...
        "Value", ...
        "PostSet", ...
        @(~, event) targetSliderChanged(fig, event.AffectedObject));

    state.edits(i) = uicontrol( ...
        "Parent", panel, ...
        "Style", "edit", ...
        "String", numericEditString(value, def.unit), ...
        "Units", "normalized", ...
        "Position", [0.69, y + 0.014, 0.25, 0.047], ...
        "HorizontalAlignment", "left", ...
        "BackgroundColor", "white", ...
        "Callback", @(src, ~) targetEditChanged(fig, src));
end

state.statusText = uicontrol( ...
    "Parent", panel, ...
    "Style", "text", ...
    "String", "", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.150, 0.88, 0.065], ...
    "HorizontalAlignment", "left", ...
    "BackgroundColor", "w");

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Move to Target", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.095, 0.88, 0.045], ...
    "Callback", @(~, ~) moveToTarget(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Reset Home", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.040, 0.27, 0.040], ...
    "Callback", @(~, ~) resetHome(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Export", ...
    "Units", "normalized", ...
    "Position", [0.365, 0.040, 0.27, 0.040], ...
    "Callback", @(~, ~) exportState(fig));
uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Close", ...
    "Units", "normalized", ...
    "Position", [0.67, 0.040, 0.27, 0.040], ...
    "Callback", @(~, ~) close(fig));

guidata(fig, state);
end

function step = normalizedSliderStep(def)
range = def.max - def.min;
step = [def.smallStep / range, def.bigStep / range];
step = min(max(step, 0.0001), 1);
end

function targetSliderChanged(fig, slider)
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

state.targetValues(index) = slider.Value;
state = updateTargetUiValue(state, index);
guidata(fig, state);
updateTargetGraphics(fig);
assignin("base", "targetPoseValues", state.targetValues);
end

function targetEditChanged(fig, editBox)
state = guidata(fig);
index = find(state.edits == editBox, 1);
if isempty(index)
    return
end

value = str2double(editBox.String);
if isnan(value)
    state = updateTargetUiValue(state, index);
    guidata(fig, state);
    return
end

def = state.targetDefs(index);
value = min(max(value, def.min), def.max);
state.targetValues(index) = value;
state.isUpdatingUi = true;
state.sliders(index).Value = value;
state.isUpdatingUi = false;
state = updateTargetUiValue(state, index);
guidata(fig, state);
updateTargetGraphics(fig);
assignin("base", "targetPoseValues", state.targetValues);
end

function state = updateTargetUiValue(state, index)
def = state.targetDefs(index);
value = state.targetValues(index);
state.valueTexts(index).String = formatTargetValue(value, def.unit);
state.edits(index).String = numericEditString(value, def.unit);
end

function moveToTarget(fig)
state = guidata(fig);
targetTform = poseVectorToTform(state.targetValues);
[qSolution, solutionInfo] = state.ik(state.endEffector, targetTform, state.weights, state.q);
[positionError, orientationError] = poseError(state.robot, qSolution, state.endEffector, targetTform);

reachable = positionError <= state.positionTolerance && orientationError <= state.orientationTolerance;
state.targetReachable = reachable;
guidata(fig, state);
updateTargetGraphics(fig);

fprintf("IK status: %s\n", string(solutionInfo.Status));
fprintf("Position error: %.4f m, orientation error: %.2f deg\n", ...
    positionError, rad2deg(orientationError));

if reachable
    setStatus(fig, sprintf("Reachable. Moving. Error %.1f mm, %.1f deg.", ...
        positionError * 1000, rad2deg(orientationError)), [0.05, 0.35, 0.12]);
    qNew = animateToConfiguration(fig, state.q, qSolution);
    state = guidata(fig);
    state.q = qNew;
    state.targetReachable = true;
    guidata(fig, state);
    assignin("base", "q", qNew);
    setStatus(fig, "Arrived. Adjust sliders for the next target.", [0.05, 0.35, 0.12]);
else
    setStatus(fig, sprintf("Not reachable. Error %.1f mm, %.1f deg.", ...
        positionError * 1000, rad2deg(orientationError)), [0.70, 0.05, 0.05]);
end
end

function resetHome(fig)
state = guidata(fig);
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
guidata(fig, state);
redrawRobot(fig);
updateTargetGraphics(fig);
assignin("base", "q", state.q);
setStatus(fig, "Robot reset to home pose.", [0.10, 0.10, 0.10]);
end

function exportState(fig)
state = guidata(fig);
assignin("base", "robot", state.robot);
assignin("base", "q", state.q);
assignin("base", "targetPoseValues", state.targetValues);
fprintf("Exported robot, q, and targetPoseValues to base workspace.\n");
fprintf("targetPoseValues = [%.3f %.3f %.3f %.1f %.1f %.1f]\n", state.targetValues);
end

function setStatus(fig, message, color)
state = guidata(fig);
if isgraphics(state.statusText)
    state.statusText.String = message;
    state.statusText.ForegroundColor = color;
end
end

function q = animateToConfiguration(fig, qStart, qGoal)
frameCount = 80;
secondsPerFrame = 0.025;

for frameIndex = 1:frameCount
    if ~isvalid(fig)
        q = qStart;
        return
    end

    t = frameIndex / frameCount;
    s = 3 * t^2 - 2 * t^3;
    q = qStart + (qGoal - qStart) * s;

    state = guidata(fig);
    state.q = q;
    guidata(fig, state);
    redrawRobot(fig);
    updateTargetGraphics(fig);
    drawnow
    pause(secondsPerFrame)
end

q = qGoal;
end

function redrawRobot(fig)
state = guidata(fig);
cameraState = captureCamera(state.ax);

show(state.robot, state.q, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "Parent", state.ax, ...
    "PreservePlot", false, ...
    "FastUpdate", true);

setupAxes(state.ax, state.workspaceMode);
restoreCamera(state.ax, cameraState);
title(state.ax, "Target-pose IK control")
drawnow limitrate
end

function updateTargetGraphics(fig)
state = guidata(fig);
deleteGraphics(state.targetHandles);

tform = poseVectorToTform(state.targetValues);
origin = tform(1:3, 4).';
rotation = tform(1:3, 1:3);
axisLength = 0.14;

if state.targetReachable
    markerColor = [0.05, 0.35, 0.90];
else
    markerColor = [0.85, 0.10, 0.10];
end

hold(state.ax, "on")
handles = gobjects(5, 1);
handles(1) = plot3(state.ax, origin(1), origin(2), origin(3), ...
    "o", "MarkerSize", 8, "MarkerFaceColor", markerColor, "MarkerEdgeColor", "k");
handles(2) = quiver3(state.ax, origin(1), origin(2), origin(3), ...
    rotation(1, 1) * axisLength, rotation(2, 1) * axisLength, rotation(3, 1) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.85, 0.10, 0.10], "MaxHeadSize", 0.8);
handles(3) = quiver3(state.ax, origin(1), origin(2), origin(3), ...
    rotation(1, 2) * axisLength, rotation(2, 2) * axisLength, rotation(3, 2) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.55, 0.16], "MaxHeadSize", 0.8);
handles(4) = quiver3(state.ax, origin(1), origin(2), origin(3), ...
    rotation(1, 3) * axisLength, rotation(2, 3) * axisLength, rotation(3, 3) * axisLength, ...
    0, "LineWidth", 2.0, "Color", [0.05, 0.25, 0.90], "MaxHeadSize", 0.8);
handles(5) = text(state.ax, origin(1), origin(2), origin(3) + 0.05, ...
    "target", "Color", markerColor, "FontWeight", "bold");
hold(state.ax, "off")

state.targetHandles = handles;
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

function q = displayPoseToConfig(displayValues)
q = zeros(1, 6);
q(1) = deg2rad(displayValues(1));
q(2) = deg2rad(displayValues(2));
q(3) = deg2rad(displayValues(3));
q(4) = displayValues(4) / 1000;
q(5) = deg2rad(displayValues(5));
q(6) = deg2rad(displayValues(6));
end

function tform = poseVectorToTform(targetValues)
translation = targetValues(1:3);
yawPitchRoll = deg2rad(targetValues(4:6));
tform = trvec2tform(translation) * eul2tform(yawPitchRoll, "ZYX");
end

function [positionError, orientationError] = poseError(robot, q, endEffector, targetTform)
actualTform = getTransform(robot, q, endEffector);
positionError = norm(actualTform(1:3, 4) - targetTform(1:3, 4));

rotationError = targetTform(1:3, 1:3).' * actualTform(1:3, 1:3);
axisAngle = rotm2axang(rotationError);
orientationError = abs(axisAngle(4));
end

function cameraState = captureCamera(ax)
cameraState = struct( ...
    "View", ax.View, ...
    "CameraPosition", ax.CameraPosition, ...
    "CameraTarget", ax.CameraTarget, ...
    "CameraUpVector", ax.CameraUpVector, ...
    "CameraViewAngle", ax.CameraViewAngle, ...
    "Projection", ax.Projection);
end

function restoreCamera(ax, cameraState)
ax.View = cameraState.View;
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
        limits = [-0.75, 0.75; -0.75, 0.75; -0.10, 0.75];
    case "large"
        limits = [-1.50, 1.50; -1.50, 1.50; -0.10, 1.25];
    case "wide"
        limits = [-2.50, 2.50; -2.50, 2.50; -0.10, 1.75];
    otherwise
        error("Unknown workspace mode '%s'. Use normal, large, or wide.", workspaceMode);
end
end

function textValue = formatTargetValue(value, unit)
if unit == "m"
    textValue = sprintf("% .3f m", value);
else
    textValue = sprintf("% .1f deg", value);
end
end

function textValue = numericEditString(value, unit)
if unit == "m"
    textValue = sprintf("%.3f", value);
else
    textValue = sprintf("%.1f", value);
end
end
