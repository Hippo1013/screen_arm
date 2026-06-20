function result = demo_face_view_target_ik_trajectory_curves(yawDeg, pitchDeg, nominalDistance)
%DEMO_FACE_VIEW_TARGET_IK_TRAJECTORY_CURVES Generate IK and joint trajectory plots.
%
% Usage:
%   result = demo_face_view_target_ik_trajectory_curves
%   result = demo_face_view_target_ik_trajectory_curves(18, -8, 0.45)
%
% Without input arguments, this script opens the original simulation UI and
% adds one button for exporting joint trajectory curves. With numeric input
% arguments, it directly computes and exports the curves for the specified
% yaw, pitch, and nominal screen distance.

if nargin == 0
    result = launchInteractiveCurveExporter();
    return
end

if isempty(yawDeg)
    yawDeg = 18;
end
if nargin < 2 || isempty(pitchDeg)
    pitchDeg = -8;
end
if nargin < 3 || isempty(nominalDistance)
    nominalDistance = 0.45;
end

projectRoot = fileparts(fileparts(mfilename("fullpath")));
urdfPath = fullfile(projectRoot, "screen_arm", "generated", "urdf", ...
    "face_screen_support_arm_depth_camera.urdf");
figureDir = fullfile(projectRoot, "test", "result");
if ~isfolder(figureDir)
    mkdir(figureDir);
end

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

state = struct;
state.robot = robot;
state.ik = inverseKinematics("RigidBodyTree", robot);
state.ik.SolverParameters.MaxIterations = 1500;
state.ik.SolverParameters.MaxTime = 1.5;
state.endEffector = "screen_center";
state.weights = [0.7, 0.7, 0.7, 1, 1, 1];
state.jointInfo = movingJointInfo(robot);
state.q = displayPoseToConfig([0, -120, 120, 30, 0, 0]).';
state.faceCenter = [0.65, 0.00, 1.00];
state.viewAngles = [yawDeg, pitchDeg];
state.viewDistance = nominalDistance;
state.distanceRange = [0.30, 0.60];
state.positionTolerance = 0.025;
state.normalTolerance = deg2rad(8);

solveResult = solveTargetWithDistanceFallback(state);
[time, qTrajectory] = planSmoothJointTrajectory(state.q, solveResult.q);

plotFig = plotJointTrajectories(time, qTrajectory, state, solveResult);

combinedPath = fullfile(figureDir, "逆运动学关节轨迹曲线.png");
exportgraphics(plotFig, combinedPath, "Resolution", 300);
exportJointCurveImages(time, qTrajectory, state, solveResult, figureDir);
exportCurveResultSummary(time, qTrajectory, state, solveResult, figureDir);

targetPoint = solveResult.targetTform(1:3, 4).';
fprintf("\nIK trajectory curve demo finished.\n");
fprintf("Face view yaw/pitch: [%.1f %.1f] deg\n", yawDeg, pitchDeg);
fprintf("Target distance used: %.3f m\n", solveResult.distance);
fprintf("Target screen center: [%.3f %.3f %.3f] m\n", targetPoint);
fprintf("Reachable flag: %d, IK status: %s\n", solveResult.reachable, solveResult.status);
fprintf("Position error: %.4f m, normal error: %.2f deg\n", ...
    solveResult.positionError, rad2deg(solveResult.normalError));
if strlength(string(combinedPath)) > 0
    fprintf("Saved combined figure: %s\n", combinedPath);
end

result = struct( ...
    "robot", robot, ...
    "qStart", state.q, ...
    "qGoal", solveResult.q, ...
    "time", time, ...
    "qTrajectory", qTrajectory, ...
    "targetTform", solveResult.targetTform, ...
    "targetPoint", targetPoint, ...
    "targetDistance", solveResult.distance, ...
    "reachable", solveResult.reachable, ...
    "positionError", solveResult.positionError, ...
    "normalError", solveResult.normalError, ...
    "figure", plotFig, ...
    "combinedFigurePath", string(combinedPath));

assignin("base", "ikTrajectoryCurveResult", result);
assignin("base", "robot", robot);
assignin("base", "q", solveResult.q);
end

function result = launchInteractiveCurveExporter()
fig = [];
try
    fig = evalin("base", "faceViewFigure");
catch
end

if isempty(fig) || ~isvalid(fig)
    demo_face_view_target_ik_trajectory;
    fig = evalin("base", "faceViewFigure");
end

panel = findall(fig, "Type", "uipanel", "Title", "Face Direction");
if isempty(panel)
    error("Cannot find the Face Direction panel in the simulation UI.");
end
panel = panel(1);

state = guidata(fig);
if isfield(state, "statusText") && isgraphics(state.statusText)
    state.statusText.Position = [0.06, 0.280, 0.88, 0.085];
    state.statusText.String = sprintf("%s\nSet yaw/pitch, then click Plot Joint Curves.", ...
        string(state.statusText.String));
end
guidata(fig, state);

oldButtons = findall(panel, "Tag", "JointTrajectoryCurveButton");
delete(oldButtons(isgraphics(oldButtons)));

uicontrol( ...
    "Parent", panel, ...
    "Style", "pushbutton", ...
    "String", "Plot Joint Curves", ...
    "Units", "normalized", ...
    "Position", [0.06, 0.225, 0.88, 0.045], ...
    "Tag", "JointTrajectoryCurveButton", ...
    "Callback", @(~, ~) exportCurvesFromInteractiveFigure(fig));

figure(fig);
fprintf("\nInteractive IK trajectory curve exporter attached.\n");
fprintf("Use the simulation UI to set yaw/pitch, then click Plot Joint Curves.\n");
fprintf("Figures will be saved under test\\result.\n\n");

result = struct( ...
    "figure", fig, ...
    "message", "Interactive exporter attached to faceViewFigure");
assignin("base", "ikTrajectoryCurveExporter", result);
end

function result = exportCurvesFromInteractiveFigure(fig)
if isempty(fig) || ~isvalid(fig)
    return
end

state = guidata(fig);
projectRoot = fileparts(fileparts(mfilename("fullpath")));
figureDir = fullfile(projectRoot, "test", "result");
if ~isfolder(figureDir)
    mkdir(figureDir);
end

solveResult = solveTargetWithDistanceFallback(state);
[time, qTrajectory] = planSmoothJointTrajectory(state.q, solveResult.q);
plotFig = plotJointTrajectories(time, qTrajectory, state, solveResult);

combinedPath = fullfile(figureDir, "逆运动学关节轨迹曲线.png");
exportgraphics(plotFig, combinedPath, "Resolution", 300);
exportJointCurveImages(time, qTrajectory, state, solveResult, figureDir);
exportCurveResultSummary(time, qTrajectory, state, solveResult, figureDir);

targetPoint = solveResult.targetTform(1:3, 4).';
fprintf("\nInteractive IK trajectory curves exported.\n");
fprintf("Face view yaw/pitch: [%.1f %.1f] deg\n", state.viewAngles(1), state.viewAngles(2));
fprintf("Target distance used: %.3f m\n", solveResult.distance);
fprintf("Target screen center: [%.3f %.3f %.3f] m\n", targetPoint);
fprintf("Reachable flag: %d, IK status: %s\n", solveResult.reachable, solveResult.status);
fprintf("Position error: %.4f m, normal error: %.2f deg\n", ...
    solveResult.positionError, rad2deg(solveResult.normalError));
fprintf("Saved combined figure: %s\n", combinedPath);

if isfield(state, "statusText") && isgraphics(state.statusText)
    if solveResult.reachable
        state.statusText.String = sprintf( ...
            "Joint curves exported.\nd %.3f m, pos %.1f mm, normal %.1f deg.", ...
            solveResult.distance, solveResult.positionError * 1000, ...
            rad2deg(solveResult.normalError));
        state.statusText.ForegroundColor = [0.05, 0.35, 0.12];
    else
        state.statusText.String = sprintf( ...
            "Best curves exported, but target is not reachable.\nd %.3f m, pos %.1f mm, normal %.1f deg.", ...
            solveResult.distance, solveResult.positionError * 1000, ...
            rad2deg(solveResult.normalError));
        state.statusText.ForegroundColor = [0.70, 0.05, 0.05];
    end
    guidata(fig, state);
end

result = struct( ...
    "robot", state.robot, ...
    "qStart", state.q, ...
    "qGoal", solveResult.q, ...
    "time", time, ...
    "qTrajectory", qTrajectory, ...
    "targetTform", solveResult.targetTform, ...
    "targetPoint", targetPoint, ...
    "targetDistance", solveResult.distance, ...
    "reachable", solveResult.reachable, ...
    "positionError", solveResult.positionError, ...
    "normalError", solveResult.normalError, ...
    "figure", plotFig, ...
    "combinedFigurePath", string(combinedPath));

assignin("base", "ikTrajectoryCurveResult", result);
end

function result = solveTargetWithDistanceFallback(state)
distances = candidateDistances(state.distanceRange, state.viewDistance);
result = solveSingleDistance(state, distances(1));
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
[targetTform, faceCenter] = buildStateTargetTform(state, distance);
[qSolution, solutionInfo] = state.ik( ...
    state.endEffector, targetTform, state.weights, state.q);
[positionError, fullOrientationError, normalError] = poseErrors( ...
    state.robot, qSolution, state.endEffector, targetTform);

actualTform = getTransform(state.robot, qSolution, state.endEffector);
actualDistance = norm(actualTform(1:3, 4).' - faceCenter);
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
fallbackDistances = linspace(distanceRange(1), distanceRange(2), 16);
fallbackDistances(abs(fallbackDistances - nominalDistance) < 1e-9) = [];
[~, order] = sort(abs(fallbackDistances - nominalDistance));
distances = [nominalDistance, fallbackDistances(order)];
end

function [time, qTrajectory] = planSmoothJointTrajectory(qStart, qGoal)
motionTime = 2.0;
sampleCount = 121;
time = linspace(0, motionTime, sampleCount);
tau = time / motionTime;
s = 3 * tau.^2 - 2 * tau.^3;
qTrajectory = qStart(:) + (qGoal(:) - qStart(:)) * s;
end

function fig = plotJointTrajectories(time, qTrajectory, state, solveResult)
fig = figure( ...
    "Name", "IK Joint Trajectory Curves", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "Units", "pixels", ...
    "Position", [80, 80, 1280, 760]);

layout = tiledlayout(fig, 2, 3, ...
    "TileSpacing", "compact", ...
    "Padding", "compact");

for jointIndex = 1:6
    ax = nexttile(layout, jointIndex);
    [values, unitText, variableText] = jointTrajectoryDisplayValues( ...
        qTrajectory(jointIndex, :), state.jointInfo.types(jointIndex), jointIndex);

    plot(ax, time, values, "LineWidth", 2.0, "Color", [0.10, 0.35, 0.85]);
    hold(ax, "on")
    plot(ax, time(1), values(1), "o", ...
        "MarkerFaceColor", [0.10, 0.55, 0.20], "MarkerEdgeColor", "none");
    plot(ax, time(end), values(end), "s", ...
        "MarkerFaceColor", [0.85, 0.20, 0.15], "MarkerEdgeColor", "none");
    grid(ax, "on")
    box(ax, "on")
    xlabel(ax, "t / s");
    ylabel(ax, unitText);
    title(ax, sprintf("Joint %d: %s", jointIndex, variableText), ...
        "FontWeight", "bold");
end

title(layout, sprintf( ...
    "IK solution and smooth joint-space trajectory, yaw %.1f deg, pitch %.1f deg, distance %.2f m", ...
    state.viewAngles(1), state.viewAngles(2), solveResult.distance), ...
    "FontWeight", "bold");
end

function exportJointCurveImages(time, qTrajectory, state, solveResult, figureDir)
for jointIndex = 1:6
    fig = figure( ...
        "Name", sprintf("Joint %d Trajectory", jointIndex), ...
        "NumberTitle", "off", ...
        "Color", "w", ...
        "Visible", "off", ...
        "Units", "pixels", ...
        "Position", [80, 80, 720, 420]);
    ax = axes("Parent", fig);

    [values, unitText, variableText] = jointTrajectoryDisplayValues( ...
        qTrajectory(jointIndex, :), state.jointInfo.types(jointIndex), jointIndex);
    plot(ax, time, values, "LineWidth", 2.2, "Color", [0.10, 0.35, 0.85]);
    hold(ax, "on")
    plot(ax, time(1), values(1), "o", ...
        "MarkerFaceColor", [0.10, 0.55, 0.20], "MarkerEdgeColor", "none");
    plot(ax, time(end), values(end), "s", ...
        "MarkerFaceColor", [0.85, 0.20, 0.15], "MarkerEdgeColor", "none");
    grid(ax, "on")
    box(ax, "on")
    xlabel(ax, "t / s");
    ylabel(ax, unitText);
    title(ax, sprintf("Joint %d: %s, target distance %.2f m", ...
        jointIndex, variableText, solveResult.distance), "FontWeight", "bold");

    exportgraphics(fig, fullfile(figureDir, ...
        sprintf("逆运动学关节%d轨迹.png", jointIndex)), "Resolution", 300);
    close(fig);
end
end

function exportCurveResultSummary(time, qTrajectory, state, solveResult, figureDir)
targetPoint = solveResult.targetTform(1:3, 4).';
qStartDisplay = jointDisplayVector(qTrajectory(:, 1), state.jointInfo.types);
qGoalDisplay = jointDisplayVector(qTrajectory(:, end), state.jointInfo.types);

summaryPath = fullfile(figureDir, "逆运动学关节轨迹结果.txt");
fid = fopen(summaryPath, "w");
if fid < 0
    return
end

cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "Face view yaw/pitch: [%.3f %.3f] deg\n", ...
    state.viewAngles(1), state.viewAngles(2));
fprintf(fid, "Target distance used: %.6f m\n", solveResult.distance);
fprintf(fid, "Target screen center: [%.6f %.6f %.6f] m\n", targetPoint);
fprintf(fid, "Reachable flag: %d\n", solveResult.reachable);
fprintf(fid, "IK status: %s\n", string(solveResult.status));
fprintf(fid, "Position error: %.6f m\n", solveResult.positionError);
fprintf(fid, "Normal error: %.6f deg\n", rad2deg(solveResult.normalError));
fprintf(fid, "Full orientation error: %.6f deg\n", rad2deg(solveResult.fullOrientationError));
fprintf(fid, "q start display [theta1 theta2 theta3 d4_mm theta5 theta6]: [");
fprintf(fid, "%.6f ", qStartDisplay(1:end - 1));
fprintf(fid, "%.6f]\n", qStartDisplay(end));
fprintf(fid, "q goal display [theta1 theta2 theta3 d4_mm theta5 theta6]: [");
fprintf(fid, "%.6f ", qGoalDisplay(1:end - 1));
fprintf(fid, "%.6f]\n", qGoalDisplay(end));
fprintf(fid, "Trajectory duration: %.6f s\n", time(end) - time(1));
fprintf(fid, "Sample count: %d\n", numel(time));
end

function qDisplay = jointDisplayVector(q, jointTypes)
qDisplay = zeros(1, numel(q));
for i = 1:numel(q)
    if jointTypes(i) == "prismatic"
        qDisplay(i) = q(i) * 1000;
    else
        qDisplay(i) = rad2deg(q(i));
    end
end
end

function [values, unitText, variableText] = jointTrajectoryDisplayValues(qValues, jointType, jointIndex)
if jointType == "prismatic"
    values = qValues * 1000;
    unitText = "d / mm";
    variableText = sprintf("d_%d", jointIndex);
else
    values = rad2deg(qValues);
    unitText = "\theta / deg";
    variableText = sprintf("\\theta_%d", jointIndex);
end
end

function [targetTform, faceCenter] = buildStateTargetTform(state, distance)
[faceCenter, faceNormal] = currentFacePoseForCurveExport(state);
[targetTform, ~] = buildTargetTformFromPose(faceCenter, distance, faceNormal);
end

function [faceCenter, faceNormal] = currentFacePoseForCurveExport(state)
faceNormal = faceNormalFromAngles(state.viewAngles);

if isfield(state, "headPivot") && isfield(state, "steveHeadSize")
    faceCenter = state.headPivot + 0.50 * state.steveHeadSize * faceNormal;
else
    faceCenter = state.faceCenter;
end
end

function [targetTform, targetPoint] = buildTargetTformFromPose(faceCenter, distance, faceNormal)
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

function q = displayPoseToConfig(displayValues)
q = zeros(1, 6);
q(1) = deg2rad(displayValues(1));
q(2) = deg2rad(displayValues(2));
q(3) = deg2rad(displayValues(3));
q(4) = displayValues(4) / 1000;
q(5) = deg2rad(displayValues(5));
q(6) = deg2rad(displayValues(6));
end
