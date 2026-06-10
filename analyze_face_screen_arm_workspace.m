function results = analyze_face_screen_arm_workspace(sampleCount, showPlot, seed)
%ANALYZE_FACE_SCREEN_ARM_WORKSPACE Estimate reachable and dexterous workspace.
%
% Usage:
%   results = analyze_face_screen_arm_workspace
%   results = analyze_face_screen_arm_workspace(50000, true)
%   results = analyze_face_screen_arm_workspace(100000, false, 1)
%
% Definitions used in this demo:
%   Reachable workspace:
%       All sampled screen_center positions produced by joint values inside
%       the URDF joint limits.
%
%   Dexterous workspace, approximate:
%       A sampled position is treated as dexterous when its configuration is
%       not close to joint limits and the 6D geometric Jacobian is reasonably
%       well conditioned. This is a practical local dexterity definition, not
%       the strict textbook definition of "all orientations are reachable".

if nargin < 1 || isempty(sampleCount)
    sampleCount = 30000;
end
if nargin < 2 || isempty(showPlot)
    showPlot = true;
end
if nargin < 3 || isempty(seed)
    seed = 1;
end

rng(seed, "twister");

projectRoot = fileparts(mfilename("fullpath"));
urdfPath = fullfile(projectRoot, "generated", "urdf", "face_screen_support_arm.urdf");
endEffector = "screen_center";

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

[jointNames, jointTypes, lowerLimits, upperLimits] = movingJointLimits(robot);
dof = numel(lowerLimits);

qSamples = lowerLimits(:) + (upperLimits(:) - lowerLimits(:)) .* rand(dof, sampleCount);

positions = zeros(sampleCount, 3);
linearManipulability = zeros(sampleCount, 1);
fullManipulability = zeros(sampleCount, 1);
linearCondition = zeros(sampleCount, 1);
fullCondition = zeros(sampleCount, 1);
jointLimitMargin = zeros(sampleCount, 1);

charLength = 1.0; % meter; used to normalize mixed angular/linear Jacobian rows

fprintf("Sampling %d configurations...\n", sampleCount);
tic
for i = 1:sampleCount
    q = qSamples(:, i);

    tform = getTransform(robot, q, endEffector);
    positions(i, :) = tform(1:3, 4).';

    jacobian = geometricJacobian(robot, q, endEffector);
    jacobianLinear = jacobian(4:6, :);
    jacobianScaled = [jacobian(1:3, :); jacobianLinear / charLength];

    linearSingularValues = svd(jacobianLinear, "econ");
    fullSingularValues = svd(jacobianScaled, "econ");

    linearManipulability(i) = prod(linearSingularValues);
    fullManipulability(i) = prod(fullSingularValues);
    linearCondition(i) = safeConditionNumber(linearSingularValues);
    fullCondition(i) = safeConditionNumber(fullSingularValues);

    normalizedLowerDistance = (q - lowerLimits(:)) ./ (upperLimits(:) - lowerLimits(:));
    normalizedUpperDistance = (upperLimits(:) - q) ./ (upperLimits(:) - lowerLimits(:));
    jointLimitMargin(i) = min([normalizedLowerDistance; normalizedUpperDistance]);
end
elapsedSeconds = toc;

manipulabilityThreshold = percentileNoToolbox(fullManipulability, 60);
conditionThreshold = 60;
jointLimitMarginThreshold = 0.05;

dexterousMask = ...
    fullManipulability >= manipulabilityThreshold & ...
    fullCondition <= conditionThreshold & ...
    jointLimitMargin >= jointLimitMarginThreshold;

voxelSize = 0.04; % meter
reachableVolume = estimateVoxelVolume(positions, voxelSize);
dexterousVolume = estimateVoxelVolume(positions(dexterousMask, :), voxelSize);

reachableMin = min(positions, [], 1);
reachableMax = max(positions, [], 1);
reachableSpan = reachableMax - reachableMin;

results = struct;
results.robot = robot;
results.endEffector = endEffector;
results.sampleCount = sampleCount;
results.seed = seed;
results.elapsedSeconds = elapsedSeconds;
results.jointNames = jointNames;
results.jointTypes = jointTypes;
results.lowerLimits = lowerLimits;
results.upperLimits = upperLimits;
results.positions = positions;
results.qSamples = qSamples;
results.linearManipulability = linearManipulability;
results.fullManipulability = fullManipulability;
results.linearCondition = linearCondition;
results.fullCondition = fullCondition;
results.jointLimitMargin = jointLimitMargin;
results.dexterousMask = dexterousMask;
results.dexterousPositions = positions(dexterousMask, :);
results.reachableMin = reachableMin;
results.reachableMax = reachableMax;
results.reachableSpan = reachableSpan;
results.voxelSize = voxelSize;
results.reachableVolume = reachableVolume;
results.dexterousVolume = dexterousVolume;
results.dexterousSampleRatio = nnz(dexterousMask) / sampleCount;
results.dexterityConditionThreshold = conditionThreshold;
results.dexterityManipulabilityThreshold = manipulabilityThreshold;
results.dexterityJointLimitMarginThreshold = jointLimitMarginThreshold;

assignin("base", "workspaceResults", results);

printSummary(results);

if showPlot
    plotWorkspace(robot, results);
end
end

function [jointNames, jointTypes, lowerLimits, upperLimits] = movingJointLimits(robot)
jointNames = strings(0, 1);
jointTypes = strings(0, 1);
lowerLimits = zeros(0, 1);
upperLimits = zeros(0, 1);

for bodyIndex = 1:numel(robot.Bodies)
    joint = robot.Bodies{bodyIndex}.Joint;
    if joint.Type == "fixed"
        continue
    end

    limits = joint.PositionLimits;
    if any(~isfinite(limits))
        error("Joint '%s' has non-finite limits. This script needs finite joint limits.", joint.Name);
    end

    jointNames(end + 1, 1) = string(joint.Name); %#ok<AGROW>
    jointTypes(end + 1, 1) = string(joint.Type); %#ok<AGROW>
    lowerLimits(end + 1, 1) = limits(1); %#ok<AGROW>
    upperLimits(end + 1, 1) = limits(2); %#ok<AGROW>
end
end

function conditionNumber = safeConditionNumber(singularValues)
largest = max(singularValues);
smallest = min(singularValues);
if smallest <= eps(largest)
    conditionNumber = inf;
else
    conditionNumber = largest / smallest;
end
end

function value = percentileNoToolbox(values, percentile)
values = sort(values(isfinite(values)));
if isempty(values)
    value = NaN;
    return
end

index = max(1, min(numel(values), round(percentile / 100 * numel(values))));
value = values(index);
end

function volumeValue = estimateVoxelVolume(points, voxelSize)
if isempty(points)
    volumeValue = 0;
    return
end

origin = min(points, [], 1);
voxelIndices = floor((points - origin) / voxelSize);
uniqueVoxels = unique(voxelIndices, "rows");
volumeValue = size(uniqueVoxels, 1) * voxelSize^3;
end

function printSummary(results)
fprintf("\nJoint limits:\n");
for i = 1:numel(results.jointNames)
    if results.jointTypes(i) == "prismatic"
        fprintf("  %-24s [%7.1f, %7.1f] mm\n", ...
            results.jointNames(i), ...
            results.lowerLimits(i) * 1000, ...
            results.upperLimits(i) * 1000);
    else
        fprintf("  %-24s [%7.1f, %7.1f] deg\n", ...
            results.jointNames(i), ...
            rad2deg(results.lowerLimits(i)), ...
            rad2deg(results.upperLimits(i)));
    end
end

fprintf("\nReachable workspace of %s, Monte Carlo estimate:\n", results.endEffector);
fprintf("  Samples:             %d\n", results.sampleCount);
fprintf("  Elapsed:             %.2f s\n", results.elapsedSeconds);
fprintf("  X range:             [%.3f, %.3f] m, span %.3f m\n", ...
    results.reachableMin(1), results.reachableMax(1), results.reachableSpan(1));
fprintf("  Y range:             [%.3f, %.3f] m, span %.3f m\n", ...
    results.reachableMin(2), results.reachableMax(2), results.reachableSpan(2));
fprintf("  Z range:             [%.3f, %.3f] m, span %.3f m\n", ...
    results.reachableMin(3), results.reachableMax(3), results.reachableSpan(3));
fprintf("  Voxel size:          %.3f m\n", results.voxelSize);
fprintf("  Reachable volume:    %.4f m^3\n", results.reachableVolume);

fprintf("\nApproximate dexterous workspace:\n");
fprintf("  Full Jacobian cond <= %.1f\n", results.dexterityConditionThreshold);
fprintf("  Full manipulability >= %.4g, relative 60th percentile threshold\n", ...
    results.dexterityManipulabilityThreshold);
fprintf("  Joint limit margin >= %.1f %% of each joint range\n", ...
    results.dexterityJointLimitMarginThreshold * 100);
fprintf("  Dexterous samples:   %d / %d, %.2f %%\n", ...
    nnz(results.dexterousMask), results.sampleCount, results.dexterousSampleRatio * 100);
fprintf("  Dexterous volume:    %.4f m^3\n", results.dexterousVolume);
fprintf("  Volume ratio:        %.2f %% of reachable voxel volume\n\n", ...
    100 * results.dexterousVolume / max(results.reachableVolume, eps));
fprintf("Results saved to base workspace as workspaceResults.\n");
end

function plotWorkspace(robot, results)
figure( ...
    "Name", "Face Screen Support Arm Workspace", ...
    "NumberTitle", "off", ...
    "Color", "w");

reachablePositions = results.positions;
dexterousPositions = results.dexterousPositions;

maxReachablePlotCount = 20000;
if size(reachablePositions, 1) > maxReachablePlotCount
    plotIndices = round(linspace(1, size(reachablePositions, 1), maxReachablePlotCount));
    reachablePositions = reachablePositions(plotIndices, :);
end

scatter3( ...
    reachablePositions(:, 1), ...
    reachablePositions(:, 2), ...
    reachablePositions(:, 3), ...
    4, [0.70, 0.70, 0.70], "filled");
hold on

if ~isempty(dexterousPositions)
    scatter3( ...
        dexterousPositions(:, 1), ...
        dexterousPositions(:, 2), ...
        dexterousPositions(:, 3), ...
        8, [0.05, 0.35, 0.90], "filled");
end

qHome = zeros(numel(results.lowerLimits), 1);
qHome(2) = deg2rad(-120);
qHome(3) = deg2rad(120);
qHome(4) = 0.03;
show(robot, qHome, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "PreservePlot", true);

axis equal
grid on
view(135, 25)
xlabel("X / m")
ylabel("Y / m")
zlabel("Z / m")
title("Reachable workspace and approximate dexterous workspace")
legend(["Reachable samples", "Dexterous samples", "Robot home pose"], "Location", "best")
hold off
end
