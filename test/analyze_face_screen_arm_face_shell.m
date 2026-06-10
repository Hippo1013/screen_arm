function results = analyze_face_screen_arm_face_shell(center, radius, thickness, sampleCount)
%ANALYZE_FACE_SCREEN_ARM_FACE_SHELL Check a face-centered hemispherical shell.
%
% Usage:
%   results = analyze_face_screen_arm_face_shell
%   results = analyze_face_screen_arm_face_shell([0.7 0 0.5], 0.45, 0.08, 900)
%
% The shell opens toward +X, so target points are on the -X side of center:
%   x <= center(1)
%
% Two checks are reported:
%   1) position-only reachability of screen_center
%   2) pose reachability, requiring screen_center local +X to point at center

if nargin < 1 || isempty(center)
    center = [0.7, 0, 0.5];
end
if nargin < 2 || isempty(radius)
    radius = 0.45;
end
if nargin < 3 || isempty(thickness)
    thickness = 0.08;
end
if nargin < 4 || isempty(sampleCount)
    sampleCount = 900;
end

projectRoot = fileparts(fileparts(mfilename("fullpath")));
urdfPath = fullfile(projectRoot, "generated", "urdf", "face_screen_support_arm.urdf");

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

endEffector = "screen_center";
qHome = zeros(6, 1);
qHome(2) = deg2rad(-120);
qHome(3) = deg2rad(120);
qHome(4) = 0.03;

points = sampleHemisphereShell(center, radius, thickness, sampleCount);

ik = inverseKinematics("RigidBodyTree", robot);
ik.SolverParameters.MaxIterations = 800;
ik.SolverParameters.MaxTime = 0.35;

positionTolerance = 0.03;
orientationTolerance = deg2rad(12);

positionErrors = zeros(size(points, 1), 1);
posePositionErrors = zeros(size(points, 1), 1);
poseOrientationErrors = zeros(size(points, 1), 1);

positionReachable = false(size(points, 1), 1);
poseReachable = false(size(points, 1), 1);

qSeedPosition = qHome;
qSeedPose = qHome;

for i = 1:size(points, 1)
    targetPoint = points(i, :);

    positionTarget = trvec2tform(targetPoint);
    [qPosition, ~] = ik(endEffector, positionTarget, [0 0 0 1 1 1], qSeedPosition);
    actualPositionTform = getTransform(robot, qPosition, endEffector);
    positionErrors(i) = norm(actualPositionTform(1:3, 4).' - targetPoint);
    positionReachable(i) = positionErrors(i) <= positionTolerance;
    if positionReachable(i)
        qSeedPosition = qPosition;
    end

    poseTarget = facePointingTform(targetPoint, center);
    [qPose, ~] = ik(endEffector, poseTarget, [0.7 0.7 0.7 1 1 1], qSeedPose);
    actualPoseTform = getTransform(robot, qPose, endEffector);
    posePositionErrors(i) = norm(actualPoseTform(1:3, 4).' - targetPoint);
    rotationError = poseTarget(1:3, 1:3).' * actualPoseTform(1:3, 1:3);
    axisAngle = rotm2axang(rotationError);
    poseOrientationErrors(i) = abs(axisAngle(4));
    poseReachable(i) = ...
        posePositionErrors(i) <= positionTolerance && ...
        poseOrientationErrors(i) <= orientationTolerance;
    if poseReachable(i)
        qSeedPose = qPose;
    end
end

distancesToCenter = vecnorm(points - center, 2, 2);
radialShortfall = max(0, positionErrors - positionTolerance);
poseShortfall = max(0, posePositionErrors - positionTolerance);

results = struct;
results.center = center;
results.radius = radius;
results.thickness = thickness;
results.sampleCount = size(points, 1);
results.points = points;
results.endEffector = endEffector;
results.positionTolerance = positionTolerance;
results.orientationTolerance = orientationTolerance;
results.positionReachable = positionReachable;
results.poseReachable = poseReachable;
results.positionErrors = positionErrors;
results.posePositionErrors = posePositionErrors;
results.poseOrientationErrors = poseOrientationErrors;
results.positionCoverage = nnz(positionReachable) / numel(positionReachable);
results.poseCoverage = nnz(poseReachable) / numel(poseReachable);
results.maxPositionError = max(positionErrors);
results.maxPosePositionError = max(posePositionErrors);
results.maxPoseOrientationError = max(poseOrientationErrors);
results.meanUnreachablePositionShortfall = mean(radialShortfall(~positionReachable), "omitnan");
results.maxUnreachablePositionShortfall = max(radialShortfall(~positionReachable), [], "omitnan");
results.meanUnreachablePoseShortfall = mean(poseShortfall(~poseReachable), "omitnan");
results.maxUnreachablePoseShortfall = max(poseShortfall(~poseReachable), [], "omitnan");
results.distanceRange = [min(distancesToCenter), max(distancesToCenter)];

assignin("base", "faceShellResults", results);
printFaceShellSummary(results);
plotFaceShell(robot, qHome, results);
end

function points = sampleHemisphereShell(center, radius, thickness, sampleCount)
radialCount = max(3, round(sampleCount ^ (1 / 3)));
angularCount = max(12, round(sqrt(sampleCount / radialCount)));
azimuthCount = 2 * angularCount;

radii = linspace(radius - thickness / 2, radius + thickness / 2, radialCount);
theta = linspace(0, pi / 2, angularCount);      % angle away from -X axis
phi = linspace(0, 2 * pi, azimuthCount + 1);
phi(end) = [];

points = zeros(radialCount * angularCount * azimuthCount, 3);
index = 1;
for ri = 1:radialCount
    for ti = 1:angularCount
        for phiIndex = 1:azimuthCount
            direction = [ ...
                -cos(theta(ti)), ...
                sin(theta(ti)) * cos(phi(phiIndex)), ...
                sin(theta(ti)) * sin(phi(phiIndex))];
            points(index, :) = center + radii(ri) * direction;
            index = index + 1;
        end
    end
end
end

function tform = facePointingTform(point, center)
xAxis = center(:) - point(:);
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

rotation = [xAxis, yAxis, zAxis];
tform = eye(4);
tform(1:3, 1:3) = rotation;
tform(1:3, 4) = point(:);
end

function printFaceShellSummary(results)
fprintf("\nFace-centered hemispherical shell check\n");
fprintf("  Center:              [%.3f, %.3f, %.3f] m\n", results.center);
fprintf("  Radius:              %.3f m\n", results.radius);
fprintf("  Thickness:           %.3f m\n", results.thickness);
fprintf("  Distance range:      [%.3f, %.3f] m\n", results.distanceRange);
fprintf("  Samples:             %d\n", results.sampleCount);
fprintf("  Shell direction:     x <= %.3f, opening toward +X\n", results.center(1));
fprintf("  End effector:        %s\n", results.endEffector);
fprintf("  Position tolerance:  %.1f mm\n", results.positionTolerance * 1000);
fprintf("  Orientation tol.:    %.1f deg\n\n", rad2deg(results.orientationTolerance));

fprintf("Position-only coverage:\n");
fprintf("  Reachable samples:   %d / %d, %.2f %%\n", ...
    nnz(results.positionReachable), results.sampleCount, results.positionCoverage * 100);
fprintf("  Max position error:  %.1f mm\n", results.maxPositionError * 1000);
fprintf("  Mean shortfall:      %.1f mm\n", results.meanUnreachablePositionShortfall * 1000);
fprintf("  Max shortfall:       %.1f mm\n\n", results.maxUnreachablePositionShortfall * 1000);

fprintf("Pose coverage, screen local +X points to face center:\n");
fprintf("  Reachable samples:   %d / %d, %.2f %%\n", ...
    nnz(results.poseReachable), results.sampleCount, results.poseCoverage * 100);
fprintf("  Max pos. error:      %.1f mm\n", results.maxPosePositionError * 1000);
fprintf("  Max orient. error:   %.1f deg\n", rad2deg(results.maxPoseOrientationError));
fprintf("  Mean pos. shortfall: %.1f mm\n", results.meanUnreachablePoseShortfall * 1000);
fprintf("  Max pos. shortfall:  %.1f mm\n\n", results.maxUnreachablePoseShortfall * 1000);
end

function plotFaceShell(robot, qHome, results)
figure( ...
    "Name", "Face Shell Coverage", ...
    "NumberTitle", "off", ...
    "Color", "w");

hold on
unreachable = ~results.positionReachable;
scatter3( ...
    results.points(unreachable, 1), ...
    results.points(unreachable, 2), ...
    results.points(unreachable, 3), ...
    8, [0.82, 0.82, 0.82], "filled");
scatter3( ...
    results.points(results.positionReachable, 1), ...
    results.points(results.positionReachable, 2), ...
    results.points(results.positionReachable, 3), ...
    12, [0.05, 0.35, 0.90], "filled");
scatter3(results.center(1), results.center(2), results.center(3), ...
    80, [0.90, 0.10, 0.10], "filled");

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
title("Face-centered hemispherical shell: blue reachable, gray not position-reachable")
legend(["Not position-reachable", "Position-reachable", "Face center", "Robot home pose"], ...
    "Location", "best")
hold off
end
