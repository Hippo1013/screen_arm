function demo_face_screen_arm_motion_loop(cycles, workspaceMode)
%DEMO_FACE_SCREEN_ARM_MOTION_LOOP Animate the arm through one closed loop.
%
% Usage:
%   demo_face_screen_arm_motion_loop
%   demo_face_screen_arm_motion_loop(3)
%   demo_face_screen_arm_motion_loop(1, "large")
%
% The robot base is fixed by the rigidBodyTree root. This demo only changes
% joint positions, so base_link stays fixed in the world frame.

if nargin < 1
    cycles = 1;
end
if nargin < 2 || strlength(string(workspaceMode)) == 0
    workspaceMode = "normal";
else
    workspaceMode = lower(string(workspaceMode));
end

projectRoot = fileparts(fileparts(mfilename("fullpath")));
urdfPath = fullfile(projectRoot, "generated", "urdf", "face_screen_support_arm.urdf");

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

% Display-unit waypoints:
% J1/J2/J3/J5/J6 are degrees, J4 is millimeters.
waypointsDisplay = [
      0, -120, 120,  30,   0,  0;  % home
     45,  -55, -35, 160, -45, 10;  % move left and forward
      0,  -35, -65, 260,   0,  5;  % reach outward
    -45,  -55, -35, 160,  45, 10;  % move right
      0, -120, 120,  30,   0,  0   % back home
];

waypoints = zeros(size(waypointsDisplay));
for i = 1:size(waypointsDisplay, 1)
    waypoints(i, :) = displayPoseToConfig(waypointsDisplay(i, :));
end

q = waypoints(1, :).';

fig = figure( ...
    "Name", "Face Screen Support Arm Motion Demo", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "Visible", "on");
ax = axes("Parent", fig);

show(robot, q, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "Parent", ax);

setupAxes(ax, workspaceMode);
title(ax, "Base fixed, joints moving, returning to home")
drawnow

framesPerSegment = 45;
secondsPerFrame = 0.03;

for cycleIndex = 1:cycles
    for segmentIndex = 1:(size(waypoints, 1) - 1)
        qStart = waypoints(segmentIndex, :).';
        qGoal = waypoints(segmentIndex + 1, :).';

        for frameIndex = 1:framesPerSegment
            t = frameIndex / framesPerSegment;
            s = smoothStep(t);
            q = qStart + (qGoal - qStart) * s;

            show(robot, q, ...
                "Visuals", "on", ...
                "Collisions", "off", ...
                "Frames", "off", ...
                "Parent", ax, ...
                "PreservePlot", false, ...
                "FastUpdate", true);

            setupAxes(ax, workspaceMode);
            title(ax, sprintf( ...
                "Cycle %d/%d, segment %d/%d", ...
                cycleIndex, cycles, segmentIndex, size(waypoints, 1) - 1));
            drawnow
            pause(secondsPerFrame)
        end
    end
end

q = waypoints(1, :).';
show(robot, q, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "Parent", ax, ...
    "PreservePlot", false, ...
    "FastUpdate", true);
setupAxes(ax, workspaceMode);
title(ax, "Returned to home")
drawnow

assignin("base", "robot", robot);
assignin("base", "q", q);
assignin("base", "motionWaypointsDisplay", waypointsDisplay);
assignin("base", "motionFigure", fig);

fprintf("Motion demo finished. Base stayed fixed. Variables: robot, q, motionWaypointsDisplay, motionFigure\n");
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

function s = smoothStep(t)
% Cubic interpolation with zero velocity at both ends.
s = 3 * t^2 - 2 * t^3;
end

function setupAxes(ax, workspaceMode)
axis(ax, "equal")
grid(ax, "on")
view(ax, 135, 25)

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

xlim(ax, limits(1, :))
ylim(ax, limits(2, :))
zlim(ax, limits(3, :))
xlabel(ax, "X / m")
ylabel(ax, "Y / m")
zlabel(ax, "Z / m")
end
