function [robot, q] = open_face_screen_arm_matlab(poseName)
%OPEN_FACE_SCREEN_ARM_MATLAB Open the face screen support arm URDF in MATLAB.

if nargin < 1 || strlength(string(poseName)) == 0
    poseName = "home";
else
    poseName = lower(string(poseName));
end

projectRoot = fileparts(mfilename("fullpath"));
urdfPath = fullfile(projectRoot, "generated", "urdf", "face_screen_support_arm.urdf");

robot = importrobot(urdfPath);
robot.DataFormat = "column";
robot.Gravity = [0 0 -9.81];

q = homeConfiguration(robot);
q = applyPose(q, poseName);

fig = figure( ...
    "Name", "Face Screen Support Arm", ...
    "NumberTitle", "off", ...
    "Color", "w", ...
    "Visible", "on");

show(robot, q, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "off", ...
    "Parent", axes("Parent", fig));

axis equal
grid on
view(135, 25)
title("Face Screen Support Arm - " + poseName)
drawnow
figure(fig)

assignin("base", "robot", robot);
assignin("base", "q", q);
assignin("base", "faceScreenArmFigure", fig);

fprintf("Face Screen Support Arm loaded. Pose=%s. Variables: robot, q, faceScreenArmFigure\n", poseName);
end

function q = applyPose(q, poseName)
poses = containers.Map;
poses("home") = [0, -120, 120, 30, 0, 0];
poses("left") = [45, -35, -45, 120, -90, 10];
poses("right") = [-45, -35, -45, 120, 90, 10];
poses("near") = [0, -55, -55, 20, 0, 5];
poses("far") = [0, -20, -20, 420, 0, 0];

if ~isKey(poses, poseName)
    error("Unknown pose '%s'. Available poses: home, left, right, near, far.", poseName);
end

displayValues = poses(poseName);
q(1) = deg2rad(displayValues(1));
q(2) = deg2rad(displayValues(2));
q(3) = deg2rad(displayValues(3));
q(4) = displayValues(4) / 1000;
q(5) = deg2rad(displayValues(5));
q(6) = deg2rad(displayValues(6));
end
