%% 1. Clear Workspace & Create Robot
clear; clc; close all;
robot = rigidBodyTree('DataFormat', 'column');

%% 2. Define Link Visual Dimensions & Names
% Radii and lengths for Base + 6 Links
radii   = [0.09, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02];
lengths = [0.15, 0.25, 0.40, 0.35, 0.15, 0.10, 0.05];

bodyNames   = {'link_1', 'link_2', 'link_3', 'link_4', 'link_5', 'link_6'};
parentNames = {'base_link', 'link_1', 'link_2', 'link_3', 'link_4', 'link_5'};

% Offsets from parent joint frame [X, Y, Z]
jointOffsets = [
    0.00,  0.00,  0.15;   % J1: Base height
    0.00,  0.00,  0.25;   % J2: Shoulder offset
    0.00,  0.00,  0.40;   % J3: Upper arm (Z displacement)
    0.35,  0.00,  0.00;   % J4: Forearm extension (X displacement)
    0.15,  0.00,  0.00;   % J5: Wrist pitch (X displacement)
    0.10,  0.00,  0.00    % J6: Flange/Torch (X displacement)
];

% Joint axes [X Y Z]
jointAxes = [
    0 0 1;  % J1: Base Yaw
    0 1 0;  % J2: Shoulder Pitch
    0 1 0;  % J3: Elbow Pitch
    1 0 0;  % J4: Forearm Roll
    0 1 0;  % J5: Wrist Pitch
    1 0 0   % J6: Flange Roll
];

%% 3. Add Base Link
baseLink = rigidBody('base_link');
baseJoint = rigidBodyJoint('base_joint', 'fixed');
setFixedTransform(baseJoint, trvec2tform([0, 0, 0]));
baseLink.Joint = baseJoint;

% Base cylinder centered on local origin
addVisual(baseLink, "Cylinder", [radii(1), lengths(1)], trvec2tform([0, 0, lengths(1)/2]));
addCollision(baseLink, "Cylinder", [radii(1), lengths(1)], trvec2tform([0, 0, lengths(1)/2]));
addBody(robot, baseLink, 'base');

%% 4. Assemble Robot Links
for i = 1:6
    body = rigidBody(bodyNames{i});
    joint = rigidBodyJoint(sprintf('joint_%d', i), 'revolute');
    joint.JointAxis = jointAxes(i, :);
    
    % Set relative transformation
    setFixedTransform(joint, trvec2tform(jointOffsets(i, :)));
    body.Joint = joint;
    
    % Position visual mesh to fill the gap between current joint and parent joint
    if jointOffsets(i, 1) > 0 
        % Horizontal link along +X: Center cylinder midway back toward parent
        visTransform = trvec2tform([-jointOffsets(i,1)/2, 0, 0]) * eul2tform([0, pi/2, 0]);
        meshLen = jointOffsets(i, 1);
    else 
        % Vertical link along +Z: Center cylinder midway back toward parent
        visTransform = trvec2tform([0, 0, -jointOffsets(i,3)/2]);
        meshLen = jointOffsets(i, 3);
    end
    
    addVisual(body, "Cylinder", [radii(i+1), meshLen], visTransform);
    addCollision(body, "Cylinder", [radii(i+1), meshLen], visTransform);
    
    addBody(robot, body, parentNames{i});
end

%% 5. Render & Verify Visuals
figure('Name', 'Fully Connected 6-DOF Robot Arm', 'Color', 'w');
show(robot, homeConfiguration(robot), 'Visuals', 'on', 'Collisions', 'on');
title('Connected 6-DOF Manipulator (No Gaps)');
grid on; axis equal; view(135, 20);

%% 6. Export Corrected URDF
urdfFileName = 'custom_kuka_kr6.urdf';
fid = fopen(urdfFileName, 'w');

fprintf(fid, '<?xml version="1.0"?>\n<robot name="custom_kuka_kr6">\n\n');
fprintf(fid, '  <link name="base_link">\n    <visual><geometry><cylinder length="%.3f" radius="%.3f"/></geometry></visual>\n  </link>\n\n', lengths(1), radii(1));

for i = 1:6
    fprintf(fid, '  <joint name="joint_%d" type="revolute">\n', i);
    fprintf(fid, '    <parent link="%s"/>\n', parentNames{i});
    fprintf(fid, '    <child link="%s"/>\n', bodyNames{i});
    fprintf(fid, '    <origin xyz="%.4f %.4f %.4f" rpy="0 0 0"/>\n', ...
            jointOffsets(i,1), jointOffsets(i,2), jointOffsets(i,3));
    fprintf(fid, '    <axis xyz="%.0f %.0f %.0f"/>\n', ...
            jointAxes(i,1), jointAxes(i,2), jointAxes(i,3));
    fprintf(fid, '    <limit lower="-3.1416" upper="3.1416" effort="100" velocity="3.0"/>\n');
    fprintf(fid, '  </joint>\n\n');
    
    if jointOffsets(i,1) > 0
        visX = -jointOffsets(i,1)/2; visZ = 0; rpyStr = '0 1.5708 0';
        cLen = jointOffsets(i,1);
    else
        visX = 0; visZ = -jointOffsets(i,3)/2; rpyStr = '0 0 0';
        cLen = jointOffsets(i,3);
    end
    
    fprintf(fid, '  <link name="%s">\n', bodyNames{i});
    fprintf(fid, '    <visual>\n');
    fprintf(fid, '      <origin xyz="%.4f 0 %.4f" rpy="%s"/>\n', visX, visZ, rpyStr);
    fprintf(fid, '      <geometry><cylinder length="%.4f" radius="%.4f"/></geometry>\n', cLen, radii(i+1));
    fprintf(fid, '    </visual>\n');
    fprintf(fid, '  </link>\n\n');
end

fprintf(fid, '</robot>\n');
fclose(fid);
fprintf('\nGenerated solid URDF: %s\n', fullfile(pwd, urdfFileName));