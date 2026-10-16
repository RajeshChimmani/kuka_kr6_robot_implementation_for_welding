function kuka_weld_sim(mode)
% KUKA_WELD_SIM  One-file welding-robot simulation generator (MATLAB -> ROS 2 Jazzy / Gazebo Harmonic / RViz2)
%
%   kuka_weld_sim('gazebo')    generate everything, launch Gazebo cell + ros2_control, auto-play weld pass (default)
%   kuka_weld_sim('rviz')      generate everything, launch RViz2 + joint sliders + seam marker
%   kuka_weld_sim('replay')    generate everything, launch RViz2 and replay the planned weld pass (looping)
%   kuka_weld_sim('generate')  only (re)generate files and the trajectory plan, launch nothing
%   kuka_weld_sim('animate')   play the planned weld pass live in a MATLAB figure (table, plate, bead; MATLAB only, no ROS)
%   kuka_weld_sim('video')     render the weld pass to <pkgDir>/data/weld_simulation.mp4 (MATLAB only, no ROS needed)
%
% Run it from anywhere (e.g. your "kuka cad-urdf files" folder). It writes the ROS package to
% ~/implementation_of_kuka/custom_kuka_description/ (change cfg.pkgDir to move it). Target: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.
%
% What it creates in cfg.pkgDir :
%   urdf/   custom_kuka_kr6.urdf      links with physically consistent inertias, world link, torch + TCP frame,
%                                     ros2_control + gz_ros2_control tags
%   config/ ros2_controllers.yaml     joint_state_broadcaster + arm_controller (JointTrajectoryController)
%   worlds/ weld_cell.sdf             ground, welding table, plate, green seam marker
%   rviz/   kuka_weld.rviz
%   scripts/weld_player.py            plays the planned pass (trajectory / joint_states / seam marker)
%   launch/ gazebo.launch.py, rviz.launch.py
%   data/   weld_trajectory.csv, weld_path.csv   (output of the MATLAB IK weld planner)
%   package.xml, CMakeLists.txt       so you can later colcon-build it as a normal package
%
% Requirements: MATLAB Robotics System Toolbox (importrobot, inverseKinematics).
% ROS side:  sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
%                             ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher ros-jazzy-rviz2
%
% EXTENSION POINTS (search for "EXT:" in this file)
%   EXT:joints   edit cfg.joints (geometry, limits, effort) -> everything regenerates
%   EXT:weld     seam start/stop, speed, travel angle; add more seams by calling planWeld per seam
%   EXT:cell     add fixtures / parts in buildWorld
%   EXT:sensors  add camera / laser-scanner / F-T links in buildUrdf and bridge topics in the gazebo launch
%   EXT:control  swap arm_controller for your MPC / ADRC node by publishing to /arm_controller/joint_trajectory
%                or switching the command interface (velocity / effort) in buildUrdf + buildControllers

if nargin < 1, mode = 'gazebo'; end
cfg = defaultConfig();
if any(strcmp(mode, {'video','animate'})), cfg.preview = false; end   % those modes open their own figure
P   = buildPaths(cfg);

% ---- 1. files -----------------------------------------------------------------------------------------------
wf(P.urdf,       buildUrdf(cfg, true,  P));
plan = planWeld(cfg, buildUrdf(cfg, false, P));
writePlan(P, plan);
wf(P.ctrl,       buildControllers(cfg));
wf(P.world,      buildWorld(cfg));
wf(P.rviz,       buildRviz());
wf(P.player,     buildPlayer(cfg, P));
wf(P.launchGz,   buildLaunchGazebo(P));
wf(P.launchRviz, buildLaunchRviz(P));
wf(P.pkgXml,     buildPackageXml(cfg));
wf(P.cmake,      buildCmake(cfg));

fprintf('\nPackage generated in: %s\n', P.pkg);
fprintf('Weld pass: %d waypoints, %.1f s total, max IK pose error %.2e\n', ...
        size(plan.Q,1), plan.T(end), plan.maxErr);

% ---- 2. launch ----------------------------------------------------------------------------------------------
if strcmp(mode, 'animate'),  makeVideo(cfg, plan, P, false); return; end
if strcmp(mode, 'video'),    makeVideo(cfg, plan, P, true);  return; end
if strcmp(mode, 'generate'), return; end
assert(isfolder('/opt/ros/jazzy'), 'ROS 2 Jazzy not found in /opt/ros/jazzy');
system('bash -c "killall -9 rviz2 robot_state_publisher joint_state_publisher_gui parameter_bridge ruby 2>/dev/null"');
logFile = fullfile(getenv('HOME'), '.ros', 'kuka_weld_sim.log');
if ~isfolder(fileparts(logFile)), mkdir(fileparts(logFile)); end
switch mode
    case 'gazebo', cmd = sprintf('ros2 launch "%s" play:=true',   P.launchGz);
    case 'rviz',   cmd = sprintf('ros2 launch "%s" replay:=false', P.launchRviz);
    case 'replay', cmd = sprintf('ros2 launch "%s" replay:=true',  P.launchRviz);
    otherwise, error('Unknown mode "%s"', mode);
end
% Write a small launcher script and run it: avoids nested-quote problems with spaces in paths,
% and MATLAB's bundled libs (which break Qt/RViz/Gazebo) are removed from the environment first.
wf(P.runner, sprintf(['#!/bin/bash\nunset LD_LIBRARY_PATH QT_PLUGIN_PATH LD_PRELOAD\n' ...
                      'source "%s"\n%s\n'], cfg.rosSetup, cmd));
system(sprintf('chmod +x "%s"', P.runner));
fprintf('Launching (%s). Log: %s\n', mode, logFile);
system(sprintf('"%s" > "%s" 2>&1 &', P.runner, logFile));
end

%% =====================================================================================================
%% CONFIGURATION  (edit here)
%% =====================================================================================================
function cfg = defaultConfig()
cfg.pkg      = 'custom_kuka_description';
cfg.name     = 'custom_kuka_kr6';
% generated ROS package location (kept free of spaces on purpose; matches your earlier Gazebo URDF path)
cfg.pkgDir   = fullfile(getenv('HOME'), 'implementation_of_kuka', 'custom_kuka_description');
cfg.rosSetup = '/opt/ros/jazzy/setup.bash';
cfg.updateRate = 500;                      % controller_manager Hz (Gazebo step is 2 ms)
cfg.homeDeg  = [0 0 0 0 90 0];             % ready pose: forearm forward, torch pointing down
cfg.preview  = true;                       % MATLAB figure of robot + seam
cfg.video    = struct('fps',25,'speedup',4);   % kuka_weld_sim('video'): output frame rate, playback speed-up

% mass model: effective density of a hollow aluminium tube link
cfg.density  = 945;                        % kg/m^3 (2700 * 0.35 fill)
cfg.minMass  = 0.3;                        % kg, keeps tiny wrist links numerically stable in the physics engine
cfg.base     = struct('radius',0.09,'length',0.15,'mass',12);

% EXT:joints  -- offset xyz is parent-joint -> this joint, expressed in the parent frame.
%              each link's cylinder is drawn from its own joint to the next joint, so no visual gaps.
%                     name       parent      xyz              axis     limDeg      velDeg effort radius
cfg.joints = [ ...
    mkJ('joint_1','base_link',[0.00 0 0.15],[0 0 1],[-170 170],200,200,0.07); ...
    mkJ('joint_2','link_1',   [0.00 0 0.25],[0 1 0],[-100 100],200,200,0.06); ...
    mkJ('joint_3','link_2',   [0.00 0 0.40],[0 1 0],[-120 150],200,120,0.05); ...
    mkJ('joint_4','link_3',   [0.35 0 0.00],[1 0 0],[-185 185],300, 40,0.04); ...
    mkJ('joint_5','link_4',   [0.15 0 0.00],[0 1 0],[-120 120],300, 30,0.03); ...
    mkJ('joint_6','link_5',   [0.10 0 0.00],[1 0 0],[-350 350],400, 20,0.02)];

% welding torch (fixed to link_6) and TCP = wire tip
cfg.torch = struct('mountX',0.05,'length',0.12,'radius',0.012,'mass',1.0);

cfg.col = struct('base',[0.20 0.20 0.20 1],'arm',[1.00 0.42 0.05 1], ...
                 'wrist',[0.30 0.30 0.32 1],'torch',[0.72 0.45 0.20 1]);

% EXT:weld
cfg.weld.start        = [0.55 -0.15 0.40];  % m, world frame (plate top surface)
cfg.weld.stop         = [0.55  0.15 0.40];
cfg.weld.speed        = 0.010;              % m/s travel speed
cfg.weld.approachH    = 0.08;               % m above seam start
cfg.weld.retractH     = 0.10;
cfg.weld.approachSpeed= 0.020;              % m/s for descend / retract
cfg.weld.travelDeg    = 10;                 % torch tilt about world X (push/drag angle)
cfg.weld.homeMoveTime = 4.0;                % s, joint-space moves home <-> approach
cfg.weld.dt           = 0.1;                % s, waypoint spacing

% EXT:cell
cfg.cell.plateSize = [0.25 0.40];           % x, y of the plate
cfg.cell.plateT    = 0.02;
end

function j = mkJ(name,parent,xyz,axis,limDeg,velDeg,effort,radius)
j = struct('name',name,'parent',parent,'child',strrep(name,'joint','link'), ...
           'xyz',xyz,'axis',axis,'lim',deg2rad(limDeg),'vel',deg2rad(velDeg), ...
           'effort',effort,'radius',radius);
end

function P = buildPaths(cfg)
P.pkg  = cfg.pkgDir;
P.runner = fullfile(P.pkg, 'run_sim.sh');
P.urdf = fullfile(P.pkg, 'urdf',    [cfg.name '.urdf']);
P.ctrl = fullfile(P.pkg, 'config',  'ros2_controllers.yaml');
P.world= fullfile(P.pkg, 'worlds',  'weld_cell.sdf');
P.rviz = fullfile(P.pkg, 'rviz',    'kuka_weld.rviz');
P.player = fullfile(P.pkg, 'scripts', 'weld_player.py');
P.launchGz   = fullfile(P.pkg, 'launch', 'gazebo.launch.py');
P.launchRviz = fullfile(P.pkg, 'launch', 'rviz.launch.py');
P.traj = fullfile(P.pkg, 'data', 'weld_trajectory.csv');
P.path = fullfile(P.pkg, 'data', 'weld_path.csv');
P.pkgXml = fullfile(P.pkg, 'package.xml');
P.cmake  = fullfile(P.pkg, 'CMakeLists.txt');
end

%% =====================================================================================================
%% URDF
%% =====================================================================================================
function s = buildUrdf(cfg, withControl, P)
L = {'<?xml version="1.0"?>', sprintf('<robot name="%s">', cfg.name), ...
     '  <link name="world"/>', ...
     '  <joint name="world_fixed" type="fixed">', ...
     '    <parent link="world"/>', '    <child link="base_link"/>', ...
     '    <origin xyz="0 0 0" rpy="0 0 0"/>', '  </joint>', ''};

b = cfg.base;
L = [L, linkXml('base_link', b.mass, [0 0 b.length/2], cylInertia(b.mass,b.radius,b.length,'z'), ...
                b.radius, b.length, [0 0 b.length/2], [0 0 0], cfg.col.base)];

for i = 1:numel(cfg.joints)
    j   = cfg.joints(i);
    % the link's cylinder runs from ITS OWN joint frame to the NEXT joint (or the torch mount for link_6),
    % so it moves rigidly with its joint and never separates from the next one
    if i < numel(cfg.joints), seg = cfg.joints(i+1).xyz; else, seg = [cfg.torch.mountX 0 0]; end
    len = norm(seg);
    if abs(seg(1)) > 0, ax = 'x'; rpy = [0 pi/2 0]; else, ax = 'z'; rpy = [0 0 0]; end
    m   = max(cfg.minMass, cfg.density*pi*j.radius^2*len);
    c   = seg/2;                                 % body centre: halfway to the next joint
    if i <= 4, col = cfg.col.arm; else, col = cfg.col.wrist; end
    L = [L, jointXml(j)];
    L = [L, linkXml(j.child, m, c, cylInertia(m,j.radius,len,ax), j.radius, len, c, rpy, col)];
end

t = cfg.torch;
L = [L, fixedJointXml('torch_joint','link_6','torch',[t.mountX 0 0])];
L = [L, linkXml('torch', t.mass, [t.length/2 0 0], cylInertia(t.mass,t.radius,t.length,'x'), ...
                t.radius, t.length, [t.length/2 0 0], [0 pi/2 0], cfg.col.torch)];
L = [L, fixedJointXml('tcp_joint','torch','tcp',[t.length 0 0]), {'  <link name="tcp"/>', ''}];

if withControl
    home = deg2rad(cfg.homeDeg);
    L{end+1} = '  <ros2_control name="GazeboSystem" type="system">';
    L{end+1} = '    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>';
    for i = 1:numel(cfg.joints)
        L{end+1} = sprintf('    <joint name="%s">', cfg.joints(i).name);
        L{end+1} = '      <command_interface name="position"/>';
        L{end+1} = sprintf('      <state_interface name="position"><param name="initial_value">%.5f</param></state_interface>', home(i));
        L{end+1} = '      <state_interface name="velocity"/>';
        L{end+1} = '    </joint>';
    end
    L{end+1} = '  </ros2_control>';
    L{end+1} = '  <gazebo>';
    L{end+1} = '    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">';
    L{end+1} = sprintf('      <parameters>%s</parameters>', P.ctrl);
    L{end+1} = '    </plugin>';
    L{end+1} = '  </gazebo>';
end
L{end+1} = '</robot>';
s = joinLines(L);
end

function L = linkXml(name, m, com, I, r, len, visXYZ, visRPY, rgba)
org  = sprintf('<origin xyz="%.5f %.5f %.5f" rpy="%.5f %.5f %.5f"/>', [visXYZ visRPY]);
geom = sprintf('<geometry><cylinder radius="%.4f" length="%.4f"/></geometry>', r, len);
mat  = sprintf('<material name="mat_%s"><color rgba="%g %g %g %g"/></material>', name, rgba);
L = { sprintf('  <link name="%s">', name), ...
      '    <inertial>', sprintf('      <mass value="%.4f"/>', m), ...
      sprintf('      <origin xyz="%.5f %.5f %.5f" rpy="0 0 0"/>', com), ...
      sprintf('      <inertia ixx="%.6g" ixy="0" ixz="0" iyy="%.6g" iyz="0" izz="%.6g"/>', I), ...
      '    </inertial>', ...
      '    <visual>', ['      ' org], ['      ' geom], ['      ' mat], '    </visual>', ...
      '    <collision>', ['      ' org], ['      ' geom], '    </collision>', ...
      '  </link>', '' };
end

function L = jointXml(j)
L = { sprintf('  <joint name="%s" type="revolute">', j.name), ...
      sprintf('    <parent link="%s"/>', j.parent), sprintf('    <child link="%s"/>', j.child), ...
      sprintf('    <origin xyz="%.5f %.5f %.5f" rpy="0 0 0"/>', j.xyz), ...
      sprintf('    <axis xyz="%d %d %d"/>', j.axis), ...
      sprintf('    <limit lower="%.5f" upper="%.5f" effort="%g" velocity="%.5f"/>', j.lim(1), j.lim(2), j.effort, j.vel), ...
      '    <dynamics damping="1.0" friction="0.05"/>', ...
      '  </joint>', '' };
end

function L = fixedJointXml(name, parent, child, xyz)
L = { sprintf('  <joint name="%s" type="fixed">', name), ...
      sprintf('    <parent link="%s"/>', parent), sprintf('    <child link="%s"/>', child), ...
      sprintf('    <origin xyz="%.5f %.5f %.5f" rpy="0 0 0"/>', xyz), '  </joint>', '' };
end

function I = cylInertia(m, r, L, ax)
% principal inertias about the COM, expressed in the link frame (cylinder axis along link x or z)
Ia = 0.5*m*r^2;  Ir = m*(3*r^2 + L^2)/12;
if ax == 'z', I = [Ir Ir Ia]; else, I = [Ia Ir Ir]; end
I = max(I, 1e-4);
end

%% =====================================================================================================
%% WELD PLANNER  (MATLAB inverse kinematics on the same URDF)
%% =====================================================================================================
function plan = planWeld(cfg, urdfTxt)
tmp = [tempname '.urdf'];  wf(tmp, urdfTxt);  cleanup = onCleanup(@() delete(tmp)); %#ok<NASGU>
rbt = importrobot(tmp);  rbt.DataFormat = 'row';

names = {};
for k = 1:numel(rbt.Bodies)
    if ~strcmp(rbt.Bodies{k}.Joint.Type, 'fixed'), names{end+1} = rbt.Bodies{k}.Joint.Name; end %#ok<AGROW>
end
assert(isequal(names, {cfg.joints.name}), 'Unexpected joint order in imported robot: %s', strjoin(names, ','));

w    = cfg.weld;  dt = w.dt;
home = deg2rad(cfg.homeDeg);
ik   = inverseKinematics('RigidBodyTree', rbt);
wts  = ones(1,6);
R    = axang2tform([1 0 0 deg2rad(w.travelDeg)]) * eul2tform([0 pi/2 0]);   % torch (x axis) pointing down + tilt
pose = @(p) trvec2tform(p) * R;

pA = w.start + [0 0 w.approachH];   pS = w.start;   pE = w.stop;   pR = w.stop + [0 0 w.retractH];

Q = home;  T = 0;  Pc = tform2trvec(getTransform(rbt, home, 'tcp'));  wm = false;  maxErr = 0;

% joint-space move: home -> above the seam start
[q, e] = solveIK(ik, pose(pA), wts, home);  maxErr = max(maxErr, e);
Q(end+1,:) = q;  T(end+1,1) = T(end) + w.homeMoveTime;  Pc(end+1,:) = pA;  wm(end+1,1) = false;

% Cartesian segments: descend, weld, retract
segs = {pA, pS, w.approachSpeed, false; pS, pE, w.speed, true; pE, pR, w.approachSpeed, false};
for s = 1:size(segs,1)
    a = segs{s,1};  b = segs{s,2};  v = segs{s,3};
    n = max(1, ceil(norm(b-a)/v/dt));
    for k = 1:n
        p = a + (b-a)*k/n;
        [q, e] = solveIK(ik, pose(p), wts, q);  maxErr = max(maxErr, e);
        Q(end+1,:) = q;  T(end+1,1) = T(end) + norm(b-a)/v/n;  Pc(end+1,:) = p;  wm(end+1,1) = segs{s,4}; %#ok<AGROW>
    end
end

% return home
Q(end+1,:) = home;  T(end+1,1) = T(end) + w.homeMoveTime;  Pc(end+1,:) = Pc(1,:);  wm(end+1,1) = false;

% sanity checks
if maxErr > 1e-3, warning('IK pose error up to %.2e - seam may be at the edge of the workspace.', maxErr); end
rate = abs(diff(Q)) ./ diff(T);
vmax = [cfg.joints.vel];
for i = 1:6
    if max(rate(:,i)) > vmax(i)
        warning('%s exceeds its velocity limit (%.2f > %.2f rad/s). Lower weld speed or change the seam.', ...
                cfg.joints(i).name, max(rate(:,i)), vmax(i));
    end
end
if max(abs(diff(Q(2:end-1,:))), [], 'all') > 0.2
    warning('Large joint step in the weld pass - IK may have flipped configuration.');
end

plan = struct('Q',Q,'T',T,'weldPath',[w.start; Pc(wm,:)],'maxErr',maxErr,'rbt',rbt,'Pc',Pc,'wm',wm);

if cfg.preview
    fig = figure('Name','KUKA weld cell preview','Color','w');
    ax  = axes(fig);  hold(ax,'on');
    drawCell(ax, cfg);
    show(rbt, Q(round(size(Q,1)/2),:), 'Parent', ax, 'Frames','off', 'PreservePlot', false);
    plot3(ax, plan.weldPath(:,1), plan.weldPath(:,2), plan.weldPath(:,3), '-', 'Color',[1 0.3 0], 'LineWidth', 4);
    plot3(ax, Pc(:,1), Pc(:,2), Pc(:,3), 'b:', 'LineWidth', 1);
    axis(ax, [-0.2 0.8 -0.4 0.4 0 0.9]);  daspect(ax, [1 1 1]);  view(ax, -40, 32);
    title(ax, 'Planned weld pass (orange = bead, blue dotted = TCP path). kuka_weld_sim(''animate'') plays it.');
end
end

function [q, err] = solveIK(ik, Tgt, wts, q0)
[q, info] = ik('tcp', Tgt, wts, q0);
err = info.PoseErrorNorm;
end

function writePlan(P, plan)
if ~isfolder(fileparts(P.traj)), mkdir(fileparts(P.traj)); end
fid = fopen(P.traj, 'w');  fprintf(fid, 't,j1,j2,j3,j4,j5,j6\n');
fprintf(fid, '%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', [plan.T plan.Q]');  fclose(fid);
fid = fopen(P.path, 'w');  fprintf(fid, 'x,y,z\n');
fprintf(fid, '%.5f,%.5f,%.5f\n', plan.weldPath');  fclose(fid);
end

%% =====================================================================================================
%% ros2_control YAML, RViz config, package files
%% =====================================================================================================
function s = buildControllers(cfg)
names = strjoin({cfg.joints.name}, ', ');
L = { 'controller_manager:', '  ros__parameters:', ...
      sprintf('    update_rate: %d', cfg.updateRate), '    use_sim_time: true', ...
      '    joint_state_broadcaster:', '      type: joint_state_broadcaster/JointStateBroadcaster', ...
      '    arm_controller:', '      type: joint_trajectory_controller/JointTrajectoryController', '', ...
      'arm_controller:', '  ros__parameters:', ...
      sprintf('    joints: [%s]', names), ...
      '    command_interfaces: [position]', '    state_interfaces: [position, velocity]', ...
      '    state_publish_rate: 100.0', '    action_monitor_rate: 20.0', ...
      '    allow_partial_joints_goal: false', '    open_loop_control: false', ...
      '    constraints:', '      stopped_velocity_tolerance: 0.05', '      goal_time: 0.0' };
s = joinLines(L);
end

function s = buildRviz()
L = { 'Visualization Manager:', '  Displays:', ...
      '    - Class: rviz_default_plugins/Grid', '      Enabled: true', '      Name: Grid', ...
      '    - Class: rviz_default_plugins/RobotModel', '      Description Topic:', '        Value: /robot_description', ...
      '      Enabled: true', '      Name: RobotModel', ...
      '    - Class: rviz_default_plugins/TF', '      Enabled: true', '      Name: TF', ...
      '    - Class: rviz_default_plugins/Marker', '      Enabled: true', '      Name: WeldSeam', ...
      '      Topic:', '        Value: /weld_seam', ...
      '  Global Options:', '    Fixed Frame: world', ...
      '  Views:', '    Current:', '      Class: rviz_default_plugins/Orbit', ...
      '      Distance: 2.0', '      Focal Point: [0.4, 0.0, 0.4]' };
s = joinLines(L);
end

function s = buildPackageXml(cfg)
L = { '<?xml version="1.0"?>', '<package format="3">', ...
      sprintf('  <name>%s</name>', cfg.pkg), '  <version>0.1.0</version>', ...
      '  <description>Welding KUKA KR6-style robot cell (generated by kuka_weld_sim.m)</description>', ...
      '  <maintainer email="user@example.com">user</maintainer>', '  <license>MIT</license>', ...
      '  <buildtool_depend>ament_cmake</buildtool_depend>', ...
      '  <exec_depend>robot_state_publisher</exec_depend>', '  <exec_depend>joint_state_publisher_gui</exec_depend>', ...
      '  <exec_depend>rviz2</exec_depend>', '  <exec_depend>ros_gz_sim</exec_depend>', '  <exec_depend>ros_gz_bridge</exec_depend>', ...
      '  <exec_depend>gz_ros2_control</exec_depend>', '  <exec_depend>controller_manager</exec_depend>', ...
      '  <exec_depend>joint_trajectory_controller</exec_depend>', '  <exec_depend>joint_state_broadcaster</exec_depend>', ...
      '  <export><build_type>ament_cmake</build_type></export>', '</package>' };
s = joinLines(L);
end

function s = buildCmake(cfg)
L = { 'cmake_minimum_required(VERSION 3.8)', sprintf('project(%s)', cfg.pkg), ...
      'find_package(ament_cmake REQUIRED)', ...
      'install(DIRECTORY urdf launch config worlds rviz scripts data DESTINATION share/${PROJECT_NAME})', ...
      'ament_package()' };
s = joinLines(L);
end

%% =====================================================================================================
%% Gazebo world
%% =====================================================================================================
function s = buildWorld(cfg)
w = cfg.weld;  c = cfg.cell;
mid = (w.start + w.stop)/2;
d   = w.stop - w.start;
seamLen = norm(d);  yaw = atan2(d(2), d(1));
tableTop = w.start(3) - c.plateT;
L = { '<?xml version="1.0"?>', '<sdf version="1.9">', '  <world name="weld_cell">', ...
      '    <physics name="fast" type="ode"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>', ...
      '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>', ...
      '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>', ...
      '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>', ...
      '    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 5 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.2 -0.9</direction></light>', ...
      '    <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision><visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material></visual></link></model>', ...
      ['    ' boxModel('weld_table', [mid(1) mid(2) tableTop/2 0 0 0], [c.plateSize(1)+0.15 c.plateSize(2)+0.10 tableTop], [0.35 0.35 0.40 1], true)], ...
      ['    ' boxModel('weld_plate', [mid(1) mid(2) tableTop+c.plateT/2 0 0 0], [c.plateSize(1) c.plateSize(2) c.plateT], [0.65 0.65 0.68 1], true)], ...
      ['    ' boxModel('seam_marker', [mid(1) mid(2) w.start(3)+0.001 0 0 yaw], [seamLen 0.008 0.002], [0 1 0.2 1], false)], ...
      '  </world>', '</sdf>' };
s = joinLines(L);
end

function s = boxModel(name, pose, size3, rgba, collide)
head = sprintf('<model name="%s"><static>true</static><pose>%.4f %.4f %.4f %.4f %.4f %.4f</pose><link name="l">', name, pose);
col  = '';
if collide
    col = sprintf('<collision name="c"><geometry><box><size>%.4f %.4f %.4f</size></box></geometry></collision>', size3);
end
vis  = sprintf('<visual name="v"><geometry><box><size>%.4f %.4f %.4f</size></box></geometry><material><ambient>%g %g %g %g</ambient><diffuse>%g %g %g %g</diffuse></material></visual>', ...
               size3, rgba, rgba);
s = [head col vis '</link></model>'];
end

%% =====================================================================================================
%% Python: weld player + launch files
%% =====================================================================================================
function s = buildPlayer(cfg, P)
jn = strjoin(cellfun(@(x) ['"' x '"'], {cfg.joints.name}, 'UniformOutput', false), ', ');
L = { '#!/usr/bin/env python3', ...
 '# Plays the MATLAB-planned weld pass. Modes: trajectory (to arm_controller), jointstate (RViz replay), marker (seam only)', ...
 'import argparse, bisect, csv', 'import rclpy', 'from rclpy.node import Node', ...
 'from builtin_interfaces.msg import Duration', 'from geometry_msgs.msg import Point', ...
 'from sensor_msgs.msg import JointState', 'from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint', ...
 'from visualization_msgs.msg import Marker', '', ...
 sprintf('JOINTS = [%s]', jn), 'TRAJ = "@TRAJ@"', 'PATH = "@PATH@"', '', ...
 'def load(path):', '    with open(path) as f:', '        rows = list(csv.reader(f))[1:]', ...
 '    return [[float(v) for v in r] for r in rows]', '', ...
 'def dur(t):', '    s = int(t)', '    return Duration(sec=s, nanosec=int((t - s) * 1e9))', '', ...
 'class Player(Node):', ...
 '    def __init__(self, mode, loop):', '        super().__init__("weld_player")', ...
 '        self.loop = loop', '        self.traj = load(TRAJ)', '        self.pts = load(PATH)', ...
 '        self.times = [r[0] for r in self.traj]', ...
 '        if mode in ("jointstate", "marker"):', ...
 '            self.mpub = self.create_publisher(Marker, "/weld_seam", 10)', ...
 '            self.create_timer(1.0, self.pub_marker)', ...
 '        if mode == "jointstate":', ...
 '            self.jpub = self.create_publisher(JointState, "/joint_states", 10)', ...
 '            self.t0 = self.get_clock().now()', '            self.create_timer(0.02, self.pub_state)', ...
 '        if mode == "trajectory":', '            self.send()', '', ...
 '    def pub_marker(self):', ...
 '        m = Marker()', '        m.header.frame_id = "world"', '        m.ns = "weld"', '        m.id = 0', ...
 '        m.type = Marker.LINE_STRIP', '        m.action = Marker.ADD', '        m.scale.x = 0.005', ...
 '        m.color.r = 0.0', '        m.color.g = 1.0', '        m.color.b = 0.2', '        m.color.a = 1.0', ...
 '        m.pose.orientation.w = 1.0', ...
 '        for x, y, z in self.pts:', '            m.points.append(Point(x=x, y=y, z=z))', ...
 '        self.mpub.publish(m)', '', ...
 '    def pub_state(self):', ...
 '        el = (self.get_clock().now() - self.t0).nanoseconds * 1e-9', '        end = self.times[-1]', ...
 '        if self.loop:', '            el = el % end', '        el = min(el, end)', ...
 '        i = max(1, min(bisect.bisect_left(self.times, el), len(self.times) - 1))', ...
 '        a, b = self.traj[i - 1], self.traj[i]', ...
 '        f = 0.0 if b[0] == a[0] else (el - a[0]) / (b[0] - a[0])', ...
 '        js = JointState()', '        js.header.stamp = self.get_clock().now().to_msg()', '        js.name = JOINTS', ...
 '        js.position = [a[k] + f * (b[k] - a[k]) for k in range(1, 7)]', '        self.jpub.publish(js)', '', ...
 '    def send(self):', ...
 '        pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)', ...
 '        self.get_logger().info("waiting for arm_controller ...")', ...
 '        while pub.get_subscription_count() == 0 and rclpy.ok():', '            rclpy.spin_once(self, timeout_sec=0.2)', ...
 '        msg = JointTrajectory()', '        msg.joint_names = JOINTS', ...
 '        for r in self.traj:', '            if r[0] <= 0.0:', '                continue', ...
 '            p = JointTrajectoryPoint()', '            p.positions = r[1:7]', '            p.time_from_start = dur(r[0])', ...
 '            msg.points.append(p)', ...
 '        pub.publish(msg)', ...
 '        self.get_logger().info("weld pass sent: %d points, %.1f s" % (len(msg.points), self.traj[-1][0]))', ...
 '        for _ in range(10):', '            rclpy.spin_once(self, timeout_sec=0.1)', '        raise SystemExit', '', ...
 'def main():', '    ap = argparse.ArgumentParser()', ...
 '    ap.add_argument("--mode", choices=["trajectory", "jointstate", "marker"], default="trajectory")', ...
 '    ap.add_argument("--loop", action="store_true")', '    a = ap.parse_args()', ...
 '    rclpy.init()', '    try:', '        rclpy.spin(Player(a.mode, a.loop))', ...
 '    except (SystemExit, KeyboardInterrupt):', '        pass', '    rclpy.try_shutdown()', '', ...
 'if __name__ == "__main__":', '    main()' };
s = joinLines(L);
s = strrep(s, '@TRAJ@', P.traj);
s = strrep(s, '@PATH@', P.path);
end

function s = buildLaunchGazebo(P)
L = { 'import os', 'from ament_index_python.packages import get_package_share_directory', ...
 'from launch import LaunchDescription', ...
 'from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction', ...
 'from launch.conditions import IfCondition', 'from launch.event_handlers import OnProcessExit', ...
 'from launch.launch_description_sources import PythonLaunchDescriptionSource', ...
 'from launch.substitutions import LaunchConfiguration', 'from launch_ros.actions import Node', '', ...
 'URDF = "@URDF@"', 'WORLD = "@WORLD@"', 'PLAYER = "@PLAYER@"', '', ...
 'def generate_launch_description():', '    with open(URDF) as f:', '        robot_desc = f.read()', ...
 '    gz = IncludeLaunchDescription(', ...
 '        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),', ...
 '        launch_arguments={"gz_args": "-r " + WORLD}.items())', ...
 '    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",', ...
 '               parameters=[{"robot_description": robot_desc, "use_sim_time": True}])', ...
 '    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",', ...
 '                  arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"])', ...
 '    spawn = Node(package="ros_gz_sim", executable="create", output="screen",', ...
 '                 arguments=["-topic", "robot_description", "-name", "custom_kuka_kr6", "-z", "0.0"])', ...
 '    jsb = Node(package="controller_manager", executable="spawner",', ...
 '               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])', ...
 '    arm = Node(package="controller_manager", executable="spawner",', ...
 '               arguments=["arm_controller", "--controller-manager", "/controller_manager"])', ...
 '    player = TimerAction(period=3.0, actions=[ExecuteProcess(', ...
 '        cmd=["python3", PLAYER, "--mode", "trajectory"], output="screen",', ...
 '        condition=IfCondition(LaunchConfiguration("play")))])', ...
 '    return LaunchDescription([', '        DeclareLaunchArgument("play", default_value="true"),', ...
 '        gz, rsp, bridge, spawn,', ...
 '        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),', ...
 '        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),', ...
 '        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[player])),', '    ])' };
s = joinLines(L);
s = strrep(s, '@URDF@', P.urdf);  s = strrep(s, '@WORLD@', P.world);  s = strrep(s, '@PLAYER@', P.player);
end

function s = buildLaunchRviz(P)
L = { 'from launch import LaunchDescription', 'from launch.actions import DeclareLaunchArgument, ExecuteProcess', ...
 'from launch.conditions import IfCondition, UnlessCondition', 'from launch.substitutions import LaunchConfiguration', ...
 'from launch_ros.actions import Node', '', ...
 'URDF = "@URDF@"', 'RVIZ = "@RVIZ@"', 'PLAYER = "@PLAYER@"', '', ...
 'def generate_launch_description():', '    with open(URDF) as f:', '        robot_desc = f.read()', ...
 '    replay = LaunchConfiguration("replay")', ...
 '    return LaunchDescription([', '        DeclareLaunchArgument("replay", default_value="false"),', ...
 '        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",', ...
 '             parameters=[{"robot_description": robot_desc}]),', ...
 '        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", output="screen",', ...
 '             condition=UnlessCondition(replay)),', ...
 '        ExecuteProcess(cmd=["python3", PLAYER, "--mode", "marker"], output="screen", condition=UnlessCondition(replay)),', ...
 '        ExecuteProcess(cmd=["python3", PLAYER, "--mode", "jointstate", "--loop"], output="screen", condition=IfCondition(replay)),', ...
 '        Node(package="rviz2", executable="rviz2", arguments=["-d", RVIZ]),', '    ])' };
s = joinLines(L);
s = strrep(s, '@URDF@', P.urdf);  s = strrep(s, '@RVIZ@', P.rviz);  s = strrep(s, '@PLAYER@', P.player);
end

%% =====================================================================================================
%% Video (MATLAB only)
%% =====================================================================================================
function makeVideo(cfg, plan, P, record)
% record = true: write <pkgDir>/data/weld_simulation.mp4 ; record = false: just play live in a MATLAB figure
rbt = plan.rbt;
out = fullfile(P.pkg, 'data', 'weld_simulation.mp4');
fps = cfg.video.fps;
tt  = 0 : cfg.video.speedup/fps : plan.T(end);
Qi  = interp1(plan.T, plan.Q, tt, 'pchip');
wi  = interp1(plan.T, double(plan.wm), tt, 'next', 0) > 0.5;      % true while the torch is welding

fig = figure('Name','Weld simulation','Color','w','Position',[50 50 1280 720]);
ax  = axes(fig);  hold(ax,'on');
drawCell(ax, cfg);
hBead = plot3(ax, nan, nan, nan, '-', 'Color',[1 0.3 0], 'LineWidth',4);
hTip  = plot3(ax, nan, nan, nan, 'o', 'MarkerFaceColor',[1 0.9 0.2], 'MarkerEdgeColor','k', 'MarkerSize',8);

vw = [];
if record
    try
        vw = VideoWriter(out, 'MPEG-4');  vw.FrameRate = fps;  vw.Quality = 95;  open(vw);
    catch
        out = strrep(out, '.mp4', '.avi');                          % MPEG-4 needs GStreamer codecs on some Linux installs
        vw = VideoWriter(out, 'Motion JPEG AVI');  vw.FrameRate = fps;  open(vw);
    end
end
bead = zeros(0,3);
for k = 1:numel(tt)
    if ~isvalid(fig), break; end                                    % window closed -> stop
    q = Qi(k,:);
    show(rbt, q, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'Collisions', 'off');
    p = tform2trvec(getTransform(rbt, q, 'tcp'));
    if wi(k), bead(end+1,:) = p; end %#ok<AGROW>
    if ~isempty(bead), set(hBead, 'XData',bead(:,1), 'YData',bead(:,2), 'ZData',bead(:,3)); end
    set(hTip, 'XData',p(1), 'YData',p(2), 'ZData',p(3));
    axis(ax, [-0.2 0.8 -0.4 0.4 0 0.9]);  daspect(ax, [1 1 1]);  view(ax, -40, 32);
    if wi(k), state = 'WELDING'; else, state = 'moving'; end
    title(ax, sprintf('%s   |   t = %5.1f s   |   playback x%g', state, tt(k), cfg.video.speedup));
    drawnow;
    if record, writeVideo(vw, getframe(fig)); else, pause(1/fps); end
end
if record
    close(vw);  fprintf('Video written: %s\n', out);
end
end

function drawBox(ax, c, s, col)
V = c + (s/2) .* [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', V, 'Faces', F, 'FaceColor', col, 'EdgeColor', 'none');
end

function drawCell(ax, cfg)
% welding table, plate and seam line (shared by the preview figure, the live animation and the video)
w = cfg.weld;  c = cfg.cell;
mid = (w.start + w.stop)/2;  tableTop = w.start(3) - c.plateT;
drawBox(ax, [mid(1) mid(2) tableTop/2],          [c.plateSize(1)+0.15 c.plateSize(2)+0.10 tableTop], [0.29 0.33 0.39]);
drawBox(ax, [mid(1) mid(2) tableTop+c.plateT/2], [c.plateSize(1) c.plateSize(2) c.plateT],          [0.82 0.84 0.86]);
plot3(ax, [w.start(1) w.stop(1)], [w.start(2) w.stop(2)], [w.start(3) w.stop(3)]+0.001, '-', 'Color',[0.2 0.8 0.3], 'LineWidth',2);
grid(ax, 'on');
end

%% =====================================================================================================
%% helpers
%% =====================================================================================================
function s = joinLines(L)
s = [strjoin(L(:).', newline) newline];
end

function wf(p, txt)
d = fileparts(p);
if ~isempty(d) && ~isfolder(d), mkdir(d); end
fid = fopen(p, 'w');  assert(fid > 0, 'Cannot write %s', p);
fwrite(fid, txt);  fclose(fid);
end