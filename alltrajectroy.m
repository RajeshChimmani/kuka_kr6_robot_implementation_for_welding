%% KUKA KR6 - Kinematic model + Friction compensation (PID vs NMPC vs PINN-NMPC)
% Single-file project: run this file in MATLAB, nothing else is needed.
%
% Suggested location:
%   /home/rajesh/kuka_robot/kuka_nmpc_pinn_sim.m
%
% Requirements:
%   - Robotics System Toolbox (rigidBodyTree, show, getTransform)
%   - CasADi + IPOPT for the two NMPC controllers (PID runs without it)
%
% What it does:
%   1. Builds the 6-DOF rigidBodyTree (same geometry as your URDF)
%   2. Fits the smooth friction model (physics-structured "PINN" model)
%   3. Simulates PID, standard NMPC and friction-aware PINN-NMPC
%   4. Prints metrics, draws plots
%   5. Animates the 3D robot for every controller side by side
%
% NOTE: The dynamics used for control are the simplified per-joint model
% from your original script (illustrative, not manufacturer-validated).
% The 3D robot is driven by the simulated joint angles for visualisation.

clear; clc; close all;
rng(7);

%% 0. User options
opts.exportURDF   = true;     % also write custom_kuka_kr6.urdf next to this file
opts.animate      = true;     % show the 3D animation
opts.frameStride  = 1;        % animate every n-th simulation step
opts.saveVideo    = false;    % true -> writes kuka_nmpc_pinn.mp4
opts.casadiPaths  = {'/home/rajesh/kuka_robot/casadi', '/home/rajesh/casadi'};

thisFile = mfilename('fullpath');
if isempty(thisFile), outDir = pwd; else, outDir = fileparts(thisFile); end

%% 1. Kinematic model (rigidBodyTree)
robot = buildKukaRobot();
if opts.exportURDF
    exportKukaURDF(fullfile(outDir,'custom_kuka_kr6.urdf'));
end
hasRobotics = ~isempty(robot);

%% 2. Dynamics / friction settings
NJ = 6;  G = 9.81;
M  = [3.0; 4.0; 3.5; 2.0; 1.5; 0.8];
L  = [0.4; 0.4; 0.3; 0.2; 0.15; 0.1];
I0 = M .* L.^2;

FC = [1.5; 2.0; 1.2; 0.8; 0.5; 0.3];
FS = [2.5; 3.2; 2.0; 1.4; 0.9; 0.5];
VS = [0.10; 0.10; 0.08; 0.05; 0.05; 0.05];
SV = [0.8; 1.1; 0.6; 0.4; 0.3; 0.2];

trueFriction = @(dq) FC.*tanh(dq/0.015) + ...
    (FS-FC).*exp(-(dq./VS).^2).*tanh(dq/0.015) + SV.*dq;
gravity = @(q,m) m .* G .* L .* cos(q);

DT = 0.02;  T = 10.0;  Nsim = round(T/DT);
payloadTime = 0.75;  payloadMass = 4.0;
qTarget = [0.3; -0.4; 0.5; -0.2; 0.4; 0.2];
xRef = [qTarget; zeros(NJ,1)];
tauMax = 30;

hasCasadi = exist('casadi.Opti','class') == 8 || exist('Opti','class') == 8;
if ~hasCasadi
    for ci = 1:numel(opts.casadiPaths)
        if exist(opts.casadiPaths{ci},'dir'), addpath(genpath(opts.casadiPaths{ci})); end
    end
    hasCasadi = exist('casadi.Opti','class') == 8 || exist('Opti','class') == 8;
end
fprintf('Sample time %.3f s | Duration %.1f s | Steps %d | CasADi: %d | Robotics TB: %d\n', ...
    DT,T,Nsim,hasCasadi,hasRobotics);

%% 3. Friction model fit (odd tanh basis + viscous term)
nTrain = 8000;
dqTrain = -3 + 6*rand(nTrain,NJ);
yTrain = zeros(nTrain,NJ);
for j = 1:NJ
    yTrain(:,j) = FC(j).*tanh(dqTrain(:,j)/0.015) + ...
        (FS(j)-FC(j)).*exp(-(dqTrain(:,j)./VS(j)).^2).*tanh(dqTrain(:,j)/0.015) ...
        + SV(j).*dqTrain(:,j);
end
epsBasis = [0.03 0.06 0.12 0.25 0.5 1.0];
nFeat = numel(epsBasis)+1;
Theta = zeros(NJ,nFeat);
ridge = 1e-5;
for j = 1:NJ
    Phi = [tanh(dqTrain(:,j)./epsBasis), dqTrain(:,j)];
    Theta(j,:) = ((Phi'*Phi + ridge*eye(nFeat)) \ (Phi'*yTrain(:,j)))';
end
frictionModel = @(dq) structuredFriction(dq,Theta,epsBasis);

dqTest = -3 + 6*rand(2000,NJ);
yTest = zeros(size(dqTest)); yPred = zeros(size(dqTest));
for j = 1:NJ
    yTest(:,j) = FC(j).*tanh(dqTest(:,j)/0.015) + ...
        (FS(j)-FC(j)).*exp(-(dqTest(:,j)./VS(j)).^2).*tanh(dqTest(:,j)/0.015) ...
        + SV(j).*dqTest(:,j);
end
for r = 1:size(dqTest,1), yPred(r,:) = frictionModel(dqTest(r,:)')'; end
fprintf('Held-out friction RMSE [Nm]: %s\n\n', mat2str(sqrt(mean((yPred-yTest).^2,1)),4));

%% 4. Controller 1: PID (PD + gravity compensation)
t = (0:Nsim-1)*DT;
payload = double(t >= payloadTime)*payloadMass;
err = nan(3,Nsim);
qLog = nan(3,NJ,Nsim);
tauLog = nan(3,NJ,Nsim);
solverFailures = [0 0];

q = zeros(NJ,1); dq = zeros(NJ,1);
Kp = 120*ones(NJ,1);
Kd = 2*0.95*sqrt(Kp.*(I0+0.05));

for k = 1:Nsim
    pay = payload(k);
    tau = gravity(q,M) + Kp.*(qTarget-q) - Kd.*dq;
    tau = max(min(tau,tauMax),-tauMax);
    err(1,k) = norm(q-qTarget);
    qLog(1,:,k) = q'; tauLog(1,:,k) = tau';
    [q,dq] = plantStep(q,dq,tau,pay,DT,M,L,G,trueFriction);
end

%% 5. Controllers 2 & 3: NMPC and friction-aware PINN-NMPC
if hasCasadi
    try
        Sstd = buildNmpc(false,DT,5,M,L,G,Theta,epsBasis,tauMax);
        Spin = buildNmpc(true, DT,5,M,L,G,Theta,epsBasis,tauMax);
        solveEvery = 1;

        for kind = 1:2
            if kind==1, S = Sstd; ci = 2; else, S = Spin; ci = 3; end
            q = zeros(NJ,1); dq = zeros(NJ,1);
            Xwarm = repmat([q;dq],1,S.N+1);
            tauHold = max(min(gravity(q,M)+Kp.*(qTarget-q)-Kd.*dq,tauMax),-tauMax);
            Uwarm = repmat(tauHold,1,S.N);
            lastOK = false;

            for k = 1:Nsim
                pay = payload(k);
                x0 = [q;dq];

                if mod(k-1,solveEvery)==0
                    lastOK = false;
                    try
                        S.opti.set_value(S.P0,x0);
                        S.opti.set_value(S.PR,xRef);
                        S.opti.set_value(S.PAY,pay);
                        Xwarm(:,1) = x0;
                        S.opti.set_initial(S.X,Xwarm);
                        S.opti.set_initial(S.U,Uwarm);
                        sol = S.opti.solve();
                        Xsol = full(sol.value(S.X));
                        Usol = full(sol.value(S.U));
                        tauHold = max(min(Usol(:,1),tauMax),-tauMax);
                        Xwarm = [Xsol(:,2:end),Xsol(:,end)];
                        Uwarm = [Usol(:,2:end),Usol(:,end)];
                        lastOK = true;
                    catch ME
                        solverFailures(kind) = solverFailures(kind)+1;
                        Xwarm = repmat(x0,1,S.N+1);
                        Uwarm = repmat(tauHold,1,S.N);
                        if solverFailures(kind)==1
                            fprintf('Controller %d first solver failure; using fallback: %s\n',ci,ME.message);
                        end
                    end
                end

                % Fallback: stabilising feedback every sample if IPOPT failed
                if ~lastOK
                    tauHold = gravity(q,M) + Kp.*(qTarget-q) - Kd.*dq;
                    if kind==2, tauHold = tauHold + frictionModel(dq); end
                    tauHold = max(min(tauHold,tauMax),-tauMax);
                end

                err(ci,k) = norm(q-qTarget);
                qLog(ci,:,k) = q'; tauLog(ci,:,k) = tauHold';
                [q,dq] = plantStep(q,dq,tauHold,pay,DT,M,L,G,trueFriction);
            end
        end
    catch ME
        warning('Could not construct/run CasADi NMPC: %s',ME.message);
    end
else
    fprintf('CasADi not found: only PID will run. Put CasADi in /home/rajesh/kuka_robot/casadi\n');
end

%% 5b. Confirm the full simulated time was executed
cn = {'PID','Standard NMPC','Friction-aware PINN-NMPC'};
fprintf('\n=== Execution check (expected %d steps = %.2f s) ===\n',Nsim,T);
for c = 1:3
    nDone = nnz(isfinite(err(c,:)));
    if nDone > 0
        fprintf('%-26s %d/%d steps (%.2f s simulated)\n',cn{c},nDone,Nsim,nDone*DT);
    else
        fprintf('%-26s not run\n',cn{c});
    end
end

%% 6. Metrics
controllerNames = {'PID','Standard NMPC','Friction-aware PINN-NMPC'};
fprintf('\n=== Tracking benchmark ===\n');
for c = 1:3
    valid = isfinite(err(c,:));
    if any(valid)
        post = valid & (t>=payloadTime);
        fprintf('%-26s mean=%8.5f | peak=%8.5f | final=%8.5f | post-payload mean=%8.5f\n', ...
            controllerNames{c},mean(err(c,valid)),max(err(c,valid)), ...
            err(c,find(valid,1,'last')),mean(err(c,post)));
    end
end
if any(isfinite(err(2,:))) && any(isfinite(err(3,:)))
    mStd = mean(err(2,isfinite(err(2,:))));
    mPin = mean(err(3,isfinite(err(3,:))));
    fprintf('PINN-NMPC mean-error change vs NMPC: %+6.2f%% (positive = improvement)\n', ...
        100*(mStd-mPin)/mStd);
    fprintf('Solver failures -> NMPC: %d | PINN-NMPC: %d\n',solverFailures(1),solverFailures(2));
end

%% 7. Plots
validC = find(any(isfinite(err),2))';

figure('Name','Tracking error'); hold on;
for c = validC, plot(t,err(c,:),'LineWidth',1.4); end
xline(payloadTime,'k:','Payload applied'); grid on;
xlabel('Time [s]'); ylabel('||q - q_{ref}||_2 [rad]');
title('Tracking error'); legend(controllerNames(validC),'Location','best');

figure('Name','Joint 2 torque'); hold on;
for c = validC, plot(t,squeeze(tauLog(c,2,:)),'LineWidth',1.2); end
grid on; xlabel('Time [s]'); ylabel('Torque [Nm]');
title('Commanded torque, joint 2'); legend(controllerNames(validC),'Location','best');

figure('Name','Joint tracking');
for j = 1:NJ
    subplot(2,3,j); hold on;
    for c = validC, plot(t,squeeze(qLog(c,j,:)),'LineWidth',1.0); end
    yline(qTarget(j),'k:'); grid on;
    xlabel('Time [s]'); ylabel(sprintf('q%d [rad]',j)); title(sprintf('Joint %d',j));
    if j==1, legend([controllerNames(validC),{'target'}],'Location','best'); end
end

%% 8. 3D animation of the kinematic model
if opts.animate && hasRobotics
    animateRobots(robot,qLog,t,controllerNames,validC,payloadTime,qTarget,opts, outDir);
elseif opts.animate
    fprintf('Robotics System Toolbox not available: animation skipped.\n');
end
fprintf('\nFinished.\n');

%% ============================ Local functions ============================

function robot = buildKukaRobot()
    if exist('rigidBodyTree','class') ~= 8 && exist('rigidBodyTree','file') == 0
        robot = []; return;
    end
    robot = rigidBodyTree('DataFormat','column');

    radii        = [0.09, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02];
    bodyNames    = {'link_1','link_2','link_3','link_4','link_5','link_6'};
    parentNames  = {'base_link','link_1','link_2','link_3','link_4','link_5'};
    jointOffsets = [0 0 0.15; 0 0 0.25; 0 0 0.40; 0.35 0 0; 0.15 0 0; 0.10 0 0];
    jointAxes    = [0 0 1; 0 1 0; 0 1 0; 1 0 0; 0 1 0; 1 0 0];
    tipOffset    = [0.05 0 0];
    segVec       = [jointOffsets(2:6,:); tipOffset];

    baseLink = rigidBody('base_link');
    baseJoint = rigidBodyJoint('base_joint','fixed');
    setFixedTransform(baseJoint,trvec2tform([0 0 0]));
    baseLink.Joint = baseJoint;
    addVisual(baseLink,'Cylinder',[radii(1) jointOffsets(1,3)], ...
        trvec2tform([0 0 jointOffsets(1,3)/2]));
    addBody(robot,baseLink,'base');

    for i = 1:6
        body = rigidBody(bodyNames{i});
        joint = rigidBodyJoint(sprintf('joint_%d',i),'revolute');
        joint.JointAxis = jointAxes(i,:);
        joint.PositionLimits = [-pi pi];
        setFixedTransform(joint,trvec2tform(jointOffsets(i,:)));
        body.Joint = joint;

        d = segVec(i,:);
        addVisual(body,'Cylinder',[radii(i+1) norm(d)],segmentTform(d));
        addVisual(body,'Sphere',radii(i+1)*1.15,eye(4));
        addBody(robot,body,parentNames{i});
    end
end

function T = segmentTform(d)
    if abs(d(1)) > 0 && abs(d(3)) < 1e-12
        T = trvec2tform(d/2) * eul2tform([0 pi/2 0]);
    else
        T = trvec2tform(d/2);
    end
end

function exportKukaURDF(urdfPath)
    radii        = [0.09, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02];
    bodyNames    = {'link_1','link_2','link_3','link_4','link_5','link_6'};
    parentNames  = {'base_link','link_1','link_2','link_3','link_4','link_5'};
    jointOffsets = [0 0 0.15; 0 0 0.25; 0 0 0.40; 0.35 0 0; 0.15 0 0; 0.10 0 0];
    jointAxes    = [0 0 1; 0 1 0; 0 1 0; 1 0 0; 0 1 0; 1 0 0];
    tipOffset    = [0.05 0 0];
    segVec       = [jointOffsets(2:6,:); tipOffset];

    fid = fopen(urdfPath,'w');
    if fid < 0, warning('Cannot write URDF to %s',urdfPath); return; end
    fprintf(fid,'<?xml version="1.0"?>\n<robot name="custom_kuka_kr6">\n\n');
    fprintf(fid,['  <link name="base_link">\n    <visual>\n' ...
        '      <origin xyz="0 0 %.4f" rpy="0 0 0"/>\n' ...
        '      <geometry><cylinder length="%.4f" radius="%.4f"/></geometry>\n' ...
        '    </visual>\n  </link>\n\n'],jointOffsets(1,3)/2,jointOffsets(1,3),radii(1));
    for i = 1:6
        fprintf(fid,'  <joint name="joint_%d" type="revolute">\n',i);
        fprintf(fid,'    <parent link="%s"/>\n    <child link="%s"/>\n',parentNames{i},bodyNames{i});
        fprintf(fid,'    <origin xyz="%.4f %.4f %.4f" rpy="0 0 0"/>\n',jointOffsets(i,:));
        fprintf(fid,'    <axis xyz="%d %d %d"/>\n',jointAxes(i,:));
        fprintf(fid,'    <limit lower="-3.1416" upper="3.1416" effort="100" velocity="3.0"/>\n  </joint>\n\n');
        d = segVec(i,:);
        if abs(d(1)) > 0 && abs(d(3)) < 1e-12, rpyStr = '0 1.5708 0'; else, rpyStr = '0 0 0'; end
        fprintf(fid,['  <link name="%s">\n    <visual>\n' ...
            '      <origin xyz="%.4f %.4f %.4f" rpy="%s"/>\n' ...
            '      <geometry><cylinder length="%.4f" radius="%.4f"/></geometry>\n' ...
            '    </visual>\n  </link>\n\n'],bodyNames{i},d(1)/2,d(2)/2,d(3)/2,rpyStr,norm(d),radii(i+1));
    end
    fprintf(fid,'</robot>\n');
    fclose(fid);
    fprintf('URDF written: %s\n',urdfPath);
end

function animateRobots(robot,qLog,t,names,validC,payloadTime,qTarget,opts,outDir)
    nv = numel(validC);
    Nsim = numel(t);

    % End-effector (torch/flange) paths
    eePath = nan(3,Nsim,nv);
    for a = 1:nv
        for k = 1:Nsim
            Tf = getTransform(robot,reshape(qLog(validC(a),:,k),[],1),'link_6');
            eePath(:,k,a) = Tf(1:3,4);
        end
    end
    Tt = getTransform(robot,qTarget,'link_6');
    eeTarget = Tt(1:3,4);

    fig = figure('Name','KUKA KR6 animation','Color','w','Position',[80 80 450*nv 480]);
    ax = gobjects(nv,1); trail = gobjects(nv,1); ttl = gobjects(nv,1);
    for a = 1:nv
        ax(a) = subplot(1,nv,a);
        hold(ax(a),'on'); grid(ax(a),'on');
        plot3(ax(a),eeTarget(1),eeTarget(2),eeTarget(3),'rp','MarkerSize',12,'MarkerFaceColor','r');
        trail(a) = plot3(ax(a),nan,nan,nan,'b-','LineWidth',1.5);
        ttl(a) = title(ax(a),names{validC(a)},'Interpreter','none');
    end

    if opts.saveVideo
        vw = VideoWriter(fullfile(outDir,'kuka_nmpc_pinn.mp4'),'MPEG-4');
        vw.FrameRate = round(1/(t(2)-t(1)));
        open(vw);
    end

    for k = 1:opts.frameStride:Nsim
        if ~isvalid(fig), break; end
        for a = 1:nv
            qk = reshape(qLog(validC(a),:,k),[],1);
            show(robot,qk,'Parent',ax(a),'PreservePlot',false,'Visuals','on','Frames','off');
            set(trail(a),'XData',squeeze(eePath(1,1:k,a)), ...
                         'YData',squeeze(eePath(2,1:k,a)), ...
                         'ZData',squeeze(eePath(3,1:k,a)));
            axis(ax(a),[-0.9 0.9 -0.9 0.9 0 1.2]); axis(ax(a),'equal');
            axis(ax(a),[-0.9 0.9 -0.9 0.9 0 1.2]);
            view(ax(a),135,20);
            if t(k) >= payloadTime, pl = 'payload ON'; else, pl = 'no payload'; end
            title(ax(a),sprintf('%s\nt = %.2f s | %s',names{validC(a)},t(k),pl),'Interpreter','none');
        end
        drawnow;
        if opts.saveVideo, writeVideo(vw,getframe(fig)); end
    end
    if opts.saveVideo, close(vw); fprintf('Video saved in %s\n',outDir); end
end

function f = structuredFriction(dq,Theta,epsBasis)
    dq = dq(:);
    Phi = [tanh(dq./epsBasis),dq];
    f = sum(Theta.*Phi,2);
end

function [q,dq] = plantStep(q,dq,tau,payload,DT,M,L,G,trueFriction)
    m = M; m(6) = m(6)+payload;
    I = m.*L.^2;
    nSub = 5; h = DT/nSub;
    for ii = 1:nSub
        grav = m.*G.*L.*cos(q);
        ddq = (tau-grav-trueFriction(dq))./max(I,1e-5);
        dq = dq+h*ddq;
        q = q+h*dq;
    end
end

function S = buildNmpc(useFriction,dt,N,M,L,G,Theta,epsBasis,tauMax)
    import casadi.*
    opti = Opti();
    X = opti.variable(12,N+1);
    U = opti.variable(6,N);
    P0 = opti.parameter(12);
    PR = opti.parameter(12);
    PAY = opti.parameter(1);

    nSub = 2; hs = dt/nSub;
    x = MX.sym('x',12); u = MX.sym('u',6); pay = MX.sym('pay',1);
    mm = vertcat(DM(M(1:5)), DM(M(6)) + pay);
    II = mm.*DM(L).^2;
    xx = x;
    for sub = 1:nSub
        q = xx(1:6); dq = xx(7:12);
        grav = mm.*G.*DM(L).*cos(q);
        tf = MX.zeros(6,1);
        if useFriction
            for j = 1:6
                f = Theta(j,end)*dq(j);
                for b = 1:numel(epsBasis)
                    f = f+Theta(j,b)*tanh(dq(j)/epsBasis(b));
                end
                tf(j) = f;
            end
        end
        ddq = (u-grav-tf)./II;
        dqn = dq + hs*ddq;                 % same integrator as plantStep
        xx = [q + hs*dqn; dqn];
    end
    F = Function('F',{x,u,pay},{xx});

    % Reference torque = gravity holding torque at the target (with payload)
    mmP  = vertcat(DM(M(1:5)), DM(M(6)) + PAY);
    uRef = mmP.*G.*DM(L).*cos(PR(1:6));

    Q = diag(DM([250*ones(6,1); 3*ones(6,1)]));
    R = diag(DM(0.005*ones(6,1)));
    obj = 0;
    for k = 1:N
        e  = X(:,k)-PR;
        du = U(:,k)-uRef;
        obj = obj+e'*Q*e+du'*R*du;
        opti.subject_to(X(:,k+1)==F(X(:,k),U(:,k),PAY));
    end
    eN = X(:,N+1)-PR;
    obj = obj+8*(eN'*Q*eN);
    opti.minimize(obj);
    opti.subject_to(X(:,1)==P0);
    opti.subject_to(-tauMax<=U<=tauMax);

    p = struct('max_iter',300,'print_level',0,'tol',1e-4, ...
        'acceptable_iter',5,'acceptable_tol',1e-2);
    opti.solver('ipopt',struct('ipopt',p,'print_time',0));

    S = struct('opti',opti,'X',X,'U',U,'P0',P0,'PR',PR,'PAY',PAY,'N',N);
end
