%% KUKA KR6-style 6-DOF tracking comparison: PID vs NMPC vs friction-aware NMPC
% File: kuka_kr6_controller_comparison_improved.m
%
% IMPORTANT:
% This is an illustrative 6-DOF simulation, NOT a manufacturer-validated
% KUKA KR6 model and NOT suitable for direct deployment on a real robot.
% Validate dynamics, units, limits, and safety independently before hardware use.
%
% Requires MATLAB. CasADi + IPOPT are required for the two NMPC cases.
%
% Cases:
%   PID, Standard NMPC, Friction-aware NMPC; each with plant friction OFF/ON.
%   Standard NMPC predicts with no friction.
%   Friction-aware NMPC uses a physics-structured friction regression.
%   That regression is NOT a neural network, so "PINN-NMPC" would be a
%   misleading name for this implementation.
%
% Main improvements over the earlier version:
%   1) NMPC receives a future reference trajectory, not one frozen reference.
%   2) NMPC receives payload preview over its horizon.
%   3) Torque saturation and torque-rate limits are applied consistently to
%      PID and NMPC; the optimizer uses the same rate limit as the plant input.
%   4) Warm starts are shifted and re-anchored at the measured state.
%   5) Friction regression uses deterministic, well-conditioned training data.
%   6) Solver success, failures, and fallback steps are counted separately.
%   7) Reports RMSE and post-payload metrics in addition to peak/final error.
%
% The model remains intentionally simplified: independent joint dynamics,
% diagonal effective inertia, and a lumped payload effect on joint 6.

clear; clc; close all;
rng(7, 'twister');

%% 1. Simulation settings and simplified plant
NJ = 6;
G = 9.81;
DT = 0.02;
T = 4.0;
Nsim = round(T/DT);
t = (0:Nsim-1)*DT;

M = [3.0; 4.0; 3.5; 2.0; 1.5; 0.8];
L = [0.4; 0.4; 0.3; 0.2; 0.15; 0.1];
INERTIA_SCALE = 4.0;
I0 = INERTIA_SCALE*M.*L.^2;

payloadTime = 0.75;
payloadMass = 4.0;
payload = double(t >= payloadTime)*payloadMass;

qTarget = [0.3; -0.4; 0.5; -0.2; 0.4; 0.2];
rampTime = 0.60;
qRefLog = zeros(NJ,Nsim);
dqRefLog = zeros(NJ,Nsim);
for k = 1:Nsim
    r = min(max(t(k)/rampTime,0),1);
    s = 3*r^2 - 2*r^3;
    ds = (6*r - 6*r^2)/rampTime;
    qRefLog(:,k) = s*qTarget;
    dqRefLog(:,k) = ds*qTarget;
end
refLog = [qRefLog; dqRefLog];

tauMax = 30;                 % Nm
maxTorqueRate = 300;          % Nm/s, shared by every controller
maxTorqueStep = maxTorqueRate*DT;

% Simplified gravity and friction model
gravity = @(q,m) m.*G.*L.*cos(q);
FC = [1.5; 2.0; 1.2; 0.8; 0.5; 0.3];
FS = [2.5; 3.2; 2.0; 1.4; 0.9; 0.5];
VS = [0.10; 0.10; 0.08; 0.05; 0.05; 0.05];
SV = [0.8; 1.1; 0.6; 0.4; 0.3; 0.2];
trueFriction = @(dq) FC.*tanh(dq/0.015) + ...
    (FS-FC).*exp(-(dq./VS).^2).*tanh(dq/0.015) + SV.*dq;

fprintf('KUKA KR6-style simulation: %d samples, %.2f s, DT=%.3f s\n',Nsim,T,DT);
fprintf('Payload step: %.1f kg at t=%.2f s\n',payloadMass,payloadTime);
fprintf('Shared torque limits: +/- %.1f Nm, %.1f Nm/s rate limit\n',tauMax,maxTorqueRate);

%% 2. Fit physics-structured friction regression
% Use a deterministic, broad velocity grid rather than random training data.
% This makes runs reproducible and samples low-speed Stribeck behavior densely.
nTrain = 3001;
dqGrid = linspace(-3,3,nTrain)';
epsBasis = [0.01 0.02 0.04 0.08 0.16 0.32 0.64 1.0];
nFeat = numel(epsBasis)+2;
Theta = zeros(NJ,nFeat);
ridge = 1e-7;

for j = 1:NJ
    y = FC(j).*tanh(dqGrid/0.015) + ...
        (FS(j)-FC(j)).*exp(-(dqGrid./VS(j)).^2).*tanh(dqGrid/0.015) + ...
        SV(j).*dqGrid;
    Phi = [tanh(dqGrid./epsBasis), ...
        exp(-(dqGrid./VS(j)).^2).*tanh(dqGrid/0.015), dqGrid];
    % Scale columns before ridge regression to improve conditioning.
    colScale = max(vecnorm(Phi,2,1),1e-12);
    PhiScaled = Phi./colScale;
    thetaScaled = (PhiScaled'*PhiScaled + ridge*eye(nFeat)) \ (PhiScaled'*y);
    Theta(j,:) = (thetaScaled'./colScale);
end
frictionModel = @(dq) structuredFriction(dq,Theta,epsBasis,VS);

% Quantify fit quality against the friction law used by the simulated plant.
fitRMSE = zeros(NJ,1);
for j = 1:NJ
    Phi = [tanh(dqGrid./epsBasis), ...
        exp(-(dqGrid./VS(j)).^2).*tanh(dqGrid/0.015), dqGrid];
    yhat = Phi*Theta(j,:)';
    ytrue = FC(j).*tanh(dqGrid/0.015) + ...
        (FS(j)-FC(j)).*exp(-(dqGrid./VS(j)).^2).*tanh(dqGrid/0.015) + SV(j).*dqGrid;
    fitRMSE(j) = sqrt(mean((yhat-ytrue).^2));
end
fprintf('Structured friction-fit RMSE by joint [Nm]: ');
fprintf('%.4g ',fitRMSE); fprintf('\n');

%% 3. Controller settings
controllerNames = {'PID','Standard NMPC','Friction-aware NMPC'};
plantNames = {'Friction OFF','Friction ON'};
nControllers = numel(controllerNames);
nConditions = numel(plantNames);

Kp = 18*ones(NJ,1);
Kd = 2.2*sqrt(Kp.*I0);
Ki = 0.25*ones(NJ,1);
iMax = 5;

Nhor = 12;  % 0.24 s preview; longer horizons increase IPOPT cost
err = nan(nControllers,nConditions,Nsim);
qLog = nan(nControllers,nConditions,NJ,Nsim);
dqLog = nan(nControllers,nConditions,NJ,Nsim);
tauLog = nan(nControllers,nConditions,NJ,Nsim);
solverFailures = zeros(nControllers,nConditions);
solverSuccesses = zeros(nControllers,nConditions);
fallbackSteps = zeros(nControllers,nConditions);

hasCasadi = (exist('casadi.Opti','class') == 8) || (exist('Opti','class') == 8);
if ~hasCasadi
    fprintf('\nCasADi not detected: PID cases will run; NMPC cases will remain unavailable.\n');
    fprintf('Add CasADi to the MATLAB path and ensure its IPOPT plugin is available.\n');
end

%% 4. Run all six controller/plant combinations
for cond = 1:nConditions
    plantHasFriction = (cond == 2);
    if plantHasFriction
        plantFriction = trueFriction;
    else
        plantFriction = @(dq) zeros(size(dq));
    end

    % ----- PID with the same saturation and torque-rate limits as NMPC -----
    q = zeros(NJ,1); dq = zeros(NJ,1); intErr = zeros(NJ,1);
    previousAppliedTorque = gravity(q,M); % assumed pre-run gravity-hold command
    for k = 1:Nsim
        qr = qRefLog(:,k); dqr = dqRefLog(:,k);
        intErr = max(min(intErr + Ki.*(qr-q)*DT,iMax),-iMax);
        tauRaw = gravity(q,M) + Kp.*(qr-q) + Kd.*(dqr-dq) + intErr;
        tau = applyActuatorLimits(tauRaw,previousAppliedTorque,tauMax,maxTorqueStep);
        err(1,cond,k) = norm(q-qr);
        qLog(1,cond,:,k) = q';
        dqLog(1,cond,:,k) = dq';
        tauLog(1,cond,:,k) = tau';
        previousAppliedTorque = tau;
        [q,dq] = plantStep(q,dq,tau,payload(k),DT,M,L,G,INERTIA_SCALE,plantFriction);
    end

    if ~hasCasadi
        continue;
    end

    % ----- Standard NMPC and friction-aware NMPC -----
    for ctrl = 2:3
        useFrictionModel = (ctrl == 3) && plantHasFriction;
        try
            S = buildNmpc(useFrictionModel,DT,Nhor,M,L,G,INERTIA_SCALE, ...
                Theta,epsBasis,VS,tauMax,maxTorqueStep);
            q = zeros(NJ,1); dq = zeros(NJ,1);
            previousAppliedTorque = gravity(q,M);
            Xwarm = repmat([q;dq],1,Nhor+1);
            Uwarm = repmat(previousAppliedTorque,1,Nhor);

            for k = 1:Nsim
                x0 = [q;dq];
                idx = min(k+(0:Nhor),Nsim);
                refPreview = refLog(:,idx);
                payloadPreview = payload(min(k+(0:Nhor-1),Nsim));

                solveOK = false;
                try
                    S.opti.set_value(S.P0,x0);
                    S.opti.set_value(S.PR,refPreview);
                    S.opti.set_value(S.PAY,payloadPreview);
                    S.opti.set_value(S.UPREV,previousAppliedTorque);
                    Xwarm(:,1) = x0;
                    S.opti.set_initial(S.X,Xwarm);
                    S.opti.set_initial(S.U,Uwarm);
                    sol = S.opti.solve_limited();
                    Xsol = full(sol.value(S.X));
                    Usol = full(sol.value(S.U));
                    if any(~isfinite(Xsol(:))) || any(~isfinite(Usol(:)))
                        error('IPOPT returned non-finite decision variables.');
                    end
                    tauCandidate = Usol(:,1);
                    solveOK = true;
                    solverSuccesses(ctrl,cond) = solverSuccesses(ctrl,cond)+1;
                    Xwarm = [Xsol(:,2:end),Xsol(:,end)];
                    Uwarm = [Usol(:,2:end),Usol(:,end)];
                catch MEsolve
                    solverFailures(ctrl,cond) = solverFailures(ctrl,cond)+1;
                    if solverFailures(ctrl,cond) == 1
                        fprintf('%s / %s first solve failure: %s\n', ...
                            controllerNames{ctrl},plantNames{cond},MEsolve.message);
                    end
                end

                if solveOK
                    tauRaw = tauCandidate;
                else
                    fallbackSteps(ctrl,cond) = fallbackSteps(ctrl,cond)+1;
                    qr = qRefLog(:,k); dqr = dqRefLog(:,k);
                    tauRaw = gravity(q,M) + Kp.*(qr-q) + Kd.*(dqr-dq);
                    if useFrictionModel
                        tauRaw = tauRaw + frictionModel(dq);
                    end
                    Xwarm = repmat(x0,1,Nhor+1);
                    Uwarm = repmat(previousAppliedTorque,1,Nhor);
                end

                % Optimizer and fallback both pass through exactly the same limiter.
                tau = applyActuatorLimits(tauRaw,previousAppliedTorque,tauMax,maxTorqueStep);
                err(ctrl,cond,k) = norm(q-qRefLog(:,k));
                qLog(ctrl,cond,:,k) = q';
                dqLog(ctrl,cond,:,k) = dq';
                tauLog(ctrl,cond,:,k) = tau';
                previousAppliedTorque = tau;
                [q,dq] = plantStep(q,dq,tau,payload(k),DT,M,L,G,INERTIA_SCALE,plantFriction);
            end
        catch MEbuild
            warning('Could not run %s / %s: %s', ...
                controllerNames{ctrl},plantNames{cond},MEbuild.message);
        end
    end
end

%% 5. Metrics
fprintf('\n================ TRACKING METRICS ================\n');
fprintf('%-22s %-13s %10s %10s %10s %10s %10s %10s\n', ...
    'Controller','Plant','Mean','RMSE','Peak','Final','Post mean','Post RMS');
for ctrl = 1:nControllers
    for cond = 1:nConditions
        e = squeeze(err(ctrl,cond,:));
        valid = isfinite(e);
        if ~any(valid)
            fprintf('%-22s %-13s %s\n',controllerNames{ctrl},plantNames{cond},'NOT RUN');
            continue;
        end
        ev = e(valid);
        postMask = valid(:) & (t(:)>=payloadTime);
        ep = e(postMask); ep = ep(isfinite(ep));
        if isempty(ep), postMean = NaN; postRMS = NaN;
        else, postMean = mean(ep); postRMS = sqrt(mean(ep.^2));
        end
        fprintf('%-22s %-13s %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n', ...
            controllerNames{ctrl},plantNames{cond},mean(ev),sqrt(mean(ev.^2)), ...
            max(ev),ev(end),postMean,postRMS);
    end
end

if hasCasadi
    fprintf('\nIPOPT accounting (friction OFF / ON):\n');
    for ctrl = 2:3
        fprintf('%s: failures %d/%d | successful solves %d/%d | fallback steps %d/%d\n', ...
            controllerNames{ctrl},solverFailures(ctrl,1),solverFailures(ctrl,2), ...
            solverSuccesses(ctrl,1),solverSuccesses(ctrl,2), ...
            fallbackSteps(ctrl,1),fallbackSteps(ctrl,2));
    end
end

%% 6. Plot all tracking-error cases
styles = {'-','--'};
figure('Name','Improved tracking error comparison','Color','w');
hold on; grid on;
for ctrl = 1:nControllers
    for cond = 1:nConditions
        e = squeeze(err(ctrl,cond,:));
        if any(isfinite(e))
            plot(t,max(e,1e-8),'LineStyle',styles{cond},'LineWidth',1.7, ...
                'DisplayName',[controllerNames{ctrl} ' | ' plantNames{cond}]);
        end
    end
end
xline(payloadTime,'k:','Payload applied','LabelVerticalAlignment','bottom');
set(gca,'YScale','log');
xlabel('Time [s]'); ylabel('Tracking error norm [rad] (display floor = 1e-8)');
title('Tracking error: controller and friction comparison');
legend('Location','best'); ylim([1e-8 2]);

%% 7. Joint tracking
figure('Name','Improved joint tracking','Color','w');
for j = 1:NJ
    subplot(2,3,j); hold on; grid on;
    for ctrl = 1:nControllers
        for cond = 1:nConditions
            qj = squeeze(qLog(ctrl,cond,j,:));
            if any(isfinite(qj))
                plot(t,qj,'LineStyle',styles{cond},'LineWidth',1.0, ...
                    'DisplayName',[controllerNames{ctrl} ' | ' plantNames{cond}]);
            end
        end
    end
    plot(t,qRefLog(j,:),'k:','LineWidth',1.2,'DisplayName','Reference');
    xline(payloadTime,'k:');
    xlabel('Time [s]'); ylabel(sprintf('q_%d [rad]',j));
    title(sprintf('Joint %d',j));
    if j==1, legend('Location','best','FontSize',6); end
end
sgtitle('Joint tracking: solid = friction OFF, dashed = friction ON');

%% 8. Torque tracking: all six joints in one figure
% One 2-by-3 tiled figure; each panel compares all controllers and both
% plant-friction conditions for that joint.
figure('Name','Improved torque comparison: all joints','Color','w');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
for j = 1:NJ
    nexttile; hold on; grid on;
    for ctrl = 1:nControllers
        for cond = 1:nConditions
            tauj = squeeze(tauLog(ctrl,cond,j,:));
            if any(isfinite(tauj))
                plot(t,tauj,'LineStyle',styles{cond},'LineWidth',1.2, ...
                    'DisplayName',[controllerNames{ctrl} ' | ' plantNames{cond}]);
            end
        end
    end
    xline(payloadTime,'k:','HandleVisibility','off');
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d torque [Nm]',j));
    title(sprintf('Joint %d',j));
    if j == 1
        legend('Location','best','FontSize',6);
    end
end
sgtitle('Commanded torque for all six joints: solid = friction OFF, dashed = friction ON');

fprintf('\nFinished. Solid = friction OFF; dashed = friction ON.\n');
fprintf('Reminder: results apply only to this simplified illustrative model.\n');

%% ============================ Local functions ============================
function tau = applyActuatorLimits(tauRaw,tauPrev,tauMax,maxTorqueStep)
    tauSat = max(min(tauRaw,tauMax),-tauMax);
    tau = max(min(tauSat,tauPrev+maxTorqueStep),tauPrev-maxTorqueStep);
    tau = max(min(tau,tauMax),-tauMax);
end

function f = structuredFriction(dq,Theta,epsBasis,VS)
    dq = dq(:);
    f = zeros(6,1);
    for j = 1:6
        phi = [tanh(dq(j)./epsBasis), ...
            exp(-(dq(j)./VS(j)).^2).*tanh(dq(j)/0.015), dq(j)];
        f(j) = Theta(j,:)*phi';
    end
end

function [q,dq] = plantStep(q,dq,tau,payload,DT,M,L,G,inertiaScale,frictionFcn)
    m = M; m(6) = m(6)+payload;
    I = max(inertiaScale*m.*L.^2,1e-3);
    grav = m.*G.*L.*cos(q);
    fric = frictionFcn(dq);
    ddq = (tau-grav-fric)./I;
    dq = dq+DT*ddq;
    q = q+DT*dq;
end

function S = buildNmpc(useFriction,dt,N,M,L,G,inertiaScale,Theta,epsBasis,VS,tauMax,maxTorqueStep)
    import casadi.*
    opti = Opti();
    X = opti.variable(12,N+1);
    U = opti.variable(6,N);
    P0 = opti.parameter(12);
    PR = opti.parameter(12,N+1);  % previewed [q_ref; dq_ref] at every stage
    PAY = opti.parameter(1,N);    % previewed payload for each predicted interval

    x = MX.sym('x',12); u = MX.sym('u',6); pay = MX.sym('pay',1);
    mm = vertcat(DM(M(1:5)),DM(M(6))+pay);
    II = inertiaScale*mm.*DM(L).^2;
    q = x(1:6); dq = x(7:12);
    grav = mm.*G.*DM(L).*cos(q);
    tf = MX.zeros(6,1);
    if useFriction
        for j = 1:6
            fj = Theta(j,end)*dq(j) + ...
                Theta(j,numel(epsBasis)+1)*exp(-(dq(j)/VS(j))^2)*tanh(dq(j)/0.015);
            for b = 1:numel(epsBasis)
                fj = fj + Theta(j,b)*tanh(dq(j)/epsBasis(b));
            end
            tf(j) = fj;
        end
    end
    ddq = (u-grav-tf)./II;
    dqn = dq+dt*ddq;
    xn = [q+dt*dqn;dqn];
    F = Function('F',{x,u,pay},{xn});

    Q = diag(DM([500*ones(6,1);25*ones(6,1)]));
    R = diag(DM(0.02*ones(6,1)));
    Rd = diag(DM(0.15*ones(6,1)));
    obj = 0;
    Uprev = opti.parameter(6);
    UPREV = Uprev;
    opti.subject_to(X(:,1)==P0);

    for k = 1:N
        refk = PR(:,k);
        e = X(:,k)-refk;
        payk = PAY(k);
        mmk = vertcat(DM(M(1:5)),DM(M(6))+payk);
        uRef = mmk.*G.*DM(L).*cos(refk(1:6));
        du = U(:,k)-uRef;
        dU = U(:,k)-Uprev;
        obj = obj + e'*Q*e + du'*R*du + dU'*Rd*dU;
        opti.subject_to(-maxTorqueStep <= dU <= maxTorqueStep);
        opti.subject_to(X(:,k+1)==F(X(:,k),U(:,k),payk));
        Uprev = U(:,k);
    end
    eN = X(:,N+1)-PR(:,N+1);
    obj = obj + 8*(eN'*Q*eN);
    opti.minimize(obj);
    opti.subject_to(-tauMax<=U<=tauMax);

    p = struct('max_iter',100,'print_level',0,'tol',1e-4, ...
        'acceptable_iter',5,'acceptable_tol',1e-2, ...
        'acceptable_obj_change_tol',1e-5);
    opti.solver('ipopt',struct('ipopt',p,'print_time',0));
    S = struct('opti',opti,'X',X,'U',U,'P0',P0,'PR',PR,'PAY',PAY, ...
        'UPREV',UPREV,'N',N);
end