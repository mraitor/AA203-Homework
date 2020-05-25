%hw4_p4
clear all
close all
clc
tic;
model = LTISystem('A', [0.99 1; 0 0.99], 'B', [0; 1]);
model.x.min = [-5; -5];
model.x.max = [5; 5];
model.u.min = -0.5;
model.u.max = 0.5;
model.x.penalty = QuadFunction(eye(2));
model.u.penalty = QuadFunction(1);
% add LQR terminal set and terminal penalty
model.x.with('terminalSet');
model.x.terminalSet = model.LQRSet();
model.x.with('terminalPenalty');
model.x.terminalPenalty = model.LQRPenalty();
%% 4a
model.x.terminalSet.plot()

%% 4b
N = 4; % prediction horizon
x0 = [-1.5, 0.5]';
ctrl = MPCController(model, N)
% and simulated for zero initial conditions over 30 samples
loop = ClosedLoop(ctrl, model);
Nsim = 30;
%timers
setupIMPC = toc;
tic;

data = loop.simulate(x0, Nsim);
simIMPC = toc;

figure;
subplot(2,1,1)
plot(1:(Nsim+1), data.X);
% hold on;
% plot(1:Nsim, ys*ones(1, Nsim), 'k--')
title('state')
subplot(2,1,2)
plot(1:Nsim, data.U);
% hold on;
% plot(1:Nsim, us*ones(1, Nsim), 'k--')
title('inputs')
%% 4c
% optimoptions(MPTOPTIONS, 'Algorithm', 'interior-point-convex');
% mpt_options('qpsolver','interior-point-convex');

% P = Polyhedron('lb', [-10; -10], 'ub', [10; 10]);
% model.x.with('setConstraint');
% model.x.setConstraint = P;
% ctrl.qpsolver


tic;
options = mptopt();
options.modules.solvers.quadprog.Algorithm = 'interior-point-convex';
empc = MPCController(model, N).toExplicit();

% create a closed-loop system
loop_exp = ClosedLoop(empc, model);

additionalSetupTimeEMPC = toc;
tic;
% % convert the closed-loop system into an autonomous PWA system
data_exp = loop_exp.simulate(x0, Nsim);
simTimeEMPC = toc;

figure;
subplot(2,1,1)
plot(1:(Nsim+1), data_exp.X);
% hold on;
% plot(1:Nsim, ys*ones(1, Nsim), 'k--')
title('state')
subplot(2,1,2)
plot(1:Nsim, data_exp.U);
% hold on;
% plot(1:Nsim, us*ones(1, Nsim), 'k--')
title('inputs')

figure
empc.partition.plot()