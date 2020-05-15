%hw 4 p 2

clear all;
close all;
clc;

n = 2;
m = 1;
A = [1 1;
     0 1];
B = [0 1]';
Q = eye(n)';
R = 0.01;

[P_inf, G, E] = dare(A, B, Q, R);

%% p2b
x0 = [-4.5, 2]';
xf = [];
u_bar = 0.5;
x_bar = 5;
N = 3; 
P = eye(2);
R = 10;
T = 11;
%defined xo but not xf
[x_mat, u_mat, ~] = solve_MPC(A, B, x_bar, u_bar, P, Q, R, x0, true, xf, false, N, T);
% [x_mat, u_mat, goalReached] = solve_RHC_CVX(A, B, x_bar, u_bar, P, Q, R, x0, true, xf, false, N);
% x_mat = reshape(x,n,[]);
% u_mat = reshape(u,m,[]);
figure
% subplot(2,1,1)
plot(x_mat(1,:),x_mat(2,:))
% legend(strcat('x0 = ', num2str(x0')))
hold on
% plot(x_mat(2,:))
% subplot(2,1,2)
% plot(u_mat(1,:))

x0 = [-4.5, 3]';
[x_mat,u_mat, ~] = solve_MPC(A, B, x_bar, u_bar, P, Q, R, x0, true, xf, false, N, T);
% x_mat = reshape(x,n,[]);
% u_mat = reshape(u,m,[]);
% figure
% subplot(2,1,1)
plot(x_mat(1,:),x_mat(2,:))
legend(strcat('x0 = ', '-4.5, 2'), strcat('x0 = ', '-4.5, 4'))
% hold on
% plot(x_mat(2,:))
% subplot(2,1,2)
% plot(u_mat(1,:))

% %% p2 c
% x_bar = 10;
% u_bar = 1;
% N = 2;
% P = P_inf;
% xf = [0 0]';
% res = 0.5;
% %loop through discritized options for state to solve for positive, control
% %invariant set approximation
% x_span = 5;
% x1Disc = -x_span:res:x_span;
% x2Disc = -x_span:res:x_span;
% domainAttract = [];
% figure;
% hold on
% for i = 1:length(x1Disc)
%     for j = 1:length(x2Disc)
%          x0 = [x1Disc(i) x2Disc(j)]';
% %          x0 = [0 2]';
%          [x_mat, u_mat, goalReached] = solve_RHC_CVX(A, B, x_bar, u_bar, P, Q, R, x0, true, xf, true, N);
%          if goalReached
%              scatter(x0(1),x0(2),'b')
%          else
%              scatter(x0(1),x0(2),'r')
%          end
%     end
% end
% % figure
% plot(x(1,:),x(2,:))
% figure
% plot(domainAttract(1,:),domainAttract(2,:))