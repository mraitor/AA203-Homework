%hw 4 p 2f

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
%% p2 h
x_bar = 10;
u_bar = 1;
% N = 6;
T = 20;
P = P_inf;
xf = [0 0]';
res = 1;
%loop through discritized options for state to solve for positive, control
%invariant set approximation
N_list = [8 7 6 5 4];
figure;
hold on
for i = 1:length(N_list)
%     x0 = [-4.5, 2]';
    x0 = [-9, 3]';
    N = N_list(i);
    %          x0 = [0 2]';
    [x_mat, u_mat, goalReached] = solve_MPC(A, B, x_bar, u_bar, P, Q, R, x0, true, xf, true, N, T);
    if goalReached
        cost(i) = dot(x_mat(:,end),(P-Q)*x_mat(:,end)) + sum(dot(x_mat,Q*x_mat)) + dot(u_mat,R*u_mat);
    else
        cost(i) = nan;
    end
    plot(x_mat(1,:), x_mat(2,:));
end
disp('Cost = ')
disp(cost)
% fig