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
%% p2 d
x_bar = 10;
u_bar = 1;
N = 6;
T = 20;
P = P_inf;
xf = [0 0]';
res = 1;
%loop through discritized options for state to solve for positive, control
%invariant set approximation
x_span = 10;
x1Disc = -x_span:res:x_span;
x2Disc = -x_span:res:x_span;
domainAttract = [];
figure;
hold on
for i = 1:length(x1Disc)
    for j = 1:length(x2Disc)
         x0 = [x1Disc(i) x2Disc(j)]';
%          x0 = [0 2]';
         [x_mat, u_mat, goalReached] = solve_MPC(A, B, x_bar, u_bar, P, Q, R, x0, true, xf, false, N, T);
         if goalReached
             scatter(x0(1),x0(2),'b')
         else
             scatter(x0(1),x0(2),'r')
         end
    end
end
% fig