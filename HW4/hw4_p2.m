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
u_bar = 0.5;
x_bar = 5;
N = 3; 
P = eye(2);
R = 10;
[x,u, ~] = solve_RHC_CVX(A, B, x_bar, u_bar, P, Q, R, x0, N);
x_mat = reshape(x,n,[]);
u_mat = reshape(u,m,[]);
figure
subplot(2,1,1)
plot(x_mat(1,:))
suptitle(strcat('x0 = ', num2str(x0')))
hold on
plot(x_mat(2,:))
subplot(2,1,2)
plot(u_mat(1,:))

x0 = [-4.5, 3]';
[x,u, ~] = solve_RHC_CVX(A, B, x_bar, u_bar, P, Q, R, x0, N);
x_mat = reshape(x,n,[]);
u_mat = reshape(u,m,[]);
figure
subplot(2,1,1)
plot(x_mat(1,:))
suptitle(strcat('x0 = ', num2str(x0')))
hold on
plot(x_mat(2,:))
subplot(2,1,2)
plot(u_mat(1,:))

