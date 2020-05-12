% Function providing equality and inequality constraints
% ceq(var) = 0 and c(var) \le 0 

function [c,ceq] = constraint(var)

global N;
global T;

global x0;
global y0;

% Note that var = [x;y;u]
x = var(1:N+1); y = var(N+2:2*N+2); u = var(2*N+3:3*N+3);

% Put here constraint inequalities
c = [-x;
    x - 1;
    -y;
    y - 1;
    -u; %-u < 0
    u - 1]; %u-1 < 0

% Computing dynamical constraints via the trapezoidal rule
h = 1.0*T/(1.0*N);
for i = 1:N
    % Provide here dynamical constraints via the trapeziodal formula
    [x_dot_i, y_dot_i] = fDyn(x(i),y(i),u(i));
    [x_dot_ip, y_dot_ip] = fDyn(x(i+1),y(i+1),u(i+1));
    ceq(i) = x(i+1) - x(i) -h*(x_dot_ip + x_dot_i)/2;
    ceq(i+N) =  y(i+1) - y(i) -h*(y_dot_ip + y_dot_i)/2;
end

% Put here initial conditions
ceq(1+2*N) = x(1) - 1;
ceq(2+2*N) = y(1);