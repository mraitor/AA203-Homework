%hw2 p2a - use CVX to solce RHC problems
%Note: I drew heavily from provided code from SCP in problem 1!
function [x, u, isAtGoal] = solve_RHC_CVX(A, B, x_bar, u_bar, Qf, Q, R, x0, N)
num_steps = N-1;
isAtGoal = false;
n = size(Q,1); % get the state dimension
m = size(R,1); % get the control dimension
u_shift = n*N; % what is the last index of z that represents a state? (This is used for indexing purposes).

x_start = @(i) (i-1)*n + 1; % get the first entry of x_i in the vector z
x_end = @(i) i*n; % get the last entry of x_i in the vector z
u_start = @(i) u_shift + (i-1)*m + 1; % get the first entry of u_i in the vector z
u_end = @(i) u_shift + i*m; % get the last entry of u_i in the vector z

M = zeros((n+m)*N);
%build cost matrix
%takes the form of z = [x1...xn,u1...un]' in column matrix
%thus cost = z'Mz, z is design variable solved for in CVX further down
for i=1:N
    if (i < N)
        M(x_start(i): x_end(i), x_start(i): x_end(i)) = Q;
    else
        M(x_start(i): x_end(i), x_start(i): x_end(i)) = Qf;
    end
    M(u_start(i): u_end(i), u_start(i): u_end(i)) = R;
end

%build constraint matrix
num_constr = n*num_steps;
% constraints will have the form C*x == d
C = zeros(num_constr, (n+m)*N);
d = zeros(num_constr,1);
for i = 1:num_steps %for N time points, N-1 constraints linking their intermediate dynamics
    C(x_start(i):x_end(i), x_start(i):x_end(i)) = -A;
    C(x_start(i):x_end(i), u_start(i):u_end(i)) = -B;
    C(x_start(i):x_end(i), x_start(i+1):x_end(i+1)) = eye(n);
    d(x_start(i):x_end(i)) = zeros(n,1);
end
% initial condition constraint
% if x0 defined, make it a part of the constrint matrix
if exist('x0','var')
    [nRowsC, nColsC] = size(C);
    C = vertcat(C, zeros(n,nColsC)); %add zeros to bottom of c to append ic constraint
    C(nRowsC+1:nRowsC+n, x_start(1):x_end(1)) = eye(n);
    d(nRowsC+1:nRowsC+n) = x0; %start at x0
end
%     %final condition constraint
%     %if xf defined, make it part of the constraint matrix
if exist('xf','var')
    [nRowsC, nColsC] = size(C);
    C = vertcat(C, zeros(n,nColsC)); %add zeros to bottom of c to append ic constraint
    C(nRowsC+1:nRowsC+n, x_start(num_steps):x_end(num_steps)) = eye(n);
    d(nRowsC+1:nRowsC+n) = xf; %start at x0
end

%construct inequality bounds for u and x
lb = zeros((n+m)*N,1);
ub = zeros((n+m)*N,1);
lb(x_start(1):x_end(N)) = -x_bar;
ub(x_start(1):x_end(N)) = x_bar;
lb(u_start(1):u_end(N)) = -u_bar;
ub(u_start(1):u_end(N)) = u_bar;

%% Build CVX instance of our optimization problem
cvx_begin quiet

variable z((n+m)*N); % our decision variable will be z = [x,u] a concatenation of the state and control variables.
cost = quad_form(z,M);

minimize(cost)
subject to
C*z == d; %linearixed dynamics constraints
lb <= z <= ub; % control and state bounds
% max(abs(z - z_old)) <= 0.5;
cvx_end

var = z; % retrieve the optimal solution from CVX

x = var(1:n*N); % split z back into x and u
u = var(u_shift+1:(n+m)*N);

% If you want, you can uncomment the next line to see the objective
% value for this iteration.
fprintf("Current Objective value: %f \n" ,(var)'*M*(var))

end