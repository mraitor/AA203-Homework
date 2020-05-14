%call solve_RHC_CVX to determine optimal control and state trajectory
%return state history, control history, and if final goal reached
function [x_hist, u_hist, isAtGoalFinal] = solve_MPC(A, B, x_bar, u_bar, Qf, Q, R, x0, x0isDef, xf, xfisDef, N, T)
isAtGoalFinal = false;
tol = 0.0001;
%solve RHC problem using CVX at each timestep
x_hist = x0;
for t = 1:T
    [x, u, isAtGoal] = solve_RHC_CVX(A, B, x_bar, u_bar, Qf, Q, R, x_hist(:,t), x0isDef, xf, xfisDef, N);
    x_hist(:,t+1) = A*x(:,1) + B*u(:,1);
    u_hist(:,t) = u(:,1);
    if xfisDef && (t == T) && (norm(x_hist(:,t+1) - xf) <= tol)
        isAtGoalFinal = true;
    end
end


end