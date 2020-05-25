%call solve_RHC_CVX to determine optimal control and state trajectory
%return state history, control history, and if final goal reached
function [x_hist, u_hist, isAtGoalFinal] = solve_MPC(A, B, x_bar, u_bar, Qf, Q, R, x0, x0isDef, xf, xfisDef, N, T)
isAtGoalFinal = false;
tol = 0.0001;
%solve RHC problem using CVX at each timestep
x_hist = x0;
u_hist = [];
for t = 1:T
    [x, u, isAtGoal, cvx_status] = solve_RHC_CVX(A, B, x_bar, u_bar, Qf, Q, R, x_hist(:,t), x0isDef, xf, xfisDef, N);
    if strcmp(cvx_status, 'Solved')
        x_hist(:,t+1) = A*x(:,1) + B*u(:,1);
        u_hist(:,t) = u(:,1);
        if ~xfisDef %break if succesfully solved opt and no goal set
            isAtGoalFinal = true;
%             break
        elseif xfisDef && (norm(x_hist(:,t+1) - xf) <= tol) %break if solved opt prob and at goal
            isAtGoalFinal = true;
            break
        end
    else %break if opt prob not solved (nan returned)
        break;
    end
end


end