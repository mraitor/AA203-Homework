function [L, P] = lqr_infinite_horizon_solution(Q, R)

%% find the infinite horizon L and P through running LQR back-ups
%%   until norm(L_new - L_current, 2) <= 1e-4  
dt = 0.1;
mc = 10; mp = 2.; l = 1.; g= 9.81;

% TODO write A,B matrices
a1 =  mp*g/mc;
a2 = (mc +mp)*g/(l*mc);
dfds = [0 0 1 0;
        0 0 0 1;
        0 a1 0 0;
        0 a2 0 0];
dfdu = [0 0 1/mc 1/(l*mc)]';
A = eye(4) + dfds;
B = dt*dfdu;
% TODO implement Riccati recursion
P_current = Q;
P_new = P_current;
L_current = zeros(size(Q,1), size(R,2)); %general form, in ex. creates 4x1 zeros matrix
L_new = L_current;
firstIt = true; 
while (norm(L_new - L_current, 2) >= 1e-4) || (firstIt == true)
    if firstIt == true
        firstIt = false;
    end
    L_current = L_new;
    P_current = P_new;
    L_new = -inv(R + B'*P_current*B)*(B'*P_current*A);
    P_new = Q + L_new'*R*L_new + (A + B*L_new)'*P_current*(A + B*L_new);
   
end

P = P_new;
L = L_new;

end