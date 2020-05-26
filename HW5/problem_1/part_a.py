from model import dynamics, cost
import numpy as np

dynfun = dynamics(stochastic=False)
# dynfun = dynamics(stochastic=True) # uncomment for stochastic dynamics

costfun = cost()


T = 100 # episode length
N = 100 # number of episodes
gamma = 0.95 # discount factor

# Riccati recursion
def Riccati(A,B,Q,R):

    # TODO implement infinite horizon riccati recursion
    #matlab code that works
    # P_current = Q;
    # P_new = P_current;
    # L_current = zeros(size(Q,1), size(R,2)); %general form, in ex. creates 4x1 zeros matrix
    # L_new = L_current;
    # firstIt = true; 
    # while (norm(L_new - L_current, 2) >= 1e-4) || (firstIt == true)
    #     if firstIt == true
    #         firstIt = false;
    #     end
    #     L_current = L_new;
    #     P_current = P_new;
    #     L_new = -inv(R + B'*P_current*B)*(B'*P_current*A);
    #     P_new = Q + L_new'*R*L_new + (A + B*L_new)'*P_current*(A + B*L_new);
   
    #                 end

    #     P = P_new;
    #     L = L_new;
    #rewriting in stupid python
    P_current = Q
    P_new = P_current
    #fix L dims
    L_current = np.zeros([2,4])
    L_new = L_current
    firstIt = True
    while (np.linalg.norm(L_new - L_current, 2) >= 1e-3) or firstIt:
        if firstIt:
            firstIt = False
        L_current = L_new
        P_current = P_new
        L_new = np.linalg.inv(R + np.dot(np.transpose(B), np.dot(P_current, B)))@(np.dot(np.transpose(B),np.dot(P_current, A)))
        P_new = Q + np.dot(np.transpose(L_new), np.dot(R,L_new)) + np.transpose(A + np.dot(B,L_new)) @ P_current @ (A+np.dot(B,L_new))
        #print(L_new)
    L = L_new
    P = P_new   
    #DARE:
    # P = Q + np.dot(np.transpose(A),np.dot(P,A)) - np.dot(np.transpose(A), np.dot(P, np.dot(B, np.dot(np.linalg.inv(R + np.dot(np.transpose(B), np.dot(P,B))), np.dot(np.tranpsose(B),np.dot(P,A))))))
    #define K
    # K = np.dot(np.linalg.inv(R + np.dot(np.transpose(B), np.dot(P, B))), np.dot(np.transpose(B), np.dot(P,A)))
    return L,P


A = dynfun.A
B = dynfun.B
Q = costfun.Q
R = costfun.R

L,P = Riccati(A,B,Q,R)

total_costs = []

for n in range(N):
    costs = []
    
    x = dynfun.reset()
    for t in range(T):
        
        # policy 
        u = (-L @ x)
        
        # get reward
        c = costfun.evaluate(x,u)
        costs.append((gamma**t)*c)
    
        # dynamics step
        x = dynfun.step(u)
        
    total_costs.append(sum(costs))
    
print(np.mean(total_costs))