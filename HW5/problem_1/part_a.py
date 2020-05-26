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
    P = Q + np.dot(np.transpose(A),np.dot(P,A)) - np.dot(np.transpose(A), np.dot(P, np.dot(B, np.dot(np.linalg.inv(R + np.dot(np.transpose(B), np.dot(P,B))), np.dot(np.tranpsose(B),np.dot(P,A))))))
    #define K
    K = np.dot(np.linalg.inv(R + np.dot(np.transpose(B), np.dot(P, B))), np.dot(np.transpose(B), np.dot(P,A)))
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