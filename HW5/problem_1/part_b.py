from model import dynamics, cost
import numpy as np


stochastic_dynamics = False # set to True for stochastic dynamics
dynfun = dynamics(stochastic=stochastic_dynamics)
costfun = cost()

T = 100 # episode length
N = 100 # number of episodes
gamma = 0.95 # discount factor
P_hat = np.eye(6)
A_tilde = np.zeros((6,4))
total_costs = []
#tmp adjustments to variables, remove after checking code
# u = [0,0]
# N = 1
# T = 1
for n in range(N):
    costs = []
    
    x = dynfun.reset()
    for t in range(T):
        

        # TODO compute policy
        
        # TODO compute action
        #print(type(x))
        # get reward
        c = costfun.evaluate(x,u)
        costs.append((gamma**t)*c)
        
        # dynamics step
        xp = dynfun.step(u)
        
        # TODO implement recursive least squares update
        #using block matrices, and HW hint, form x and u of systme into single vector to use iterative least squares
        #A_tilde_p = np.zeros((8,6))
        xn = np.append(x,u)
        innov = np.transpose(xp) - np.dot(np.transpose(xn),A_tilde)
        A_tilde = A_tilde + (np.outer(np.dot(P_hat,xn),innov)/(1 + np.dot(np.transpose(xn),np.dot(P_hat,xn))))
        #P_hat(t-1) is used to calc A(t), so update P(t) after calc A(t)
        P_hat = P_hat - (np.dot(P_hat,np.outer(xn,np.dot(np.transpose(xn),P_hat)))/(1 + np.dot(np.transpose(xn),np.dot(P_hat,xn))))
        x = xp.copy()
        
    total_costs.append(sum(costs))
