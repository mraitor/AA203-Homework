%coppied example from MPT website
% computes a control invariant set for LTI system x^+ = A*x+B*u
system = LTISystem('A', [1 1 0; 0 0.9 1; 0 0.2 0], 'B', [0; 1; 0]);
system.x.min = [-5; -5; -5];
system.x.max = [5; 5; 5];
system.u.min = -0.5;
system.u.max = 0.5;
InvSet = system.invariantSet()
InvSet.plot()