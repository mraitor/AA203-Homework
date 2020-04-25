% Hw 2 p4 - quadcopter naviagation in storm Value iteration

clear all
close all
clc

%given variables/parameters
n = 20;
gamma = 0.95;
sigma = 10;
eyeStorm = [16, 16]'; %should be 15 and 15 but matlab is base 1
goal = [20, 10]'; %should be 19,9 but matlab base 1

stormActive = 1; %storm is active

%contruct equations components
V_current = zeros(n,n);
V_new = V_current;
A = [-1 1 0 0;  %A is selection of control actions, a, selected along column of A
    0 0 -1 1];%col. order rep: up, down, left, right
itCount = 0;

%value iteration
% while (sum(sum((V_current - V_new > 1e-4))) > 1) || itCount == 0 %check convergence by ensuring difference between old and new V has very low scalling
for b = 1:200
    %      sum(sum(((V_current - V_new) > 1e-5)))
    V_current = V_new;
    for i = 1:n %move along a row first
        for j = 1:n %move along a column
            x = [i; j];
            V_new(i,j) = 0;
            for a = 1:4 %cycle through control options
                V_tmp = 0;
                %up
                if i > 1
                    xp = x + [-1; 0];
                else
                    xp = x;
                end
                V_tmp = V_tmp + P(xp,x,A(:,a), eyeStorm, sigma)*(R(xp, goal) + gamma*V_current(xp(1),xp(2)));
                %down
                if i < n
                    xp = x + [1; 0];
                else
                    xp = x;
                end
                V_tmp = V_tmp + P(xp,x,A(:,a), eyeStorm, sigma)*(R(xp, goal)+gamma*V_current(xp(1),xp(2)));
                %left
                if j > 1
                    xp = x + [0; -1];
                else
                    xp = x;
                end
                V_tmp = V_tmp + P(xp,x,A(:,a), eyeStorm, sigma)*(R(xp, goal)+gamma*V_current(xp(1),xp(2)));
                %down
                if j < n
                    xp = x + [0; 1];
                else
                    xp = x;
                end
                V_tmp = V_tmp + P(xp,x,A(:,a), eyeStorm, sigma)*(R(xp, goal)+gamma*V_current(xp(1),xp(2)));
                
                % logic to set set max V(i,j) for available a, to
                % V_new(i,j)
                if V_tmp > V_new(i,j)
                    V_new(i,j) = V_tmp;
                end
                
            end
        end
    end
    itCount =+ 1;
end
%plot heatmap of value function
figure;
heatmap(V_new)

%calc policy
%policy strategy is to begin at start location and move up, down. left, or
%right based on which possible move has highest value function
%ince the vlaue function bakes in the future possibilities from that
%state, no need ofr fnacy planning, just a greedy, "which option is
%highest?" controller

start = [10; 20]; %should be 9. 19 but matlab is base 1
pi_star = zeros(2,1);
x_current = start;
x_hist(:,1) = start;
k = 1;
while sum(abs(x_current - goal)) > 1e-4
    V_tmp = V_new(x_current(1),x_current(2));
    for a = 1:4
        %disp(a)
        pik =  A(:,a);
        xp = x_current + A(:,a);
        %up clip
        if xp(1) == 0
            xp = xp + [1; 0];
            pik = [0; 0];
        end
        %down clip
        if xp(1) == n+1
            xp = xp + [-1; 0];
            pik = [0; 0];
        end
        %left clip
        if xp(2) == 0
            xp = xp + [0; 1];
            pik = [0; 0];
        end
        %right clip
        if xp(2) == n+1
            xp = xp + [0; -1];
            pik = [0; 0];
        end
        
        % logic to set set max V(i,j) for available a, to
        % V_new(i,j)
        if V_new(xp(1),xp(2)) > V_tmp
            pi_star(:,k) = pik; %update bet
            V_tmp = V_new(xp(1),xp(2));
        end
    end
    if stormActive && (rand<=exp(-norm(x_current-eyeStorm,2)/(2*sigma^2))); %if perturbed by storm
        randDir = rand;
        if randDir <= 0.25
            %up
            if x_current(1) > 1
                x_current = x_current + [-1;0]; %perturb if pert direction allowed given state
            end
        elseif (randDir > 0.25) && (randDir <= 0.50)
            %down
            if x_current(1) < n
                x_current = x_current + [1;0]; %perturb if pert direction allowed given state
            end
        elseif (randDir > 0.50) && (randDir <= 0.75)
             %left
            if x_current(2) > 1
                x_current = x_current + [0;-1]; %perturb if pert direction allowed given state
            end
        else 
            %right
            if x_current(2) < n
                x_current = x_current + [0;1]; %perturb if pert direction allowed given state
            end
        end
    else
    x_current = x_current +  pi_star(:,k);
    end
    k = k + 1;
    x_hist(:,k) = x_current;
end

%plot trajectory
figure;
plot(x_hist(1,:), -x_hist(2,:))
axis([0 21 -21 0])
%gnerate trajector from pi_star
% X_plot = start;
% for o = 1:(k-1)
%     X_plot(:,o+1) = X_plot(:,o) + pi_star(:,o);
% end
% figure;
% plot(X_plot(1,:), -X_plot(2,:))
% axis([0 21 -21 0])
%helper functions
%reward function, 1 if at goal, else 0
function y = R(x,goal)
if x == goal
    y = 1;
else
    y = 0;
end
end

%probability of moving to xp given x and control action a
function y = P(xp, x, a, eyeStorm, sigma)
px = exp(-norm(x-eyeStorm,2)/(2*sigma^2));
if xp == x + a
    y = 0.25*(1-px);
else
    y = 0.25*px;
end
end

% function y = P(xp, x, a, eyeStorm, sigma)
%     if xp == x + a
%         y = 1;
%     else
%         y = exp(-norm(x-eyeStorm,2)/(2*sigma^2));
%     end
% end