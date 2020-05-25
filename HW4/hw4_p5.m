%hw4_p5

clear all
close all
clc

% System constants
a = -1;
b = 3;
am = 4;
bm = 4;
gamma = 2; %learning rate

%initial conditions
y0 = 0;
ym0 = 0;
kr0 = 0;
ky0 = 0;
sys0 = [y0, ym0, kr0, ky0]';

tspan = [0 10];
rt1 = 4*tspan;
rt2 = 4*sin(tspan*3);
for j = 1:2
    [t, sys] = ode45(@(t,sys) f(t,sys,a,b,am,bm,gamma,j), tspan, sys0);
    %plot ys
    figure;
    plot(t,sys(:,1))
    hold on
    plot(t,sys(:,2))
    xlabel('time(s)')
    ylabel('y val')
    legend('y','ym')
    %plot ks
    figure;
    plot(t,sys(:,3))
    hold on
    plot(t,sys(:,4))
    xlabel('time(s)')
    ylabel('k val')
    legend('kr','ky')
end

%sys = [y,ym,kr,ky]'
function sysdot = f(t,sys,a,b,am,bm,gamma, j)
    if j == 1
        r = 4;
    elseif j == 2
        r = 4*sin(3*t);
    else
        error('Incorrect r index')
%         break;
    end
    y = sys(1);
    ym = sys(2);
    kr = sys(3);
    ky = sys(4);
    u = kr*r + ky*y;
    ydot = b*u - a*y;
    ymdot = bm*r - am*ym;
    e = y - ym;
    krdot = -sign(b)*gamma*e*r;
    kydot = -sign(b)*gamma*e*y;
    sysdot = zeros(length(sys),1);
    sysdot(1) = ydot;
    sysdot(2) = ymdot;
    sysdot(3) = krdot;
    sysdot(4) = kydot;
    
end

