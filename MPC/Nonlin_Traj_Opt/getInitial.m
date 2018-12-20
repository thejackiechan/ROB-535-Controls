function [x0] = getInitial() % might want to pass in time and nsteps

% forward integrate to give a better guess for fmincon to converge

clear variables; close all; clc

dt = 0.01;
t = 0:dt:2;

const_Fx = 1500; % 0 steering b/c on straight

x = zeros(6,length(t));
u = zeros(2,length(t));

x(:,1) = [287 5 -176 0 2 0]';

%u(1,:) = 0.001;
u(2,:) = const_Fx;

for i = 1:length(t) - 1 
    
    x(:,i+1) = x(:,i) + dt*odefun(x(:,i),u(:,i));
    
end

nsteps = 1001;
x0 = zeros(1,nsteps*8 - 2);

for i = 1:length(x)
    
    x0(6*i - 5:6*i) = x(:,i)';
    x0(6*nsteps + 2*i - 1:6*nsteps + 2*i) = u(:,i)'; 
    
end

%[g,h,dg,dh]=nonlcon(x0);

end
