clear variables; close all; clc
% define constants
b = 1.45 ; 
L = 2.8 ;
nsteps = length(0:0.01:10) ; % time change
dt = 0.01 ;
t = 0:dt:10; % time change

% import bounds from generateBounds
file = load('TestTrack.mat');
TestTrack = file.TestTrack;
[full_lb, full_ub] = generateBounds(TestTrack);

% extract section of track
first_section(1,:) = full_lb(1:21); % lb
first_section(2,:) = full_ub(1:21); % ub

separate(1,:) = first_section(1,1:3:end); 
separate(2,:) = first_section(1,2:3:end);
separate(3,:) = first_section(1,3:3:end); % lb
separate(4,:) = first_section(2,1:3:end); % ub
separate(5,:) = first_section(2,2:3:end);
separate(6,:) = first_section(2,3:3:end);

% interpolate track
assumed_T = linspace(0,10,length(separate)); % change 10 depending on time
discretized_bounds(1,:)=interp1(assumed_T,separate(1,:),t); 
discretized_bounds(2,:)=interp1(assumed_T,separate(2,:),t);
discretized_bounds(3,:)=interp1(assumed_T,separate(3,:),t); % lb
discretized_bounds(4,:)=interp1(assumed_T,separate(4,:),t); % ub
discretized_bounds(5,:)=interp1(assumed_T,separate(5,:),t);
discretized_bounds(6,:)=interp1(assumed_T,separate(6,:),t);
                  
% set upper bounds
ub = zeros(8*nsteps-2,1);
ub(1:6:6*nsteps) = discretized_bounds(4,:) ;
ub(2:6:6*nsteps) = Inf ;
ub(3:6:6*nsteps) = discretized_bounds(5,:) ;
ub(4:6:6*nsteps) = Inf ;
ub(5:6:6*nsteps) = discretized_bounds(6,:) ;
ub(6:6:6*nsteps) = Inf ;
ub(6*nsteps+1:2:8*nsteps-2) = 0.5 ;
ub(6*nsteps+2:2:8*nsteps-2) = 2500 ;
 
% set lower bounds
lb = zeros(8*nsteps-2,1);
lb(1:6:6*nsteps) = discretized_bounds(1,:) ;
lb(2:6:6*nsteps) = 0.01 ; % edited from Inf
lb(3:6:6*nsteps) = discretized_bounds(2,:) ;
lb(4:6:6*nsteps) = -Inf ;
lb(5:6:6*nsteps) = discretized_bounds(3,:) ;
lb(6:6:6*nsteps) = -Inf ;
lb(6*nsteps+1:2:8*nsteps-2) = -0.5 ;
lb(6*nsteps+2:2:8*nsteps-2) = -5000 ;
% lb(1:6:6*nsteps) = -1 ;
% lb(2:3:3*nsteps) = -3 ;
% lb(3:3:3*nsteps) = -pi/2 ;
% lb(3*nsteps+1:2:5*nsteps-2) = 0 ;
% lb(3*nsteps+2:2:5*nsteps-2) = -0.5 ;

%optimization variables
% x0 = zeros(1,5*nsteps-2) ;
%x0 = zeros(1,8*nsteps-2) ;
x0 = getInitial();
%x0(1:6) = [287 5 -176 0 2 0];
cf = @costfun ;
nc = @nonlcon ;
options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
                        'SpecifyObjectiveGradient',true) ;
% run fmincon to find trajectory
z = fmincon(cf,x0,[],[],[],[],lb',ub',nc,options) ;
 
% reshape trajectory and inputs
% Y0 = reshape(z(1:6*nsteps),6,nsteps)' ;
% U = reshape(z(6*nsteps+1:end),2,nsteps-1) ;
% 
% % write input function as zero order hold
% u = @(t) [interp1(0:dt:5999*dt,U(1,:),t,'previous','extrap');...
%        interp1(0:dt:5999*dt,U(2,:),t,'previous','extrap')] ;
%    
% % run ode45 for each initial condition
% [T1,Y1] = ode45(@(t,x) odefun(x,u(t)),[0:dt:6000*dt],[0 0 0 0 0 0]) ;
% [T2,Y2] = ode45(@(t,x) odefun(x,u(t)),[0:dt:6000*dt],[287 5 -176 0 2 0]) ;
% 
% % plot trajectories obstacle and initial condition
% plot(Y0(:,1),Y0(:,2),Y1(:,1),Y1(:,2),Y2(:,1),Y2(:,2),...
%     (0.7*cos(0:0.01:2*pi)+3.5),(0.7*sin(0:0.01:2*pi)-0.5),0,0,'x') ;
% 
% legend('fmincon trajectory','ode45 trajectory using x0 = [0;0;0]',...
%     'ode45 trajectory using x0 = [0;0;-0.01]','buffered Obstacle','start') ;
% ylim([-2,2]); xlim([-1,8]); xlabel('x'); ylabel('y') ;
