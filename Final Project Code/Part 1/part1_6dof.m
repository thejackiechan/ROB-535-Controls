function U = part1_6dof
close all
load TestTrack.mat

global dt m Nw f Iz a b By Cy Dy Ey Shy Svy G
dt      = 0.01;
m       = 1400;
Nw      = 2;
f       = 0.01;
Iz      = 2667;
a       = 1.35;
b       = 1.45;
By      = 0.27;
Cy      = 1.2;
Dy      = 0.7;
Ey      = -1.6;
Shy     = 0;
Svy     = 0;
G       = 9.806;

%% FINE GRAIN TRACK
div = 100;
TestTrack2.bl = interp1(1:246,TestTrack.bl',1:1/div:246)';
TestTrack2.br = interp1(1:246,TestTrack.br',1:1/div:246)';
TestTrack2.cline = interp1(1:246,TestTrack.cline',1:1/div:246)';
TestTrack2.theta = interp1(1:246,TestTrack.theta',1:1/div:246);
TestTrack2.leftlane = (TestTrack2.bl + TestTrack2.cline)./2;
TestTrack2.rightlane = (TestTrack2.br + TestTrack2.cline)./2;

% %% PLOT TRACK
% figure(1)
%     hold all
%     plot(TestTrack2.bl(1,:),TestTrack2.bl(2,:),'k')
%     plot(TestTrack2.br(1,:),TestTrack2.br(2,:),'k')    
%     plot(TestTrack2.cline(1,:),TestTrack2.cline(2,:),'c--')

%% CONSTANTS
x0      = [287 5 -176 0 2 0];   % IC
x       = x0;
vel     = 10;                   % Desired longitudinal velocity
nsteps  = 20000;

lim.del = [-0.5 0.5];
lim.Fx  = [-5000 2500];

% PID gains
Kp.del = 0.5;
Ki.del = 0;
Kd.del = 0.5;

Kp.Fx  = 700;
Ki.Fx  = 100;
Kd.Fx  = 700;

cteD(1) = 0;
cteI(1) = 0;
ueD(1) = 0;
ueI(1) = 0;
    
for i = 1:nsteps-1
    % Cross track error (CTE)
    cte(i,1) = getCTE(TestTrack2,[x(i,1);x(i,3)]);

    % Speed error
    ue(i,1)  = vel - x(i,2);

    % Error Derivatives & Integrals
    if i >= 2
        cteD(i,1) = diff(cte(i-1:i))/dt;    % Derivative
        cteI(i,1) = cteI(i-1) + cte(i)*dt;  % Integral

        ueD(i,1) = diff(ue(i-1:i))/dt;      % Derivative
        ueI(i,1) = ueI(i-1) + ue(i)*dt; 	% Integral
    end

    % Update control command
    U(i,1) = Kp.del*cte(i) + Ki.del*cteI(i) + Kd.del*cteD(i);   % PID
    U(i,2) = Kp.Fx*ue(i) + Ki.Fx*ueI(i) + Kd.Fx*ueD(i);         % PID

    % Controller saturation
    if U(i,1) > lim.del(2)
        U(i,1) = lim.del(2);
    elseif U(i,1) < lim.del(1)
        U(i,1) = lim.del(1);
    end
    if U(i,2) > lim.Fx(2)
        U(i,2) = lim.Fx(2);
    elseif U(i,2) < lim.Fx(1)
        U(i,2) = lim.Fx(1);
    end

    % Simulate next step
    x(i+1,:) = x(i,:) + dt*odefun(x(i,:),U(i,:))';

    % Exit after crossing finish line
    if x(i+1,3) > 821
        nsteps = i+1;
        break
    end
end

% Add extra control to ensure crossing finish line
extra   = 100;
nsteps  = nsteps + extra;
U(end+1:end+extra,1) = 0;
U(end-extra+1:end,2) = lim.Fx(2);

% U = U';   
% uFun = @(t) [interp1(0:dt:(nsteps-2)*dt,U(1,:),t,'previous','extrap');...
%              interp1(0:dt:(nsteps-2)*dt,U(2,:),t,'previous','extrap')];
% 
% options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
% [~,Y1] = ode45(@(t,x) odefun(x,uFun(t)),[0:dt:(nsteps-1)*dt],x0,options);
   
% figure(1)
%     plot(x(:,1),x(:,3),'r');
%     plot(Y1(:,1),Y1(:,3),'m');
%                 
% figure(2)
%     plot(cte)
%         
% figure(3)
%     hold all
%     plot(x(:,2))
%     plot(x(:,4))

end

function [cte,ind] = getCTE(track,point)
    [cte,ind] = min(vecnorm(track.cline-point));

    if track.theta(ind) >= deg2rad(5) & point(1) < track.cline(1,ind)
        cte = -1*cte; % CTE negative if to the left of track
%     elseif track.theta <= degtorad(-10) & point(1) > track.cline(1,ind)
%         cte = -1*cte; % CTE negative if to the left of track
    elseif track.theta(ind) < deg2rad(5) & point(2) > track.cline(2,ind)
        cte = -1*cte; % CTE negative if to the left of track
    end

end

function [dz] = odefun(X,U)
    global m Nw f Iz a b By Cy Dy Ey Shy Svy G
    
    x   = X(1);
    u   = X(2);
    y   = X(3);
    v   = X(4);
    psi = X(5);
    r   = X(6);
    delta_f = U(1);
    F_x     = U(2);
    
    % Slip angle function in degrees
    a_f = rad2deg(delta_f-atan2(v+a*r,u));
    a_r = rad2deg(-atan2(v - b*r,u));

    % Nonlinear tire dynamics
    phi_yf  = (1-Ey)*(a_f + Shy) + Ey/By*atan(By*(a_f + Shy));
    phi_yr  = (1-Ey)*(a_r + Shy) + Ey/By*atan(By*(a_r + Shy));
    
    % Front and rear lateral forces
    F_zf    = b*m*G/(a+b);
    F_yf    = F_zf*Dy*sin(Cy*atan(By*phi_yf)) + Svy;
    F_zr    = a*m*G/(a+b);
    F_yr    = F_zr*Dy*sin(Cy*atan(By*phi_yr)) + Svy;
    
    % Tire loading
    F_total = sqrt((Nw*F_x)^2 + F_yr^2);
    F_max   = 0.7*m*G;
    
    if F_total > F_max
        F_x = F_max/F_total*F_x;
        F_yr= F_max/F_total*F_yr;
    end
    
    xDot = u*cos(psi) - v*sin(psi);
    uDot = (-f*m*G + Nw*F_x - F_yf*sin(delta_f))/m + v*r;
    yDot = u*sin(psi) + v*cos(psi);
    vDot = (F_yf*cos(delta_f) + F_yr)/m - u*r;
    psiDot = r;
    rDot = (F_yf*a*cos(delta_f) - F_yr*b)/Iz;
    
    dz  = [xDot;uDot;yDot;vDot;psiDot;rDot];
end