function [U, x] = part2_6dof_edit
close all
%%
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

% FINE GRAIN TRACK
div = 100;
TestTrack2.bl = interp1(1:246,TestTrack.bl',1:1/div:246)';
TestTrack2.br = interp1(1:246,TestTrack.br',1:1/div:246)';
TestTrack2.cline = interp1(1:246,TestTrack.cline',1:1/div:246)';
TestTrack2.theta = interp1(1:246,TestTrack.theta',1:1/div:246);
TestTrack2.leftlane = TestTrack2.cline - (TestTrack2.cline - TestTrack2.bl)./4;
TestTrack2.rightlane = TestTrack2.cline + (TestTrack2.br - TestTrack2.cline)./4;

% %% PLOT TRACK
% figure(1)
%     hold all
%     plot(TestTrack2.bl(1,:),TestTrack2.bl(2,:),'k')
%     plot(TestTrack2.br(1,:),TestTrack2.br(2,:),'k')    
%     plot(TestTrack2.cline(1,:),TestTrack2.cline(2,:),'c--')

% CONSTANTS
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

Kp.hdg = 0.2;

cteD(1) = 0;
cteI(1) = 0;
ueD(1) = 0;
ueI(1) = 0;


% Import Obstacles and display them

num_obs = 25; 
obs_max_length = 4;
buffer = 0.5;
next_obs = 1;
Xobs = generateRandomObstacles(num_obs);
figure;
hold all

for i = 1:num_obs
    plot(Xobs{1,i}(:,1),Xobs{1,i}(:,2),'m');
end

plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k') 
plot(TestTrack.cline(1,:),TestTrack.cline(2,:));

[Xobs_c] = obstacleCenter(Xobs); % gives positions of obstacle center
Xobs_lane = zeros(num_obs,1);
Xobs_index = zeros(num_obs,1);

for k = 1:num_obs
    ob_c = Xobs_c(k,:);
    [ldist,lindex] = closestPoint(ob_c',TestTrack.bl);
    [rdist,rindex] = closestPoint(ob_c',TestTrack.br);
    
    if ldist > rdist 
        Xobs_lane(k) = 3;
        Xobs_index(k) = rindex;
    else
        Xobs_lane(k) = 1;
        Xobs_index(k) = lindex;
    end
end
%%

for i = 1:nsteps-1
    % Cross track error (CTE)
    cte(i,1) = getCTE(TestTrack2.cline,TestTrack2.theta,[x(i,1);x(i,3)]);
    cte_left(i,1) = getCTE(TestTrack2.leftlane,TestTrack2.theta,[x(i,1);x(i,3)]);
    cte_right(i,1) = getCTE(TestTrack2.rightlane,TestTrack2.theta,[x(i,1);x(i,3)]);
    

    % Changing Lanes (Edited this part mainly, also defined some helper
    % variables above such as buffer, max_obs_length, etc.)
    
    % Initially it would be a good idea to start in the lane opposite where the
    % first obstacle is located.
    % For every obstacle afterwards, we make a decision (to lane change or not) pretty much immediately
    % after fully passing the current obstacle; don't lane change if you
    % don't need to
    % Idea is to use X_obs index and the max length (or width not sure?) of
    % the obstacle and correlate that to an index i in which it is safe to
    % lane change; this way you don't need to do any x or y logic which can
    % be irritating for various parts of the track. As you increment index,
    % you are moving along the track in the same direction as the
    % obstacle length as well as the trajectory, which is why I believe it works best. I've defined
    % obs_max_length above and using the center, the end of the obstacle
    % should be located at X_obs_c + obs_max_length/2 (split into x and y
    % of course but let's assume a factor of safety called buffer). We need to correlate
    % distance along the track to indexes. 
    
    % need this in front for initial lane change (next_obs = 1)
    
    if next_obs == 1 % fencepost
        if Xobs_lane(next_obs) == 1 % obs in left lane
            cte = cte_right;
        elseif Xobs_lane(next_obs) == 3 % obs in right lane
            cte = cte_left; % do we need to go back to center ever? I'm not sure how this code looks
        end
    end
    
    % Below code is only useful once next_obs >= 2 b/c you will be changing lanes
    % right after you pass the current obstacle
    % The 2nd term is calculated using t = d/v, dividing t by dt
    % gets us the number of indices from the center of the obstacle to its
    % end; x(Xobs_index(next_obs),2) gives us a good approximation of how
    % fast the vehicle is moving through that particular obstacle
    
    % You can definitely tweak some of these (make obs_end_idx smaller or larger) 
    % depending on how close obstacles are as well as how long it takes for 
    % PID to perform a full lane change; the buffer distance can be tweaked
    % first
    obs_end_idx = Xobs_index(next_obs) + (obs_max_length/2 + buffer)/x(Xobs_index(next_obs),2)/dt;  % you

    
    if i > obs_end_idx && next_obs < num_obs % means you passed the obstacle completely
        next_obs = next_obs + 1; % move on to next obstacle
    end
    
    % Switches the reference trajectory, this part initiates the lane
    % change
    if Xobs_lane(next_obs) == 1 % obs in left lane
        cte = cte_right;
    elseif Xobs_lane(next_obs) == 3 % obs in right lane
        cte = cte_left; % do we need to go back to center ever?
    end
    
    % Most of my edits end here and you may need to test this to see if it
    % works for multiple obstacles. In theory, it makes sense in my head
    % but I only got 3-4 hours of sleep so... Good luck m8
    
    
    % Speed error
    ue(i,1)  = vel - x(i,2);
    
    % Heading error
    hdg(i,1) = TestTrack2.theta(i) - x(i,5);

    % Error Derivatives & Integrals
    
    if i >= 2
        cteD(i,1) = diff(cte(i-1:i))/dt;    % Derivative
        cteI(i,1) = cteI(i-1) + cte(i)*dt;  % Integral

        ueD(i,1) = diff(ue(i-1:i))/dt;      % Derivative
        ueI(i,1) = ueI(i-1) + ue(i)*dt; 	% Integral
    end

    % Update control command
    U(i,1) = Kp.del*cte(i) + Ki.del*cteI(i) + Kd.del*cteD(i) + Kp.hdg*hdg(i);   % PID
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
%     x(i+1,:) = x(i,:) + dt*odefun(x(i,:),U(i,:))';
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    [~,X]=ode45(@(t,z)odefun(z,U(i,:)),[0 dt],x(i,:),options);
    
    x(i+1,:) = X(end,:);
    
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

function [cte,ind] = getCTE(track,angle,point)
    [cte,ind] = min(vecnorm(track-point));

    if angle(ind) >= deg2rad(5) && point(1) < track(1,ind)
        cte = -1*cte; % CTE negative if to the left of track
%     elseif track.theta <= degtorad(-10) & point(1) > track.cline(1,ind)
%         cte = -1*cte; % CTE negative if to the left of track
    elseif angle(ind) < deg2rad(5) && point(2) > track(2,ind)
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

function [obs_center] = obstacleCenter(Xobs)  % returns n x 2 matrix of x, y
    for i = 1:length(Xobs)
        obs_center_x(i,1) = (Xobs{1,i}(1,1) + Xobs{1,i}(3,1))/2;
        obs_center_y(i,1) = (Xobs{1,i}(1,2) + Xobs{1,i}(3,2))/2;
    end
    obs_center = [obs_center_x obs_center_y];
end

function [dist,i] = closestPoint(x,line)
    line = interp1(1:246,line',0:0.01:246)';
    D = sqrt(sum((x-line).^2));
    [dist,i] = min(D);
end
