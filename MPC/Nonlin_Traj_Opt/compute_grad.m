clear variables; close all; clc

syms x u y v psi r Nw f Iz a b By Cy Dy Ey Shy Svy m g F_x delta_f

%slip angle functions in degrees
a_f= 180/pi*(delta_f-atan((v+a*r)/u));
a_r= 180/pi*(-atan((v-b*r)/u));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

x_dot = u*cos(psi)-v*sin(psi);
u_dot = (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+v*r;
y_dot = u*sin(psi)+v*cos(psi);
v_dot = (F_yf*cos(delta_f)+F_yr)/m-u*r;
psi_dot = r;
r_dot = (F_yf*a*cos(delta_f)-F_yr*b)/Iz;

states = [x, u, y, v, psi, r];
grad_x = gradient(x_dot,states); % remove semicolon to unsupress or type grad_x to see explicit expressions
grad_u = gradient(u_dot,states);
grad_y = gradient(y_dot,states);
grad_v = gradient(v_dot,states);
grad_psi = gradient(psi_dot,states);
grad_r = gradient(r_dot,states);

inputs = [delta_f, F_x];
grad_x_delta = gradient(x_dot,inputs);
grad_u_delta = gradient(u_dot,inputs);
grad_y_delta = gradient(y_dot,inputs);
grad_v_delta = gradient(v_dot,inputs);
grad_psi_delta = gradient(psi_dot,inputs);
grad_r_delta = gradient(r_dot,inputs);
% State equations for reference from forwardIntegrateControlInput.m

% dzdt= [u*cos(psi)-x(4)*sin(x(5));...
%           (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
%           x(2)*sin(x(5))+x(4)*cos(x(5));...
%           (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
%           x(6);...
%           (F_yf*a*cos(delta_f)-F_yr*b)/Iz];



