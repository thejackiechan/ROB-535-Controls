function [dxdt] = odefun(x,u)
    %constants
    Nw=2;
    f=0.01;
    Iz=2667;
    a=1.35;
    b=1.45;
    By=0.27;
    Cy=1.2;
    Dy=0.7;
    Ey=-1.6;
    Shy=0;
    Svy=0;
    m=1400;
    g=9.806;
    
    a_f=rad2deg(u(1)-atan2(x(4)+a*x(6),x(2)));
    a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));
    
    phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
    phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

    F_zf=b/(a+b)*m*g;
    F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

    F_zr=a/(a+b)*m*g;
    F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;
    
    F_x = u(2);
    F_total=sqrt((Nw*F_x)^2+(F_yr^2));
    F_max=0.7*m*g;

    if F_total>F_max
    
        u(2)=F_max/F_total*F_x;
  
        F_yr=F_max/F_total*F_yr;
    end

    dxdt = [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*u(2)-F_yf*sin(u(1)))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(u(1))+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(u(1))-F_yr*b)/Iz];

end