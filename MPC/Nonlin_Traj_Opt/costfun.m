function [J, dJ] = costfun(z)
    if size(z,2) > size(z,1) 
        z = z' ; % z = column vec
    end
    nsteps = (size(z,1)+2)/8 ; 

    zx = z(1:6*nsteps) ;
    zu = z(6*nsteps+1:end) ;
    R=eye(2*nsteps-2); % play with Q and R matrices later

    nom=zeros(6*nsteps,1) ;
    nom(1:6:6*nsteps) = 3.085408587860615e+02 ; % index 26 of cline
    nom(3:6:6*nsteps) = 97.323427166445100 ;
    nom(5:6:6*nsteps) = 0.553665578914144 ; % theta?
    Q=eye(6*nsteps);

    J = sqrt(zu'*R*zu+(zx-nom)'*Q*(zx-nom)) ;
    dJ = [Q*(zx-nom)./sqrt((zx-nom)'*Q*(zx-nom));...
          R*zu./sqrt(zu)'*R*zu]' ;
end