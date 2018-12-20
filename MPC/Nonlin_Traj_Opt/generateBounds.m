function [lb,ub] = generateBounds(TestTrack)

x_left = TestTrack.bl(1,:);
x_right = TestTrack.br(1,:);
y_left = TestTrack.bl(2,:);
y_right = TestTrack.br(2,:);

x_lb = zeros(size(x_left));
x_ub = zeros(size(x_left));
y_lb = zeros(size(y_left));
y_ub = zeros(size(y_left));

psi_lb = TestTrack.theta - pi/2;
psi_ub = TestTrack.theta + pi/2;
delta_lb = -0.5;
delta_ub = 0.5;
Fx_lb = -5000;
Fx_ub = 2500;

for i = 1:length(x_left) 
    if(x_left(i) < x_right(i))
        x_lb(i) = x_left(i);
        x_ub(i) = x_right(i);
    elseif(x_right(i) < x_left(i))
        x_lb(i) = x_right(i);
        x_ub(i) = x_left(i);
    end
    
    if(y_left(i) < y_right(i))
        y_lb(i) = y_left(i);
        y_ub(i) = y_right(i);
    elseif(y_right(i) < y_left(i))
        y_lb(i) = y_right(i);
        y_ub(i) = y_left(i);
    end
end % populates x_lb/ub and y_lb/ub

nsteps = length(x_left) * 5 - 2;
lb = zeros(1,nsteps);
ub = zeros(1,nsteps);

for i = 1:length(x_left)
        lb(3*i - 2:3*i) = [x_lb(i) y_lb(i) psi_lb(i)];
        ub(3*i - 2:3*i) = [x_ub(i) y_ub(i) psi_ub(i)];
    if(i < length(x_left))
        lb(2*i - 1 + 3*length(x_left):2*i + 3*length(x_left)) = [delta_lb Fx_lb];
        ub(2*i - 1 + 3*length(x_left):2*i + 3*length(x_left)) = [delta_ub Fx_ub];
    end   
end

end