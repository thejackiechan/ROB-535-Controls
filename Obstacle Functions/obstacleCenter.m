function obs_center = obstacleCenter(Xobs)  % returns n x 2 matrix of x, y
    for i = 1:length(Xobs)
        obs_center_x(i,1) = (Xobs{1,i}(1,1) + Xobs{1,i}(3,1))/2;
        obs_center_y(i,1) = (Xobs{1,i}(1,2) + Xobs{1,i}(3,2))/2;
        obs_center = [obs_center_x obs_center_y];
    end
end