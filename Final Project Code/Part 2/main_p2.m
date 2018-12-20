clear all; close all; clc;
tic

load TestTrack.mat
Xobs = generateRandomObstacles(25);

sol_2       = ROB599_ControlsProject_part2_Team9(TestTrack, Xobs);
[Y,T] = forwardIntegrateControlInput2(sol_2);
%% Results
info = getTrajectoryInfo(Y,sol_2, Xobs, TestTrack)

figure
    hold all
    plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
    plot(TestTrack.br(1,:),TestTrack.br(2,:),'k')    
    plot(info.Y(:,1),info.Y(:,3),'r')
    for i = 1:length(Xobs)
        plot(Xobs{1,i}(:,1),Xobs{1,i}(:,2),'m')
    end
    
toc
    
    

