clear all; close all; clc;
tic

load TestTrack.mat

ROB599_ControlsProject_part1_input       = part1_6dof;
[Y,T] = forwardIntegrateControlInput2(ROB599_ControlsProject_part1_input);

%% Results
info = getTrajectoryInfo(Y,ROB599_ControlsProject_part1_input)

figure
    hold all
    plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
    plot(TestTrack.br(1,:),TestTrack.br(2,:),'k')    
    plot(info.Y(:,1),info.Y(:,3),'r')
    
toc
    
    

