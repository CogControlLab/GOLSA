curPath = fileparts(mfilename('fullpath'));
addpath(genpath(curPath))

if 1
    %Run single trial of core network with fixed (correct) weights
    %in 6-state environment starting in state 1 with goal in state 6
    smallEnvNet_postLearning;
end


if 0
    %Run 20 trials of core network with learning on
    %in 6-state environment starting in state 1 with goal in state 6
    smallEnvNet_learning;
end


if 0
   %Run one trial of fully trained network with goal-learning module
   %in 6-state environment with reward 1 in state 1
   %and reward 2 in state 5; both drives start high
   smallEnvNet_driveModule_postLearning;
    
end
    

if 0
    %Run one trial of network with goal-learning module with
    %the mapping from drives to goals learned over the course
    %trial, using the same environment as above.
    
   smallEnvNet_driveModule_learning;
    
end


if 0
   %Run one trial showing the storage and execution of a plan
   %in the context of a simple 6-state environment starting 
   %in state 1 with a goal of state 6.
   smallEnvNet_queueModule;
    
end


if 0
    %Run one trial in a fully trained core network
    %in the context of the Tower of Hanoi.
    hanoiNet_postLearning;
end


if 0
    %Run 20 trials in an untrained network in the Tower 
    %of Hanoi task, demonstrating learning. Starting states
    %and goal states rotate across trials.
    hanoiNet_learning;
end