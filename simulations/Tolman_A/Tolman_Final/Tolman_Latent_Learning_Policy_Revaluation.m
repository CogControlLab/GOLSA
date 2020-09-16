


% runs TOLMAN-state environment simulation and policy revaluation
 warning('off','all')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%
% Basic Setup %
%%%%%%%%%%%%%%%
clear all
NEW_NETWORK = 1;
FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning

global dt endTime t ERROR
dt=.05;
ERROR = false;

if NEW_NETWORK
    [e, n] = create_Tolman_B_Net_core(FIXED_WEIGHTS,8);
elseif ~exist('n')
    error('Network has not been initialized. Set NEW_NETWORK=1')
    n.reset()
    e.reset()
end

load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
n.logAll();



%%%%%%%%%%%%%%%%
% Run Settings %
%%%%%%%%%%%%%%%%
MAX_TIME =300; %maximum trial length
MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
NUM_TRIALS = 100;
TEXT_OUTPUT = 1;
DISPLAY = 0;

SETUP_LIST = [ %column 1: starting states; column 2: goals
    1,14];
REPEAT_MODE = 2;  %1 = alternate, 2 = repeat, 3 = first only
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
% Set up batch %
%%%%%%%%%%%%%%%%

numSetups = size(SETUP_LIST, 1);
switch REPEAT_MODE
    case 1 %alternate rows of setupList one at time
        setupIndices = repmat(1:numSetups, 1, NUM_TRIALS/numSetups+1);
        
    case 2 %do each row of setupList a specified number of times before repeating
        numRepeats = 50;
        setupIteration = reshape(repmat(1:numSetups, numRepeats, 1), 1, numRepeats*numSetups);
        setupIndices = repmat(setupIteration, 1, ceil(NUM_TRIALS / (numRepeats * numSetups)));
        setupIndices = setupIndices(1:NUM_TRIALS);
        
    case 3 %only use first row
        setupIndices = ones(1,NUM_TRIALS);
end

%batchResults checks whether goal is reached, how long it takes, and how far from ideal
%the goalGradient->goalGradient, state->adjacent, and transition->motor weights are;
%6th column is the total error
batchResults = nan(NUM_TRIALS, 6);

n.logAll('layers');

%     the first list of items will be displayed in real time
%     the second list will be projected onto an environment of the dimenions
%     specified by the third argument
if DISPLAY
    n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
end


%%%%%%%%%%%%%%
% Run Trials %
%%%%%%%%%%%%%%


pot_goals=[14 21];
for trial_i = 1:NUM_TRIALS
    rng(45+trial_i);
    time_step=[];
    endTime=MAX_TIME;
    t = dt;
    tSinceMove = -1.5;
    moved = 0;
    exploreRate = .2;
    
    
    startingState = SETUP_LIST(setupIndices(trial_i), 1);
    goal = SETUP_LIST(setupIndices(trial_i), 2);
    
    lastState = startingState;
    
    n.reset();
    e.reset(startingState);
    n.so('n_env').set_vals(e.get_locVec.*4);
%     n.setNode('n_drives',[0.5, 1]); %Sets initial activity level of different drive
    
    
    fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
    dbx = nan(MAX_TIME/dt, 2);
    
    
    
    
    for t = dt:dt:endTime
        n.so('n_goal').vals([14 21])=[0.4 0.8]; n.so('n_goal').scaleVals(6);
        
        [moved, action, descrip] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
        if moved==1
            if action > 0
                n.setNode('n_body', action);
            end
            n.so('n_env').set_vals(e.get_locVec .* 4);
            n.setNode('n_novInhib', 1);
            tSinceMove = 0;
            
            if TEXT_OUTPUT
                newState = e.curState();
                time_step=[time_step;t];
                fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                lastState = newState;
            end
            
        else
            tSinceMove = tSinceMove + dt;
        end
        
        
        if (~isempty(find((e.curState == pot_goals))) && (tSinceMove > 2))
            disp('At goal')
            endTime = t;
                        final(trial_i,:)=[e.curState(),t];
                        timetime{trial_i,1}=time_step;
            break
        end
        
        %         if tSinceMove > 2
        %             newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
        %             n.so('n_drives').set_vals(newDrives);
        %         end
        %
        n.update();
        %         e.update();
        
        
    end
    
    if 0
        %%%%%%%%%%%%%%%%%%%%
        %Log Trial Results %
        %%%%%%%%%%%%%%%%%%%%
        
        batchResults(trial_i, 1) = 99; %usually keeps track of whether goal is attained
        
        batchResults(trial_i, 2) = endTime;
        
        gradWeights = n.so('p_gradient_gradient').weights;
        gradError = mean((gradWeights(:) - correctGradWeights(:)).^2);
        batchResults(trial_i, 3) = gradError;
        
        adjWeights = n.so('p_state_adjacent').weights;
        adjError = mean((adjWeights(:) - correctAdjWeights(:)).^2);
        batchResults(trial_i, 4) = adjError;
        
        motorWeights = n.so('p_transOut_motorIn').weights;
        motorError = mean((motorWeights(:) - correctMotorWeights(:)).^2);
        batchResults(trial_i, 5) = motorError;
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%
    %Plot Trial Results %
    %%%%%%%%%%%%%%%%%%%%%
    close all
    
    
    
   
end



%%%%%%%%%%%%% 2nd sim

% runs TOLMAN-state environment simulation and policy revaluation
 warning('off','all')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%
% Basic Setup %
%%%%%%%%%%%%%%%
clearvars -except final timetime
NEW_NETWORK = 1;
FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning

global dt endTime t ERROR
dt=.05;
ERROR = false;

if NEW_NETWORK
    [e, n] = create_Tolman_B_Net_core(FIXED_WEIGHTS,8);
elseif ~exist('n')
    error('Network has not been initialized. Set NEW_NETWORK=1')
    n.reset()
    e.reset()
end

load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
n.logAll();



%%%%%%%%%%%%%%%%
% Run Settings %
%%%%%%%%%%%%%%%%
MAX_TIME =300; %maximum trial length
MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
NUM_TRIALS = 100;
TEXT_OUTPUT = 1;
DISPLAY = 0;

SETUP_LIST = [ %column 1: starting states; column 2: goals
    1,14];
REPEAT_MODE = 2;  %1 = alternate, 2 = repeat, 3 = first only
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
% Set up batch %
%%%%%%%%%%%%%%%%

numSetups = size(SETUP_LIST, 1);
switch REPEAT_MODE
    case 1 %alternate rows of setupList one at time
        setupIndices = repmat(1:numSetups, 1, NUM_TRIALS/numSetups+1);
        
    case 2 %do each row of setupList a specified number of times before repeating
        numRepeats = 50;
        setupIteration = reshape(repmat(1:numSetups, numRepeats, 1), 1, numRepeats*numSetups);
        setupIndices = repmat(setupIteration, 1, ceil(NUM_TRIALS / (numRepeats * numSetups)));
        setupIndices = setupIndices(1:NUM_TRIALS);
        
    case 3 %only use first row
        setupIndices = ones(1,NUM_TRIALS);
end

%batchResults checks whether goal is reached, how long it takes, and how far from ideal
%the goalGradient->goalGradient, state->adjacent, and transition->motor weights are;
%6th column is the total error
batchResults = nan(NUM_TRIALS, 6);

n.logAll('layers');

%     the first list of items will be displayed in real time
%     the second list will be projected onto an environment of the dimenions
%     specified by the third argument
if DISPLAY
    n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
end


%%%%%%%%%%%%%%
% Run Trials %
%%%%%%%%%%%%%%


pot_goals=[14 21];
for trial_i = 1:NUM_TRIALS
    rng(45+trial_i);
time_step=[];
    endTime=MAX_TIME;
    t = dt;
    tSinceMove = -1.5;
    moved = 0;
    exploreRate = .2;
    
    
    startingState = SETUP_LIST(setupIndices(trial_i), 1);
    goal = SETUP_LIST(setupIndices(trial_i), 2);
    
    lastState = startingState;
    
    n.reset();
    e.reset(startingState);
    n.so('n_env').set_vals(e.get_locVec.*4);
%     n.setNode('n_drives',[0.5, 1]); %Sets initial activity level of different drive
    
    
    fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
    dbx = nan(MAX_TIME/dt, 2);
    
    
    
    
    for t = dt:dt:endTime
        n.so('n_goal').vals([14 21])=[0.38 0.8]; n.so('n_goal').scaleVals(4);
        
        [moved, action, descrip] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
        if moved==1
            if action > 0
                n.setNode('n_body', action);
            end
            n.so('n_env').set_vals(e.get_locVec .* 4);
            n.setNode('n_novInhib', 1);
            tSinceMove = 0;
            
            if TEXT_OUTPUT
                newState = e.curState();
                time_step=[time_step;t];

                fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                lastState = newState;
            end
            
        else
            tSinceMove = tSinceMove + dt;
        end
        
        
        if (~isempty(find((e.curState == pot_goals))) && (tSinceMove > 2))
            disp('At goal')
            endTime = t;
                        finalB(trial_i,:)=[e.curState(),t];
                            timetimeB{trial_i,1}=time_step;

            break
        end
        
        %         if tSinceMove > 2
        %             newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
        %             n.so('n_drives').set_vals(newDrives);
        %         end
        %
        n.update();
        %         e.update();
        
        
    end
    
    if 0
        %%%%%%%%%%%%%%%%%%%%
        %Log Trial Results %
        %%%%%%%%%%%%%%%%%%%%
        
        batchResults(trial_i, 1) = 99; %usually keeps track of whether goal is attained
        
        batchResults(trial_i, 2) = endTime;
        
        gradWeights = n.so('p_gradient_gradient').weights;
        gradError = mean((gradWeights(:) - correctGradWeights(:)).^2);
        batchResults(trial_i, 3) = gradError;
        
        adjWeights = n.so('p_state_adjacent').weights;
        adjError = mean((adjWeights(:) - correctAdjWeights(:)).^2);
        batchResults(trial_i, 4) = adjError;
        
        motorWeights = n.so('p_transOut_motorIn').weights;
        motorError = mean((motorWeights(:) - correctMotorWeights(:)).^2);
        batchResults(trial_i, 5) = motorError;
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%
    %Plot Trial Results %
    %%%%%%%%%%%%%%%%%%%%%
    close all
    
    
    
   
end




