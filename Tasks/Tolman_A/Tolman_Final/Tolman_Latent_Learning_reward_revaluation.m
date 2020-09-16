%%%%%%%%%%%%%%%
% Basic Setup %
%%%%%%%%%%%%%%%
clear all
NEW_NETWORK = 1;
FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning
rng(10);

global dt endTime t ERROR
dt=.05;
ERROR = false;


if NEW_NETWORK
    clearvars -except NEW_NETWORK FIXED_WEIGHTS dt endTime t ERROR
    [e, n] = create_Tolman_B_Net_core(FIXED_WEIGHTS,3);
elseif ~exist('n')
    error('Network has not been initialized. Set NEW_NETWORK=1')
    n.reset()
    e.reset()
end

load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
n.layerList{3}.decayRate=0;
% %%%%%%%%%%%%%%%%
% % Run Settings %
% %%%%%%%%%%%%%%%%
% MAX_TIME = 400; %maximum trial length
% MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
% NUM_TRIALS = 10;
% TEXT_OUTPUT = 1;
% DISPLAY = 0;
% 
% SETUP_LIST = [ %column 1: starting states; column 2: goals
%     1,999];
% REPEAT_MODE = 3;  %1 = alternate, 2 = repeat, 3 = first only
% %%%%%%%%%%%%%%%%%
% 
% %%%%%%%%%%%%%%%%
% % Set up batch %
% %%%%%%%%%%%%%%%%
% 
% numSetups = size(SETUP_LIST, 1);
% switch REPEAT_MODE
%     case 1 %alternate rows of setupList one at time
%         setupIndices = repmat(1:numSetups, 1, NUM_TRIALS/numSetups+1);
%         
%     case 2 %do each row of setupList a specified number of times before repeating
%         numRepeats = 50;
%         setupIteration = reshape(repmat(1:numSetups, numRepeats, 1), 1, numRepeats*numSetups);
%         setupIndices = repmat(setupIteration, 1, ceil(NUM_TRIALS / (numRepeats * numSetups)));
%         setupIndices = setupIndices(1:NUM_TRIALS);
%         
%     case 3 %only use first row
%         setupIndices = ones(1,NUM_TRIALS);
% end
% 
% %batchResults checks whether goal is reached, how long it takes, and how far from ideal
% %the goalGradient->goalGradient, state->adjacent, and transition->motor weights are;
% %6th column is the total error
% batchResults = nan(NUM_TRIALS, 6);


%     the first list of items will be displayed in real time
%     the second list will be projected onto an environment of the dimenions
%     specified by the third argument
% if DISPLAY
%     n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
% end
% 
% 
% %%%% SOME SETTINGS
% % n.layerList{4}.decayRate=0
% %%%%%%%%%%%%%%
% % Run Trials %
% %%%%%%%%%%%%%%
% 
% for trial_i = 1:NUM_TRIALS
%     
%     endTime=MAX_TIME;
%     t = dt;
%     tSinceMove = -1.5;
%     moved = 0;
%     exploreRate = .2;
%     rng(42 + trial_i);
% 
%     
%     startingState = SETUP_LIST(setupIndices(trial_i), 1);
%     goal = SETUP_LIST(setupIndices(trial_i), 2);
%     
%     lastState = startingState;
%     
%     n.reset();
%     e.reset(startingState);
%     n.so('n_env').set_vals(e.get_locVec.*4);
%     
%     if goal~=999
%         
%         n.so('n_goal').set_vals(goal); n.so('n_goal').scaleVals(20);
% 
%     end
% 
%     fprintf('Starting trial %i; FROM: %i TO: %i \n', trial_i, startingState, goal);
%     
%     
%     for t = dt:dt:endTime
%         [moved, action, descrip] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
%         if moved==1
%             if action > 0
%                 n.setNode('n_body', action);
%             end
%             n.so('n_env').set_vals(e.get_locVec .* 4);
%             n.setNode('n_novInhib', 1);
%             tSinceMove = 0;
%             
%             if TEXT_OUTPUT
%                 newState = e.curState();
%                 fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
%                 lastState = newState;
%             end
%             
%         else
%             tSinceMove = tSinceMove + dt;
%         end
%         n.update();
%         
%         %If at goal, allow a little time for learning
%         if ((e.curState == goal) && (tSinceMove > (MAX_STUCK_TIME - 1)))
%             disp('At goal')
%             endTime = t;
%             break
%         end
%         
%     end
%     
%     %%%%%%%%%%%%%%%%%%%%
%     %Log Trial Results %
%     %%%%%%%%%%%%%%%%%%%%
%     
%     batchResults(trial_i, 1) = (e.curState == goal);
%     
%     batchResults(trial_i, 2) = endTime;
%     
%     gradWeights = n.so('p_gradient_gradient').weights;
%     gradError = sum((gradWeights(:) - correctGradWeights(:)).^2);
%     batchResults(trial_i, 3) = gradError;
%     
%     adjWeights = n.so('p_state_adjacent').weights;
%     adjError = sum((adjWeights(:) - correctAdjWeights(:)).^2);
%     batchResults(trial_i, 4) = adjError;
%     
%     motorWeights = n.so('p_transOut_motorIn').weights;
%     motorError = sum((motorWeights(:) - correctMotorWeights(:)).^2);
%     batchResults(trial_i, 5) = motorError;
%     
%     totalError = gradError + adjError + motorError;
%     batchResults(trial_i, 6) = totalError;
%     state_log{trial_i}=n.layerList{1}.nodes{1}.valsLog;
%     
%     
% end





 n.logAll('layers');

%% %%%%%%%%%%%%%%% USE DIFFERENT STARTING STATES %%%%%%%%%
MAX_TIME = 400; %maximum trial length
MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
NUM_TRIALS = 1;
TEXT_OUTPUT = 1;
DISPLAY = 0;

SETUP_LIST = [ %column 1: starting states; column 2: goals
    1,14]; 
REPEAT_MODE = 1;  %1 = alternate, 2 = repeat, 3 = first only
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


%     the first list of items will be displayed in real time
%     the second list will be projected onto an environment of the dimenions 
%     specified by the third argument
if DISPLAY
    n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
end


%%%%%%%%%%%%%%
% Run Trials %
%%%%%%%%%%%%%%


for trial_i = 1:NUM_TRIALS
    
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
    
 
      if goal~=999
        
        n.so('n_goal').set_vals(goal); n.so('n_goal').scaleVals(20);

    end


    
    
    fprintf('Starting trial %i; FROM: %i TO: %i \n', trial_i, startingState, goal);
    
    
    
    for t = dt:dt:endTime

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
                fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                lastState = newState;
            end
            
        else
            tSinceMove = tSinceMove + dt;
        end
        n.update();
        
        %If at goal, allow a little time for learning
        if ((e.curState == goal) && (tSinceMove > (MAX_STUCK_TIME - 1)))
            disp('At goal')
            endTime = t;
            break
        end
        
    end
    
    %%%%%%%%%%%%%%%%%%%%
    %Log Trial Results %
    %%%%%%%%%%%%%%%%%%%%
    
    batchResults(trial_i, 1) = (e.curState == goal);
    
    batchResults(trial_i, 2) = endTime;
    
    gradWeights = n.so('p_gradient_gradient').weights;
    gradError = sum((gradWeights(:) - correctGradWeights(:)).^2);
    batchResults(trial_i, 3) = gradError;
    
    adjWeights = n.so('p_state_adjacent').weights;
    adjError = sum((adjWeights(:) - correctAdjWeights(:)).^2);
    batchResults(trial_i, 4) = adjError;
    
    motorWeights = n.so('p_transOut_motorIn').weights;
    motorError = sum((motorWeights(:) - correctMotorWeights(:)).^2);
    batchResults(trial_i, 5) = motorError;
    
    totalError = gradError + adjError + motorError;
    batchResults(trial_i, 6) = totalError;
    state_log{trial_i}=n.layerList{1}.nodes{1}.valsLog;
    
end



%% Can readdily switch between trials
MAX_TIME = 400; %maximum trial length
MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
NUM_TRIALS = 10;
TEXT_OUTPUT = 1;
DISPLAY = 0;

SETUP_LIST = [ %column 1: starting states; column 2: goals
    1,14;1 21]; 
REPEAT_MODE = 1;  %1 = alternate, 2 = repeat, 3 = first only
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


%     the first list of items will be displayed in real time
%     the second list will be projected onto an environment of the dimenions 
%     specified by the third argument
if DISPLAY
    n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
end


%%%%%%%%%%%%%%
% Run Trials %
%%%%%%%%%%%%%%


for trial_i = 1:NUM_TRIALS
    
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
    
 
      if goal~=999
        
        n.so('n_goal').set_vals(goal); n.so('n_goal').scaleVals(20);

    end


    
    
    fprintf('Starting trial %i; FROM: %i TO: %i \n', trial_i, startingState, goal);
    
    
    
    for t = dt:dt:endTime

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
                fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                lastState = newState;
            end
            
        else
            tSinceMove = tSinceMove + dt;
        end
        n.update();
        
        %If at goal, allow a little time for learning
        if ((e.curState == goal) && (tSinceMove > (MAX_STUCK_TIME - 1)))
            disp('At goal')
            endTime = t;
            break
        end
        
    end
    
    %%%%%%%%%%%%%%%%%%%%
    %Log Trial Results %
    %%%%%%%%%%%%%%%%%%%%
    
    batchResults(trial_i, 1) = (e.curState == goal);
    
    batchResults(trial_i, 2) = endTime;
    
    gradWeights = n.so('p_gradient_gradient').weights;
    gradError = sum((gradWeights(:) - correctGradWeights(:)).^2);
    batchResults(trial_i, 3) = gradError;
    
    adjWeights = n.so('p_state_adjacent').weights;
    adjError = sum((adjWeights(:) - correctAdjWeights(:)).^2);
    batchResults(trial_i, 4) = adjError;
    
    motorWeights = n.so('p_transOut_motorIn').weights;
    motorError = sum((motorWeights(:) - correctMotorWeights(:)).^2);
    batchResults(trial_i, 5) = motorError;
    
    totalError = gradError + adjError + motorError;
    batchResults(trial_i, 6) = totalError;
    state_log{trial_i}=n.layerList{1}.nodes{1}.valsLog;
    
end

%% Can readiyl switch between starting positions
MAX_TIME = 400; %maximum trial length
MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
NUM_TRIALS = 10;
TEXT_OUTPUT = 1;
DISPLAY = 0;

SETUP_LIST = [ %column 1: starting states; column 2: goals
    21,14;8 14]; 
REPEAT_MODE = 1;  %1 = alternate, 2 = repeat, 3 = first only
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


%     the first list of items will be displayed in real time
%     the second list will be projected onto an environment of the dimenions 
%     specified by the third argument
if DISPLAY
    n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
end


%%%%%%%%%%%%%%
% Run Trials %
%%%%%%%%%%%%%%


for trial_i = 1:NUM_TRIALS
    
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
    
 
      if goal~=999
        
        n.so('n_goal').set_vals(goal); n.so('n_goal').scaleVals(20);

    end


    
    
    fprintf('Starting trial %i; FROM: %i TO: %i \n', trial_i, startingState, goal);
    
    
    
    for t = dt:dt:endTime

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
                fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                lastState = newState;
            end
            
        else
            tSinceMove = tSinceMove + dt;
        end
        n.update();
        
        %If at goal, allow a little time for learning
        if ((e.curState == goal) && (tSinceMove > (MAX_STUCK_TIME - 1)))
            disp('At goal')
            endTime = t;
            break
        end
        
    end
    
    %%%%%%%%%%%%%%%%%%%%
    %Log Trial Results %
    %%%%%%%%%%%%%%%%%%%%
    
    batchResults(trial_i, 1) = (e.curState == goal);
    
    batchResults(trial_i, 2) = endTime;
    
    gradWeights = n.so('p_gradient_gradient').weights;
    gradError = sum((gradWeights(:) - correctGradWeights(:)).^2);
    batchResults(trial_i, 3) = gradError;
    
    adjWeights = n.so('p_state_adjacent').weights;
    adjError = sum((adjWeights(:) - correctAdjWeights(:)).^2);
    batchResults(trial_i, 4) = adjError;
    
    motorWeights = n.so('p_transOut_motorIn').weights;
    motorError = sum((motorWeights(:) - correctMotorWeights(:)).^2);
    batchResults(trial_i, 5) = motorError;
    
    totalError = gradError + adjError + motorError;
    batchResults(trial_i, 6) = totalError;
    state_log{trial_i}=n.layerList{1}.nodes{1}.valsLog;
    
end
