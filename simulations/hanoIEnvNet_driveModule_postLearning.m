% runs small 6-state environment simulation with goal learning

%%%%%%%%%%%%%%%
% Basic Setup %
%%%%%%%%%%%%%%%
NEW_NETWORK = 1;
SET_WEIGHTS = 1; %initialize core architecture weights to correct values
SET_GOAL_WEIGHTS = 1; %initialize drive -> goal weights to correct values

%seed=randi(100)
seed = 43;
rng(seed);

global dt endTime t 
dt=.05;


if NEW_NETWORK
    clearvars -except NEW_NETWORK SET_WEIGHTS SET_GOAL_WEIGHTS dt endTime t ERROR 
    [e, n] = Hanoi_GOALS_Net_core(SET_WEIGHTS, SET_GOAL_WEIGHTS,1);
elseif ~exist('n')
    error('Network has not been initialized. Set NEW_NETWORK=1')
end

load('stateSpaces/smallEnv_weights.mat') %used to compare with model weights to assess learning

%%%%%%%%%%%%%%%%
% Run Settings %
%%%%%%%%%%%%%%%%
MAX_TIME = 200; %maximum trial length
MAX_STUCK_TIME = pi * 4; %maximum time to not move
NUM_TRIALS = 1;
TEXT_OUTPUT = 1;
DISPLAY = 0;

SETUP_LIST = [4, 99; 5, 99]; %column 1: starting states; column 2: goals
REPEAT_MODE = 3;  %1 = alternate, 2 = repeat, 3 = first only
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
% Set up batch %
%%%%%%%%%%%%%%%%

numSetups = size(SETUP_LIST, 1);
switch REPEAT_MODE
    case 1 %alternate rows of setupList one at time
        setupIndices = repmat(1:numSetups, 1, NUM_TRIALS/numSetups+1);

    case 2 %do each row of setupList a specified number of times before repeating
        numRepeats = 4;
        setupIteration = reshape(repmat(1:numSetups, numRepeats, 1), 1, numRepeats*numSetups);
        setupIndices = repmat(setupIteration, 1, ceil(NUM_TRIALS / (numRepeats * numSetups)));
        setupIndices = setupIndices(1:NUM_TRIALS);

    case 3 %only use first row
         setupIndices = ones(1,NUM_TRIALS);
end

%batchResults checks whether goal is reached, how long it takes, and how far from ideal
%the goalGradient->goalGradient, state->adjacent, and transition->motor weights are 
batchResults = nan(NUM_TRIALS, 5); 

n.logAll();

%the first list of items will be displayed in real time
%the second list will be projected onto an environment of the dimenions 
%specified by the third argument
if DISPLAY
    n.set_display({'gradient', 'p_gradient_gradient'},{'state'}, [2,3]);
end

%%%%%%%%%%%%%%
% Run Trials %
%%%%%%%%%%%%%%
% n.so('p_drives_goal').learnRate = 0;
for trial_i = 1:NUM_TRIALS
    endTime = MAX_TIME;
    t = dt;
    tSinceMove = -1.5;
    moved = 0;
    exploreRate = 0;

    startingState = SETUP_LIST(setupIndices(trial_i), 1);

    lastState = startingState;

    n.reset();
    e.reset(startingState); 

    n.so('n_env').set_vals(e.get_locVec.*4);
    n.setNode('n_drives',[1, 0.1]);


    fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
    dbx = nan(MAX_TIME/dt, 2);

    for t = dt:dt:endTime     

        %%%%%%%%%%%%%%%%%%%%%%%%
        % Manual Drive Control %
        %%%%%%%%%%%%%%%%%%%%%%%%
        if 0          
%         if atTime(20)
%             n.so('n_drives').pulse({2, 5});
%         end


%         if atTime(30)
%             n.so('n_drives').pulse([1, 5]);
%         end
        end


        %%%%%%%%%%
        % Action %
        %%%%%%%%%%


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

        %diminish drives if reward achieved
        if tSinceMove > 2
            newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
            n.so('n_drives').set_vals(newDrives);
        end

        n.update();
        e.update();

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

    driveWeightError = n.so('p_drives_goal').weights - e.getRewardWeights();


    %%%%%%%%%%%%%%%%%%%%%
    %Plot Trial Results %
    %%%%%%%%%%%%%%%%%%%%%
    close all



%     n.gatePlot('p_drives_goal','',50);

    n.plot({'state', 'drives', 'adjacent', 'goal', 'next', 'gradient', 'n_drives', 'n_novInhib'}, [dt,200]); %choose next state
%     n.plot({'state', 'n_body', 'prevState1', 'prevMotor', 'prevState2', 'transObs'}, 50); %monitor the past
%     n.plot({'state', 'transObs', 'motorIn', 'transDes', 'motorOut', 'transOut' 'n_body', 'prevMotor'}, 50); %take action

%     n.gatePlot('p_drives_goal',[]);
%     n.gatePlot('p_gradient_next','n_subOsc');

%     %drive motor weights
%     figure(41)
%     goalWeights=n.so('p_drives_goal').weights;
%     imagesc(goalWeights)
%     title('Goal Weights'); 
%     colorbar


    %No idea why, but this plot does not appear but makes all previous
    %plots appear within the loop
    if trial_i < NUM_TRIALS
        figure(99)
    end

end


%%%%%%%%%%%%%%
%Export Data %
%%%%%%%%%%%%%%

if 0 
    %activity and weight data
    saveList = {'all'}; %specify layers or projections to export data from
    saveDir = '';

    resp = input(['Save data to directory: ' saveDir '  (y/n)?'], 's');
    if ismember(resp, {'y', 'Y'})
            n.exportPlotData(saveList, [dt,endTime], '', saveDir);

        %batch data
        if 0
           batchSaveDir = '';
           batchSaveName = '';
           csvwrite([batchSaveDir batchSaveName], batchResults) 
        end    
    else
        disp('Data not saved')
    end   
end

%%%%%%%%%%%%%%%
% Batch Plots %
%%%%%%%%%%%%%%%

if 0 
   figure()
   plotData = batchResults(:,2:end);
   numPlots = size(plotData,2);

   titles = {'Trial Time', 'Prox Error', 'Adj Error', 'Motor Error'};

   for plot_i = 1:numPlots
      subplot(numPlots, 1, plot_i);
      hold on
      plot(plotData(:, plot_i), 'LineWidth', 2);
      title(titles{plot_i});
   end
   xlabel('Trial');
end


%%%%%%%%%%%%%%%
% Trial Plots %
%%%%%%%%%%%%%%%
if 0

    %view all possible plotting items with n.names()
    plotItems = {'state', 'adjacent', 'next','transOut', 'motorIn',  'motorOut', 'n_subOsc', 'n_body'};
    n.plot(plotItems, 50)

    n.gatePlot('p_gradient_gradient','n_subOsc',50)
    n.gatePlot('p_transOut_motorIn','n_subOsc', [1,20])

    %raw adjacency weights
    figure(11)
    adjWeights = n.so('p_state_adjacent').weights;
    imagesc(adjWeights);
    title('adjacent Weights');
    colorbar

    %raw - correct adjacency weights
    figure(12)
    adjWeights = n.so('p_state_adjacent').weights;
    adjWeightError = adjWeights - correctAdjWeights
    imagesc(adjWeightError, [-1, 1]);
    title('adjacent Weights');
    colorbar

    %raw goal gradient weights
    figure(21)
    gradWeights = n.so('p_gradient_gradient').weights;
    imagesc(gradWeights)
    title('gradient Weights');
    colorbar

    %raw - correct goal gradient weights
    figure(22)
    gradWeights = n.so('p_gradient_gradient').weights;
    gradWeightError = gradWeights - correctGradWeights;
    imagesc(gradWeightError, [-1, 1]);
    title('real - correct gradient Weights');
    colorbar

    %raw motor weights
    figure(31)
    motorWeights=n.so('p_transOut_motorIn').weights;
    imagesc(motorWeights)
    title('Action Weights'); 
    colorbar

    %raw - correct motor weights
    figure(32)
    motorWeights = n.so('p_transOut_motorIn').weights;
    motorWeightError = motorWeights - correctMotorWeights;
    imagesc(motorWeightError, [-1, 1]) 
    title('Action Weights');
    colorbar

    %drive motor weights
    figure(41)
    goalWeights=n.so('p_drives_goal').weights;
    imagesc(goalWeights)
    title('Goal Weights'); 
    colorbar
end



    
    