
clear all


warning('off','all')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%
% Basic Setup %
%%%%%%%%%%%%%%%

for type_run=1:2
    NEW_NETWORK = 1;
    FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning
    rng(45);
    clearvars e n
    
    global dt endTime t ERROR
    dt=.05;
    ERROR = false;
    
    if NEW_NETWORK
        [e, n] = create_Tolman_GOALS_Net_core(FIXED_WEIGHTS,1,1);
    elseif ~exist('n')
        error('Network has not been initialized. Set NEW_NETWORK=1')
        n.reset()
        e.reset()
    end
    
    load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
    n.logAll('layers');
    
    % n.so('p_gradient_gradient').weights=n.so('p_gradient_gradient').weights.*4;
    
    %%%%%%%%%%%%%%%%
    % Run Settings %
    %%%%%%%%%%%%%%%%
    MAX_TIME =600; %maximum trial length
    MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
    NUM_TRIALS = 1;
    TEXT_OUTPUT = 1;
    DISPLAY = 0;
    
    SETUP_LIST = [ %column 1: starting states; column 2: goals
        1,999];
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
    
    if type_run==1
        %Pick up closest first, despite not highest drive.
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            19,999];
        n.setNode('n_drives',[0.1, 0.1,1]);
        
    elseif type_run==2
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            19,999];
        n.setNode('n_drives',[0.01, 0.1,1]);
        
    elseif type_run==3
        %Pick up closest first, despite not highest drive.
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            1,999];
        n.setNode('n_drives',[0.1, 0.1,1]);
    elseif type_run==4
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            1,999];
        n.setNode('n_drives',[0.001, 0.1,1]);
        
    end
    
    keep_go=1;
    %%%%%%%%%%%%%%
    % Run Trials %
    %%%%%%%%%%%%%%
    for trial_i = 1:NUM_TRIALS
        CS=[];
        order=[];
        hit=[1 1 1];
        endTime = MAX_TIME;
        t = dt;
        tSinceMove = -1.5;
        moved = 0;
        exploreRate = 0;
        
        startingState = SETUP_LIST(setupIndices(trial_i), 1);
        
        lastState = startingState;
        
        n.reset();
        e.reset(startingState);
        
        %Set up things
        
        n.so('n_env').set_vals(e.get_locVec.*4);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% SETUP DRIVES
        if type_run==1
            set_dr(1,:)=[0.1, 0.1,1];
            n.setNode('n_drives',[0.1, 0.1,1]);
        elseif type_run==2
            set_dr(2,:)=[0.01, 0.1,1];
            n.setNode('n_drives',[0.01, 0.1,1]);
        elseif type_run==3
            set_dr(3,:)=[0.1, 0.1,1];

            n.setNode('n_drives',[0.1, 0.1,1]);
        elseif type_run==4
            set_dr(4,:)=[0.01, 0.1,1];
            n.setNode('n_drives',[0.01, 0.1,1]);
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
        dbx = nan(MAX_TIME/dt, 2);
        
        for t = dt:dt:endTime
            
            %%%%%%%%%%
            % Action %
            %%%%%%%%%%
            if keep_go==1
                
                [moved, action, descrip,desnum] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
                if moved==1
                    
                    if action > 0
                        n.setNode('n_body', action);
                    end
                    n.so('n_env').set_vals(e.get_locVec .* 4);
                    n.setNode('n_novInhib', 1);
                    tSinceMove = 0;
                    
                    if TEXT_OUTPUT
                        CS=[CS;e.curState()];
                        newState = e.curState();
                        fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                        lastState = newState;
                    end
                    
                    
                    
                else
                    tSinceMove = tSinceMove + dt;
                end
                %diminish drives if reward achieved
                if tSinceMove > 2
                    
                    %Check whether in a state that fulfills a drive/reward
                    alllow=find(e.getReward);
                    if or(or(alllow==1,alllow==2),alllow==3)
                                            n.setNode('n_GoalInhib', 1);

                    end
                    if ~find(order==alllow)
                        order=[order,alllow];
                    end
                    hit([alllow])=0;
                    %             newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
                    newDrives = n.so('n_drives').get_vals() .* hit;
                    
                    n.so('n_drives').set_vals(newDrives);
                    
                    if sum(hit)==0
                        keep_go=0;
                        time_break=t+5;
                    end
                    
                end
                
                
                n.update();
                e.update();
                
            else
                n.update();
                e.update();
                
                if t>time_break
                    break
                end
                
                
            end
        end
        OUTT{type_run,1,1}=[startingState;CS];
%         OUTT{type_run,2}=[order];
        
    end
    
    
 [~,r]=(min(abs((((1:size(n.so('drives').actLog,1))*dt))-t)))
    
    timers=((1:size(n.so('drives').actLog,1))*dt);
    timers=timers(1:r);
    figure;
    hold on;
    plot(timers,n.so('drives').actLog(1:r,1),'Color','b','LineWidth',2)
    plot(timers,n.so('drives').actLog(1:r,2),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
    plot(timers,n.so('drives').actLog(1:r,3),'Color','r','LineWidth',2)
    title(strcat('Drives Run Order A:',num2str(type_run)))
    ss=xlim;
    xlim([-4 ss(2)])
    set(findall(gcf,'-property','FontSize'),'FontSize',13)
    set(findall(gcf,'-property','FontName'),'FontName','arial')
    set(findall(gcf,'-property','linewidth'),'linewidth',2)
    ylim([-0.0036  0.5])
    set(gca,'TickDir','out'); % The only other option is 'in'
    
    figure;
    hold on;
    plot(timers,n.so('goal').actLog(1:r,14),'Color','b','LineWidth',2)
    plot(timers,n.so('goal').actLog(1:r,21),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
    plot(timers,n.so('goal').actLog(1:r,22),'Color','r','LineWidth',2)
    plot(timers,n.so('goal').actLog(1:r,1:13),'LineWidth',0.5)
    title(strcat('Goal Run Order A:',num2str(type_run)))
    ss=xlim;
    xlim([-4 ss(2)])
    set(findall(gcf,'-property','FontSize'),'FontSize',13)
    set(findall(gcf,'-property','FontName'),'FontName','arial')
    set(findall(gcf,'-property','linewidth'),'linewidth',2)
    ylim([-0.0036  0.5])
    set(gca,'TickDir','out'); % The only other option is 'in'
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars -except OUTT


warning('off','all')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%
% Basic Setup %
%%%%%%%%%%%%%%%

for type_run=3:4
    NEW_NETWORK = 1;
    FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning
    rng(45);
    clearvars e n
    
    global dt endTime t ERROR
    dt=.05;
    ERROR = false;
    
    if NEW_NETWORK
        [e, n] = create_Tolman_GOALS_Net_core(FIXED_WEIGHTS,1,1);
    elseif ~exist('n')
        error('Network has not been initialized. Set NEW_NETWORK=1')
        n.reset()
        e.reset()
    end
    
    load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
    n.logAll('layers');
    
    % n.so('p_gradient_gradient').weights=n.so('p_gradient_gradient').weights.*4;
    
    %%%%%%%%%%%%%%%%
    % Run Settings %
    %%%%%%%%%%%%%%%%
    MAX_TIME =600; %maximum trial length
    MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
    NUM_TRIALS = 1;
    TEXT_OUTPUT = 1;
    DISPLAY = 0;
    
    SETUP_LIST = [ %column 1: starting states; column 2: goals
        1,999];
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
    
    if type_run==1
        %Pick up closest first, despite not highest drive.
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            19,999];
        n.setNode('n_drives',[0.1, 0.1,1]);
        
    elseif type_run==2
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            19,999];
        n.setNode('n_drives',[0.01, 0.1,1]);
        
    elseif type_run==3
        %Pick up closest first, despite not highest drive.
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            1,999];
        n.setNode('n_drives',[0.1, 0.1,1]);
    elseif type_run==4
        SETUP_LIST = [ %column 1: starting states; column 2: goals
            1,999];
        n.setNode('n_drives',[0.00001, 0.1,1]);
        
    end
    
    keep_go=1;
    %%%%%%%%%%%%%%
    % Run Trials %
    %%%%%%%%%%%%%%
    for trial_i = 1:NUM_TRIALS
        CS=[];
        order=[];
        hit=[1 1 1];
        endTime = MAX_TIME;
        t = dt;
        tSinceMove = -1.5;
        moved = 0;
        exploreRate = 0;
        
        startingState = SETUP_LIST(setupIndices(trial_i), 1);
        
        lastState = startingState;
        
        n.reset();
        e.reset(startingState);
        
        %Set up things
        
        n.so('n_env').set_vals(e.get_locVec.*4);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% SETUP DRIVES
        if type_run==1
            n.setNode('n_drives',[0.1, 0.1,1]);
        elseif type_run==2
            n.setNode('n_drives',[0.01, 0.1,1]);
        elseif type_run==3
            n.setNode('n_drives',[0.1, 0.1,1]);
        elseif type_run==4
            n.setNode('n_drives',[0.00001, 0.1,1]);
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
        dbx = nan(MAX_TIME/dt, 2);
        
        for t = dt:dt:endTime
            
            %%%%%%%%%%
            % Action %
            %%%%%%%%%%
            if keep_go==1
                
                [moved, action, descrip,desnum] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
                if moved==1
                    
                    if action > 0
                        n.setNode('n_body', action);
                    end
                    n.so('n_env').set_vals(e.get_locVec .* 4);
                    n.setNode('n_novInhib', 1);
                    tSinceMove = 0;
                    
                    if TEXT_OUTPUT
                        CS=[CS;e.curState()];
                        newState = e.curState();
                        fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
                        lastState = newState;
                    end
                    
                    
                    
                else
                    tSinceMove = tSinceMove + dt;
                end
                %diminish drives if reward achieved
                if tSinceMove > 2
                    %              e.getReward()%The rewards you received in a state. short lived.
                    
                    %Check whether in a state that fulfills a drive/reward
                    alllow=find(e.getReward);
                    if or(or(alllow==1,alllow==2),alllow==3)
                    end
                    if ~find(order==alllow)
                        order=[order,alllow];
                    end
                    hit([alllow])=0;
                    %             newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
                    newDrives = n.so('n_drives').get_vals() .* hit;
                    
                    n.so('n_drives').set_vals(newDrives);
                    
                    if sum(hit)==0
                        keep_go=0;
                        time_break=t+5;
                    end
                    
                end
                
                
                n.update();
                e.update();
                
            else
                n.update();
                e.update();
                
                if t>time_break
                    break
                end
                
                
            end
        end
        OUTT{type_run,1,1}=[startingState;CS];
        
    end
    
    
 [~,r]=(min(abs((((1:size(n.so('drives').actLog,1))*dt))-t)))
    
    timers=((1:size(n.so('drives').actLog,1))*dt);
    timers=timers(1:r);
    figure;
    hold on;
    plot(timers,n.so('drives').actLog(1:r,1),'Color','b','LineWidth',2)
    plot(timers,n.so('drives').actLog(1:r,2),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
    plot(timers,n.so('drives').actLog(1:r,3),'Color','r','LineWidth',2)
    title(strcat('Drives Run Order A:',num2str(type_run)))
    ss=xlim;
    xlim([-4 ss(2)])
    set(findall(gcf,'-property','FontSize'),'FontSize',13)
    set(findall(gcf,'-property','FontName'),'FontName','arial')
    set(findall(gcf,'-property','linewidth'),'linewidth',2)
    ylim([-0.0036  0.5])
    set(gca,'TickDir','out'); % The only other option is 'in'
    
    
    figure;
    hold on;
    plot(timers,n.so('goal').actLog(1:r,14),'Color','b','LineWidth',2)
    plot(timers,n.so('goal').actLog(1:r,21),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
    plot(timers,n.so('goal').actLog(1:r,22),'Color','r','LineWidth',2)
    plot(timers,n.so('goal').actLog(1:r,1:13),'LineWidth',0.5)
    title(strcat('Goal Run Order A:',num2str(type_run)))
    ss=xlim;
    xlim([-4 ss(2)])
    set(findall(gcf,'-property','FontSize'),'FontSize',13)
    set(findall(gcf,'-property','FontName'),'FontName','arial')
    set(findall(gcf,'-property','linewidth'),'linewidth',2)
    ylim([-0.0036  0.5])
    set(gca,'TickDir','out'); % The only other option is 'in'
    
end

% %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SAME AS ABOVE, different time constant
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% for drives function
% 
% clear all
% 
% 
% warning('off','all')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%
% % Basic Setup %
% %%%%%%%%%%%%%%%
% 
% for type_run=1:2
%     NEW_NETWORK = 1;
%     FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning
%     rng(45);
%     clearvars e n
%     
%     global dt endTime t ERROR
%     dt=.05;
%     ERROR = false;
%     
%     if NEW_NETWORK
%         [e, n] = create_Tolman_GOALS_Net_core(FIXED_WEIGHTS,1,0.4);
%     elseif ~exist('n')
%         error('Network has not been initialized. Set NEW_NETWORK=1')
%         n.reset()
%         e.reset()
%     end
%     
%     load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
%     n.logAll('layers');
%     
%     % n.so('p_gradient_gradient').weights=n.so('p_gradient_gradient').weights.*4;
%     
%     %%%%%%%%%%%%%%%%
%     % Run Settings %
%     %%%%%%%%%%%%%%%%
%     MAX_TIME =600; %maximum trial length
%     MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
%     NUM_TRIALS = 1;
%     TEXT_OUTPUT = 1;
%     DISPLAY = 0;
%     
%     SETUP_LIST = [ %column 1: starting states; column 2: goals
%         1,999];
%     REPEAT_MODE = 2;  %1 = alternate, 2 = repeat, 3 = first only
%     
%     %%%%%%%%%%%%%%%%%
%     %%%%%%%%%%%%%%%%
%     % Set up batch %
%     %%%%%%%%%%%%%%%%
%     
%     numSetups = size(SETUP_LIST, 1);
%     switch REPEAT_MODE
%         case 1 %alternate rows of setupList one at time
%             setupIndices = repmat(1:numSetups, 1, NUM_TRIALS/numSetups+1);
%             
%         case 2 %do each row of setupList a specified number of times before repeating
%             numRepeats = 50;
%             setupIteration = reshape(repmat(1:numSetups, numRepeats, 1), 1, numRepeats*numSetups);
%             setupIndices = repmat(setupIteration, 1, ceil(NUM_TRIALS / (numRepeats * numSetups)));
%             setupIndices = setupIndices(1:NUM_TRIALS);
%             
%         case 3 %only use first row
%             setupIndices = ones(1,NUM_TRIALS);
%     end
%     
%     if type_run==1
%         %Pick up closest first, despite not highest drive.
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             19,999];
%         n.setNode('n_drives',[0.1, 0.1,1]);
%         
%     elseif type_run==2
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             19,999];
%         n.setNode('n_drives',[0.01, 0.1,1]);
%         
%     elseif type_run==3
%         %Pick up closest first, despite not highest drive.
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             1,999];
%         n.setNode('n_drives',[0.1, 0.1,1]);
%     elseif type_run==4
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             1,999];
%         n.setNode('n_drives',[0.01, 0.1,1]);
%         
%     end
%     
%     keep_go=1;
%     %%%%%%%%%%%%%%
%     % Run Trials %
%     %%%%%%%%%%%%%%
%     for trial_i = 1:NUM_TRIALS
%         CS=[];
%         order=[];
%         hit=[1 1 1];
%         endTime = MAX_TIME;
%         t = dt;
%         tSinceMove = -1.5;
%         moved = 0;
%         exploreRate = 0;
%         
%         startingState = SETUP_LIST(setupIndices(trial_i), 1);
%         
%         lastState = startingState;
%         
%         n.reset();
%         e.reset(startingState);
%         
%         %Set up things
%         
%         n.so('n_env').set_vals(e.get_locVec.*4);
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %%% SETUP DRIVES
%         if type_run==1
%             n.setNode('n_drives',[0.1, 0.1,1]);
%         elseif type_run==2
%             n.setNode('n_drives',[0.01, 0.1,1]);
%         elseif type_run==3
%             n.setNode('n_drives',[0.1, 0.1,1]);
%         elseif type_run==4
%             n.setNode('n_drives',[0.01, 0.1,1]);
%             
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
%         dbx = nan(MAX_TIME/dt, 2);
%         
%         for t = dt:dt:endTime
%             
%             %%%%%%%%%%
%             % Action %
%             %%%%%%%%%%
%             if keep_go==1
%                 
%                 [moved, action, descrip,desnum] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
%                 if moved==1
%                     
%                     if action > 0
%                         n.setNode('n_body', action);
%                     end
%                     n.so('n_env').set_vals(e.get_locVec .* 4);
%                     n.setNode('n_novInhib', 1);
%                     tSinceMove = 0;
%                     
%                     if TEXT_OUTPUT
%                         CS=[CS;e.curState()];
%                         newState = e.curState();
%                         fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
%                         lastState = newState;
%                     end
%                     
%                     
%                     
%                 else
%                     tSinceMove = tSinceMove + dt;
%                 end
%                 %diminish drives if reward achieved
%                 if tSinceMove > 2
%                     %              e.getReward()%The rewards you received in a state. short lived.
%                     
%                     %Check whether in a state that fulfills a drive/reward
%                     alllow=find(e.getReward);
%                     if or(or(alllow==1,alllow==2),alllow==3)
%                     end
%                     if ~find(order==alllow)
%                         order=[order,alllow];
%                     end
%                     hit([alllow])=0;
%                     %             newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
%                     newDrives = n.so('n_drives').get_vals() .* hit;
%                     
%                     n.so('n_drives').set_vals(newDrives);
%                     
%                     if sum(hit)==0
%                         keep_go=0;
%                         time_break=t+5;
%                     end
%                     
%                 end
%                 
%                 
%                 n.update();
%                 e.update();
%                 
%             else
%                 n.update();
%                 e.update();
%                 
%                 if t>time_break
%                     break
%                 end
%                 
%                 
%             end
%         end
%         OUTT{type_run,2,1}=[startingState;CS];
%         
%     end
%     
%  [~,r]=(min(abs((((1:size(n.so('drives').actLog,1))*dt))-t)))
%     
%     timers=((1:size(n.so('drives').actLog,1))*dt);
%     timers=timers(1:r);
%     figure;
%     hold on;
%     plot(timers,n.so('drives').actLog(1:r,1),'Color','b','LineWidth',2)
%     plot(timers,n.so('drives').actLog(1:r,2),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
%     plot(timers,n.so('drives').actLog(1:r,3),'Color','r','LineWidth',2)
%     title(strcat('Drives Run Order B:',num2str(type_run)))
%     ss=xlim;
%     xlim([-4 ss(2)])
%     set(findall(gcf,'-property','FontSize'),'FontSize',13)
%     set(findall(gcf,'-property','FontName'),'FontName','arial')
%     set(findall(gcf,'-property','linewidth'),'linewidth',2)
%     ylim([-0.0036  0.5])
%     set(gca,'TickDir','out'); % The only other option is 'in'
%     
%     figure;
%     hold on;
%     plot(timers,n.so('goal').actLog(1:r,14),'Color','b','LineWidth',2)
%     plot(timers,n.so('goal').actLog(1:r,21),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
%     plot(timers,n.so('goal').actLog(1:r,22),'Color','r','LineWidth',2)
%     plot(timers,n.so('goal').actLog(1:r,1:13),'LineWidth',0.5)
%     title(strcat('Goal Run Order B:',num2str(type_run)))
%     ss=xlim;
%     xlim([-4 ss(2)])
%     set(findall(gcf,'-property','FontSize'),'FontSize',13)
%     set(findall(gcf,'-property','FontName'),'FontName','arial')
%     set(findall(gcf,'-property','linewidth'),'linewidth',2)
%     ylim([-0.0036  0.5])
%     set(gca,'TickDir','out'); % The only other option is 'in'
% 
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clearvars -except OUTT
% 
% 
% warning('off','all')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%
% % Basic Setup %
% %%%%%%%%%%%%%%%
% 
% for type_run=3:4
%     NEW_NETWORK = 1;
%     FIXED_WEIGHTS = 1; %initialize weights to correct values and turn off learning
%     rng(45);
%     clearvars e n
%     
%     global dt endTime t ERROR
%     dt=.05;
%     ERROR = false;
%     
%     if NEW_NETWORK
%         [e, n] = create_Tolman_GOALS_Net_core(FIXED_WEIGHTS,1,0.4);
%     elseif ~exist('n')
%         error('Network has not been initialized. Set NEW_NETWORK=1')
%         n.reset()
%         e.reset()
%     end
%     
%     load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
%     n.logAll('layers');
%     
%     % n.so('p_gradient_gradient').weights=n.so('p_gradient_gradient').weights.*4;
%     
%     %%%%%%%%%%%%%%%%
%     % Run Settings %
%     %%%%%%%%%%%%%%%%
%     MAX_TIME =600; %maximum trial length
%     MAX_STUCK_TIME = pi*4 ; %maximum time to have a goal and not move
%     NUM_TRIALS = 1;
%     TEXT_OUTPUT = 1;
%     DISPLAY = 0;
%     
%     SETUP_LIST = [ %column 1: starting states; column 2: goals
%         1,999];
%     REPEAT_MODE = 2;  %1 = alternate, 2 = repeat, 3 = first only
%     
%     %%%%%%%%%%%%%%%%%
%     %%%%%%%%%%%%%%%%
%     % Set up batch %
%     %%%%%%%%%%%%%%%%
%     
%     numSetups = size(SETUP_LIST, 1);
%     switch REPEAT_MODE
%         case 1 %alternate rows of setupList one at time
%             setupIndices = repmat(1:numSetups, 1, NUM_TRIALS/numSetups+1);
%             
%         case 2 %do each row of setupList a specified number of times before repeating
%             numRepeats = 50;
%             setupIteration = reshape(repmat(1:numSetups, numRepeats, 1), 1, numRepeats*numSetups);
%             setupIndices = repmat(setupIteration, 1, ceil(NUM_TRIALS / (numRepeats * numSetups)));
%             setupIndices = setupIndices(1:NUM_TRIALS);
%             
%         case 3 %only use first row
%             setupIndices = ones(1,NUM_TRIALS);
%     end
%     
%     if type_run==1
%         %Pick up closest first, despite not highest drive.
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             19,999];
%         n.setNode('n_drives',[0.1, 0.1,1]);
%         
%     elseif type_run==2
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             19,999];
%         n.setNode('n_drives',[0.01, 0.1,1]);
%         
%     elseif type_run==3
%         %Pick up closest first, despite not highest drive.
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             1,999];
%         n.setNode('n_drives',[0.1, 0.1,1]);
%     elseif type_run==4
%         SETUP_LIST = [ %column 1: starting states; column 2: goals
%             1,999];
%         n.setNode('n_drives',[0.01, 0.1,1]);
%         
%     end
%     
%     keep_go=1;
%     %%%%%%%%%%%%%%
%     % Run Trials %
%     %%%%%%%%%%%%%%
%     for trial_i = 1:NUM_TRIALS
%         CS=[];
%         order=[];
%         hit=[1 1 1];
%         endTime = MAX_TIME;
%         t = dt;
%         tSinceMove = -1.5;
%         moved = 0;
%         exploreRate = 0;
%         
%         startingState = SETUP_LIST(setupIndices(trial_i), 1);
%         
%         lastState = startingState;
%         
%         n.reset();
%         e.reset(startingState);
%         
%         %Set up things
%         
%         n.so('n_env').set_vals(e.get_locVec.*4);
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %%% SETUP DRIVES
%         if type_run==1
%             n.setNode('n_drives',[0.1, 0.1,1]);
%         elseif type_run==2
%             n.setNode('n_drives',[0.01, 0.1,1]);
%         elseif type_run==3
%             n.setNode('n_drives',[0.1, 0.1,1]);
%         elseif type_run==4
%             n.setNode('n_drives',[0.01, 0.1,1]);
%             
%         end
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         fprintf('Starting trial %i; FROM: %i \n', trial_i, startingState);
%         dbx = nan(MAX_TIME/dt, 2);
%         
%         for t = dt:dt:endTime
%             
%             %%%%%%%%%%
%             % Action %
%             %%%%%%%%%%
%             if keep_go==1
%                 
%                 [moved, action, descrip,desnum] = e.emwalk(n.so('n_body').vals, MAX_STUCK_TIME, tSinceMove, exploreRate);
%                 if moved==1
%                     
%                     if action > 0
%                         n.setNode('n_body', action);
%                     end
%                     n.so('n_env').set_vals(e.get_locVec .* 4);
%                     n.setNode('n_novInhib', 1);
%                     tSinceMove = 0;
%                     
%                     if TEXT_OUTPUT
%                         CS=[CS;e.curState()];
%                         newState = e.curState();
%                         fprintf('%s: Moving from %i to %i at t = %.2f\n', descrip, lastState, newState, t);
%                         lastState = newState;
%                     end
%                     
%                     
%                     
%                 else
%                     tSinceMove = tSinceMove + dt;
%                 end
%                 %diminish drives if reward achieved
%                 if tSinceMove > 2
%                     %              e.getReward()%The rewards you received in a state. short lived.
%                     
%                     %Check whether in a state that fulfills a drive/reward
%                     alllow=find(e.getReward);
%                     if or(or(alllow==1,alllow==2),alllow==3)
%                     end
%                     if ~find(order==alllow)
%                         order=[order,alllow];
%                     end
%                     hit([alllow])=0;
%                     %             newDrives = n.so('n_drives').get_vals() .* ~e.getReward();
%                     newDrives = n.so('n_drives').get_vals() .* hit;
%                     
%                     n.so('n_drives').set_vals(newDrives);
%                     
%                     if sum(hit)==0
%                         keep_go=0;
%                         time_break=t+5;
%                     end
%                     
%                 end
%                 
%                 
%                 n.update();
%                 e.update();
%                 
%             else
%                 n.update();
%                 e.update();
%                 
%                 if t>time_break
%                     break
%                 end
%                 
%                 
%             end
%         end
%         OUTT{type_run,2,1}=[startingState;CS];
%         OUTT{type_run,2}=[order];
%         
%     end
%     
%     
%     
%     [~,r]=(min(abs((((1:size(n.so('drives').actLog,1))*dt))-t)))
%     
%     timers=((1:size(n.so('drives').actLog,1))*dt);
%     timers=timers(1:r);
%     figure;
%     hold on;
%     plot(timers,n.so('drives').actLog(1:r,1),'Color','b','LineWidth',2)
%     plot(timers,n.so('drives').actLog(1:r,2),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
%     plot(timers,n.so('drives').actLog(1:r,3),'Color','r','LineWidth',2)
%     title(strcat('Drives Run Order B:',num2str(type_run)))
%     ss=xlim;
%     xlim([-4 ss(2)])
%     set(findall(gcf,'-property','FontSize'),'FontSize',13)
%     set(findall(gcf,'-property','FontName'),'FontName','arial')
%     set(findall(gcf,'-property','linewidth'),'linewidth',2)
%     ylim([-0.0036  0.5])
%     set(gca,'TickDir','out'); % The only other option is 'in'
%     
%     figure;
%     hold on;
%     plot(timers,n.so('goal').actLog(1:r,14),'Color','b','LineWidth',2)
%     plot(timers,n.so('goal').actLog(1:r,21),'Color',[  0.3922    0.8314    0.0745],'LineWidth',2)
%     plot(timers,n.so('goal').actLog(1:r,22),'Color','r','LineWidth',2)
%     plot(timers,n.so('goal').actLog(1:r,1:13),'LineWidth',0.5)
%     title(strcat('Goal Run Order B:',num2str(type_run)))
%     ss=xlim;
%     xlim([-4 ss(2)])
%     set(findall(gcf,'-property','FontSize'),'FontSize',13)
%     set(findall(gcf,'-property','FontName'),'FontName','arial')
%     set(findall(gcf,'-property','linewidth'),'linewidth',2)
%     
%     ylim([-0.0036  0.5])
% set(gca,'TickDir','out'); % The only other option is 'in'
% 
% end