classdef stateSpace < handle
    properties
        transitions
        curState
        transLog = []
        adjacencyMat = [];
        numStates = 0;
        timeInState = [];
        rewards = [];
        numRewards=0
    end
    
    methods
        function obj = stateSpace(transitions, curState, rewards)
            obj.transitions = transitions;
            obj.curState = curState;
            obj.numStates = numel(unique(obj.transitions(:,1)));
            obj.timeInState = zeros(1,obj.numStates);
            
            obj.adjacencyMat = zeros(obj.numStates);
            for t_i  = 1:size(obj.transitions,1)
                obj.adjacencyMat(obj.transitions(t_i,1),obj.transitions(t_i,2)) = 1;
            end
            
%             obj.transLog(end+1,:) = [curState,0];
            obj.transLog = [];
            if nargin > 2
               obj.rewards = rewards; %each row = [reward, state]
               obj.numRewards = numel(unique(obj.rewards(:,1)));
            end
            
        end
        
        function rewardVec = getReward(obj)
            rewardVec = zeros(1, obj.numRewards);
            currentState = find(obj.timeInState > 2); %only reward if been in state for two timesteps
            if isempty(currentState)
                currentState = 0;
            end
            rewardVec(obj.rewards(obj.rewards(:,2)==currentState,1))=1;
           
        end
        
        
        function obj = update(obj)
            global dt
            newTime = obj.timeInState(obj.curState) + dt;
            obj.timeInState = zeros(1,obj.numStates);
            obj.timeInState(obj.curState) = newTime;
        end
        
        
        function value = get_loc(obj)
            value = obj.curState;
        end
        
        function value = get_locVec(obj)
            value = zeros(1,obj.numStates);
            value(obj.curState) = 1;
        end
        
        function obj = set_loc(obj,location)
            obj.curState = location;
        end
        
        
        function value = get_numStates(obj)
            value = obj.numStates;
        end
        
        function value = get_numActions(obj)
            value = max(obj.transitions(:,3));
        end
        
        function value = get_numRewards(obj)
            value = obj.numRewards;
        end
        
        
        function moved = rwalk(obj)
            global t
            %randomly move
            possTrans = obj.transitions(obj.transitions(:,1) == obj.curState,:);
            newLoc = possTrans(randi(size(possTrans,1)),2);
            obj.set_loc(newLoc);
%             obj.transLog(end+1,:) = [newLoc,t];
            obj.log();
            moved=1;
        end
        
        function [moved, action] = rwalk2(obj, doMove)
            global t
            %randomly move
            possTrans = obj.transitions(obj.transitions(:,1) == obj.curState,:);
            moveResults = possTrans(randi(size(possTrans,1)),2:3);
            newLoc = moveResults(1);
            action = moveResults(2);
            
            if doMove
                obj.set_loc(newLoc);
                obj.get_loc();
%                 obj.transLog = [obj.transLog,[newLoc,t]];
                moved=1;
            else
                moved = 0;
            end
            
        end
        
        function moved = nwalk(obj,nextAct) %takes in the 'next' activities and moves to the max one if possible
            global t
            actThresh=.6;
            [val,maxNext] = max(nextAct);
            if val>actThresh
                tr = obj.transitions;
                if any(tr(tr(:,1) == obj.curState & tr(:,2) == maxNext))
                    obj.set_loc(maxNext);
                end
                obj.transLog(end+1,:) = [maxNext,t];
                moved=1;
            else
                moved=0;
            end
        end
        
        function [moved, action] = ewalk(obj) %makes transition that has been explored least
            global t
            
            possTrans = obj.transitions(obj.transitions(:,1) == obj.curState,:);
            possTransNums = obj.tr(possTrans(:, 1:2), 0); %Wut is TR?
            frequencies = horzcat(possTransNums,zeros(numel(possTransNums),1));
            logTransColumn = 5;
            tab = tabulate(obj.transLog(:, logTransColumn));
            for trans_i = possTransNums'
                transFreq = tab(tab(:, 1) == trans_i, 2);
                if ~isempty(transFreq)
                    frequencies(frequencies(:,1) == trans_i, 2) = transFreq;
                end
            end
            
            leastExploredTrans = frequencies(frequencies(:,2) == min(frequencies(:,2)), 1);
            r = rand(1, numel(leastExploredTrans));
            [~, max_i] = max(r);
            
            
            newTrans = leastExploredTrans(max_i);
            
            transLocs = obj.tr(newTrans,0);
            newLoc = transLocs(2);
                                  
            obj.set_loc(newLoc);      
            
            moved = true;
            action = possTrans(possTrans(:,2)==newLoc, 3);
            
            obj.log();
        end
        
        function moved = mwalk(obj,input)
            %move agent based on a motor command. The command (input) is
            %either a vector where one unit=1, in which case the index specifies
            %the action, or it is a scalar specifying the action directly
            global t
            
            if length(input)>1
                [val,action] = max(input); %TODO change to softmax
            else
                action = input;
                val = 1;
            end
            if val>.99 %Change to accord with stochastic... 
                transFromState_i = find(obj.transitions(:,1) == obj.curState);
                transWithAct_i = find(obj.transitions(:,3) == action);
                newState = obj.transitions(intersect(transFromState_i,transWithAct_i),2);
                if ~isempty(newState)
                    obj.set_loc(newState);
                    moved = 1;
%                     obj.transLog(end+1,:) = [newState,t];
                    obj.log();
                else
%                     warning('Cannot take that action here!')
                    moved = -1;
                end
                
            else
                moved = 0;
            end
        end
        
        function [moved, action, pathIndex] = fwalk(obj, path, curIndex, maxStuckTime, timeSinceMove,forceit)
            %force agent to walk a certain path
            if forceit==1
                nextState = path(curIndex + 1);
%                sourceTransitionsBool = obj.transitions(:, 1) == obj.curState;
%                targetTransitionsBool = obj.transitions(:, 2) == nextState;
%                desiredTransitionBool = sourceTransitionsBool & targetTransitionsBool;
% %                
%                if sum(desiredTransitionBool) ~= 1
%                    error('Transition improperly specified')
%                end
%                

               desiredTransition=obj.transitions(find(and(obj.transitions(:,1)==path(1),obj.transitions(:,2)==path(2))),:);
%                desiredTransition = obj.transitions(desiredTransitionBool,:);
               moved = 1;
               action = desiredTransition(3);
               newLoc = desiredTransition(2);
               obj.set_loc(newLoc); 
               pathIndex = curIndex + 1;
               obj.log();
               
        
            elseif (timeSinceMove > maxStuckTime) && (curIndex < length(path))
           
               nextState = path(curIndex + 1);
               sourceTransitionsBool = obj.transitions(:, 1) == obj.curState;
               targetTransitionsBool = obj.transitions(:, 2) == nextState;
               desiredTransitionBool = sourceTransitionsBool & targetTransitionsBool;
               
               if sum(desiredTransitionBool) ~= 1
                   error('Transition improperly specified')
               end
               
               desiredTransition = obj.transitions(desiredTransitionBool,:);
               moved = 1;
               action = desiredTransition(3);
               newLoc = desiredTransition(2);
               obj.set_loc(newLoc); 
               pathIndex = curIndex + 1;
               obj.log();
               
            else
                moved = 0;
                action = 0;
                pathIndex = curIndex;
                
                
            end
        end
            
        
            
            
            
        
        
        %function to tell you what state a numbered transition is from and
        %to, or vice versa; used for debugging only
        function result = tr(obj, inNums, disp)
            if nargin < 3
                disp = 1;
            end

            if size(inNums, 2) == 1 %given a transition number, return the states
                fromNum = ceil(inNums ./ obj.numStates);
                toNum = mod(inNums, obj.numStates);
                if toNum == 0
                    toNum = obj.numStates;
                end
                result = [fromNum, toNum];
                if disp
                    sprintf('From: %i To: %i\n', fromNum, toNum)
                end
            elseif size(inNums, 2) == 2
                transNum = (inNums(:,1) - 1) * obj.numStates + inNums(:,2);
                result = [transNum];
                if disp
                    sprintf('Transition: %i\n', transNum)
                end
            end               
        end
               
        
        function log(obj, cols, vals)
            %log columns; run, t, prevState, current state, transition
            %number, movetype
            global t dt
            if isempty(t)
                t = dt;
            end
%             obj.transLog(end+1,:) = [obj.curState, t];
            if size(obj.transLog,1) > 0                
                prevRun = obj.transLog(end, 1);
                prevT = obj.transLog(end, 2);
                prevState = obj.transLog(end, 4);
                moveType = NaN;
            else
                prevRun = 0;
                prevT = 0;
                prevState = 0;
                moveType = 0;
            end
            
                     
            if t ~= prevT           %log a new line
                if t < prevT
                    run = prevRun + 1;
                    prevState = 0;
                else
                    run = max(prevRun, 1);
                end

                
                transNum = max(obj.tr([prevState, obj.curState], 0), 0); %if starting state is "0" transNum will come back negative
               
                logLine = [run, t, prevState, obj.curState, transNum, moveType];
                obj.transLog(end+1,:) = logLine;
            else %modify previous line     
                if nargin > 1
                    if numel(cols) ~= numel(vals)
                        error('Wrong number of log values specified')
                    else                      
                        obj.transLog(end, cols) = vals;
                    end
                end
            end           
        end
        
        function reset(obj, startingState)
            if nargin < 2
                startingState = 1;
            end
            obj.set_loc(startingState);
            obj.log();
            obj.log(6,99);
%             obj.transLog(end+1,:) = [startingState, 0];
        end
        
        function [moved, action, descrip,descripNum] = emwalk(obj, motorVals, maxStuckTime, timeSinceMove, exploreChance)
            descripNum = 0;
            
            
            
        
            if timeSinceMove > maxStuckTime                
                [moved, action] = obj.ewalk();
                descrip = 'Stuck-explore';
                descripNum = 1;
            elseif any(motorVals > .999) && (rand() <= exploreChance)                
                [moved, action] = obj.rwalk2(1);
                descrip = 'Rand-explore';
                descripNum = 2;
            else                
                moved = obj.mwalk(motorVals);
                action = 0;
                if moved
                    descrip = 'Exploit';
                    descripNum = 3;
                else
                    descrip = 'Nothing';
                   
                end
             
            end
%             if descripNum ~= 0
%                 moveTypeCol = 6;
%                 obj.log(moveTypeCol, descripNum);
%             end
                
        end
        
        function weights = getRewardWeights(obj)
            %returns correct drives -> goals weights
           numDrives = numel(unique(obj.rewards(:,1)));
           weights = zeros(numDrives, obj.numStates);
           indices = sub2ind(size(weights), obj.rewards(:,1), obj.rewards(:,2));
           weights(indices) = 1;
        end
    end
end
        