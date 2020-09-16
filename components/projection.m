classdef projection < handle
    
% Projection object connecting layers and nodes to other layers
%     
% 
% Properties
%
% weights - matrix of weights from one layer to another (rows = source units, columns = targets)
% weightsTemp - temporary storage of weight values before update
% source - name of source object
% target - name of target object
% 
% timeTrace - array (same size as weights) of decay trace values
% traceDecay - parameter specifying how quickly trace decays
% 
% learn type - string specifying the learning law
% learnOn - boolean specifying whether learning is turned on
% norm - boolean specifying whether columns of weight matrix are normed before update
% 
% weightLog - 3d log of weights (3rd dimension = time)
% traceLog - 3d log of trace values (3rd dimension = time)
% 
% gates - Layers or nodes can act as gates which must be in a 
%         certain range for activity to pass.format: {{gateName,polarity},...}
% gateVals - current values of gates (1 = open, -1 = closed)
% 
% modulators - objects that serve as neuromodulators, like dopamine (used infrequently)
% modVals - current value of modulators
% 
% channels - cell array of objects controlling channels. Channels only allow 
%     input through units corresponding to the active units the controlling object.
%     E.g., if the controlling layer has only units 1 and 3 active then activity
%     flowing through projection will only reach target units 1 and 3. 


% Main Methods
% projection - class constructor. sets up source, target, weights, 
%     gates, and learning. Doesn't set up channels or modulators.
% 
% 
% update - function governing learning (activity transmission handled at the network level)
% 
% scaleWeights - scales weights by the specified factor (redundant now with weightFactor property?)
%     
% get_weightLog(obj,format,units, times) - gets weight log in either '2d' or '3d' 
%     format (in 3d format the weights are a 2d matrix, with the 3rd dimension 
%     as time. in 2d format, each weight is a row and the 2nd dimension is time .
%     Units specifies the units to get the data for, and times are the time 
%     boundaries within which data is reported
    
    properties
        name = ''
        weights = [];
        weightsTemp = [];
        source = '';
        target = '';    
        
        gates = {}; 
        gateVals = [];
        
        modulators = {}; %neuromodulators, like dopamine (used infrequently)
        modVals = []; %modulator values
        
        learnGate = {} %like gates above but for learning, not activity
        gateLog=[];
        
        learnOn = false;
        
        learnType = ''; %specifies weight update function
        learnRate = 0;
        norm = 0;
        
        weightLog = [];
        traceLog = [];
        sourceTraceLog = [];
        logOn = false
        
        timeTrace = [];  
        traceDecay = 1;
                
        weightFactor = 1; %weights intialized based on weightConfig .* weightFactor
        
        channels = {};
        learningParams = [];
    end
    
    methods
        function obj = projection(name,source,target,sourceSize,...
                targetSize,weightConfig,norm,weightFactor,learnType,learnRate,gates) 
            numReqArgs = 8;
            
            obj.name = name;            
            obj.source = source;                      
            obj.target = target;
            obj.norm = norm;
            obj.weightFactor = weightFactor;
            
            newWeights = zeros(sourceSize,targetSize);

            switch weightConfig
                case 'null'         
                case 'one-to-one'
                    newWeights = eye(sourceSize,targetSize);
                case 'impose'
                    newWeights = eye(sourceSize,targetSize);
                    newWeights(newWeights == 0) = -1.5;
                case 'imposeStrong'
                    newWeights = eye(sourceSize,targetSize) * 3;
                    newWeights(newWeights == 0) = -1.5;
                case 'imposeInhib'
                    newWeights = eye(sourceSize,targetSize);
                    newWeights(newWeights == 0) = -1.5;
                    newWeights(newWeights == 1) = 0; 
                case 'all-to-all'
                    newWeights = ones(sourceSize,targetSize);
                case 'unif'
                   newWeights = ones(sourceSize,targetSize) * .2;
                case 'zeros'
                   newWeights = zeros(sourceSize,targetSize);
                case 'const_inhib'
                   newWeights = (eye(sourceSize,targetSize) - 1);                             
                otherwise
                    error('Weight config error')
            end
            
            if obj.weightFactor == 0
                warning('Changing weight factor of 0 to 1')
                obj.weightFactor = 1;
            end
            newWeights = newWeights*obj.weightFactor;
            
            if obj.norm
                obj.weights = normc(newWeights);          
            else
                obj.weights = newWeights;
            end
            obj.weightsTemp = obj.weights;
            
            obj.timeTrace = zeros(1,size(obj.weights,2));
            obj.timeTrace = zeros(1,size(obj.weights,2));
                      
            if nargin>numReqArgs
                obj.learnType = learnType;
                obj.learnRate = learnRate;
                if ~strcmp(learnType,'null')
                    obj.learnOn = true;
                end
            end
            
           if nargin>numReqArgs+2
               
               %correct gates that only specify one open value instead of
               %a range; added 5/26/18, hopefully doesnt break anything
               for g_i = 1:numel(gates)
                   if numel(gates{g_i}{2}) == 1
                       val = gates{g_i}{2}(1);
                       gates{g_i}{2} = [val-.01, val+.01];
                   end
               end
               
               obj.gates = gates;
           end
           
        end
        
        function obj = update(obj,preAct,postAct,learnGate)
            %note that postactt might be modulatory activity from another projection instead of true postsynaptic activity.           
            global dt t ERROR 
            
            if nargin > 3
                doLearning=obj.checkLearnGate(learnGate);
            else
                doLearning=1;
            end                      
       
            switch obj.learnType
                case 'null'
                    obj.weightsTemp = obj.weights;
                                         
                    
                case 'adjacency'               
                                     
                    if numel(obj.learningParams) ~= 3
                        warning('Incorrect number of learning parameters specified, changing to  defaults')
                        obj.learningParams = [.6, .7, .6];
                    end
                    
                    traceThresh = obj.learningParams(1);    %level of activity where the trace rise to 1 instead of decaying away
                    traceLearnThresh = obj.learningParams(2);    %strength presynaptic trace needs to be for learning to occur
                    actLearnThresh = obj.learningParams(3);  %strength postsynaptic activity needs to be for learning to occur
                                                                 %adj weight corresponding to the current state should be higher 
                                                                 %than the rest; this threshold should discriminate
                     
                    decayAmts = (1 - obj.timeTrace) .* (preAct < traceThresh); %"inverse" exponential decay (slow at first, then rapid)
                    obj.timeTrace = obj.timeTrace + dt * (-decayAmts + (preAct >= traceThresh));
                    obj.timeTrace = max(min(obj.timeTrace,.99), 0);
                    

                    %Unit 1 is stimulated above traceThresh >> it's trace
                    %starts. Then when Unit 2 is excited
                    %above actLearnThresh, increase the weight from Unit 1
                    %to Unit 2 (weights encode relationships about the future)
                    
                    %weights can only increase
                    obj.weightsTemp = obj.weights+dt*(max((double(obj.timeTrace > traceLearnThresh)'*...
                        (postAct > actLearnThresh) - obj.weights),0) * obj.learnRate);               
                    
                    if (sum(bitxor((obj.timeTrace > traceLearnThresh), (postAct > actLearnThresh))) > 1) && any(postAct > actLearnThresh)
                        warning('Adjacency learning thinks agent was in more than one state at t = %f', t')
                    end
                    if sum(postAct > actLearnThresh) > 1
                        warning('adjacency learning thinks agent is in more than one state!');
                    end
    
                    
                case 'gradient'
                    % As above, traceThresh and actLearnThresh should
                    %discriminate between most active unit (the current
                    %state) and all others.
                    
                    if numel(obj.learningParams) ~= 3
                        warning('Incorrect number of learning parameters specified, changing to  defaults')
                        obj.learningParams = [.75, .7, .75];
                    end
                    traceThresh = obj.learningParams(1);                   
                    traceLearnThresh = obj.learningParams(2);
                    actLearnThresh = obj.learningParams(3);                   
                    
                    %"inverse" exponential decay (slow at first, then rapid)
                    decayAmts = (1 - obj.timeTrace).*(postAct < traceThresh);
                    obj.timeTrace = obj.timeTrace + dt*(-decayAmts) ;                     
                    obj.timeTrace = max(min(obj.timeTrace, .99), 0);
                    
                    %Unit 1 is stimulated above traceThresh >> it's trace
                    %starts and eventually exceeds traceLearnThersh. Then when Unit 2 is excited
                    %above actLearnThresh, increase the weight from Unit 2
                    %to unit 1 to match current value of the trace.
                    %(weights encode relationships about the past)
                    
                    
                    if doLearning
                        obj.timeTrace = obj.timeTrace + dt * ((preAct >= traceThresh) * 5);
                        obj.timeTrace = max(min(obj.timeTrace, .99), 0);

                        obj.weightsTemp = obj.weights + dt*(max(((preAct > actLearnThresh)' * double(obj.timeTrace > traceLearnThresh) - obj.weights),0) * obj.learnRate);
                          
                        otherTrace = obj.timeTrace;
                        [~, curState] = max(preAct);
                        otherTrace(curState) = 0;
                        
                        if (sum(preAct > actLearnThresh) > 1) && (sum(otherTrace > traceLearnThresh) > 0)
                           warning('Goal gradient learning thinks agent is in more than one state at t = %f', t)
                           ERROR = true;
                        end

                        if sum((obj.timeTrace > traceLearnThresh) - (preAct > traceThresh)) > 1
                            warning('Goal gradient learning thinks agent was in more than one state at t = %f', t')
                        end
                    else
                        obj.weightsTemp = obj.weights;
                    end
                    
                    
                    
                case 'oscHebb' 

                    %learning function for the weights mapping transitions
                    %to actions. Assumes an oscillatory learnGate. Only
                    %turns learning on if the learnGate is in the
                    %appropriate value range and, if applicable, the
                    %appropriate derivative sign. If it is, weights are
                    %binarized according to hand-tuned parameters and
                    %hebbian learning takes place. Weights are essentially
                    %all or nothing - gradations are not informative. There
                    %is surely a less clumsy way to do this, however.

                    if numel(obj.learningParams) ~= 2
                        warning('Incorrect number of learning parameters specified, changing to  defaults')
                        obj.learningParams = [.42, .15];
                    end

                    preActThresh = obj.learningParams(1);
                    postActThresh = obj.learningParams(2);

                    if doLearning                      
                        newPreAct = double(preAct > preActThresh);                                               
                        newPostAct = double(postAct > postActThresh);                       

%                         weightChange = ((newPreAct'*newPostAct))*obj.learnRate;                                             
%                         obj.weightsTemp = min(obj.weights+dt*weightChange, 1);

                                                              
                        obj.weightsTemp = obj.weights + dt * max(((newPreAct' * newPostAct) - obj.weights), 0) * obj.learnRate; 
                    else
                        obj.weightsTemp = obj.weights;
                    end
                                 
                case 'drives'
                    
                    if numel(obj.learningParams) ~= 2
                        warning('Incorrect number of learning parameters specified, changing to  defaults')
                        obj.learningParams = [-.4, .7];
                    end
                    
                    driveDtThresh = obj.learningParams(1); 
                    modLearnThresh = obj.learningParams(2);
                    
                    obj.timeTrace = obj.timeTrace + dt * (obj.modVals - obj.timeTrace);
                    

                    curStateVec = double(obj.modVals >= modLearnThresh);
                    curDriveVec = (obj.source.dadt < driveDtThresh) & (obj.source.act > .01);

                    obj.weightsTemp = obj.weights+dt*( max((curDriveVec'*curStateVec - obj.weights),0)*obj.learnRate);               
               
                otherwise
                    error(['No learning law named: ' obj.learnType])
            end
            
            if obj.norm
                %normalizes weights to a length of abs(obj.norm)
                %a negative norm value is just a code for normalizing the
                %rows (presynaptic) instead of the columns (postsynaptic)
                if obj.norm>0
                    obj.weightsTemp = max(normc(obj.weightsTemp),0)*obj.norm; 
                elseif obj.norm<0
                    obj.weightsTemp = max(normr(obj.weightsTemp),0)*-obj.norm;
                end
            end
                    
            
            if obj.logOn
                obj.weightLog(:,:,round(t/dt)) = obj.weights;
                obj.traceLog(round(t/dt),:) = obj.timeTrace;           
            end
                    
        end
        
        function gateOpen = checkLearnGate(obj, learnGateInfo)
            gateOpen = 1;

            curGate = learnGateInfo{1};
            gateCondition = learnGateInfo{2}; %should be (v1, v2, dt) where v1 and v2 are the bounds 
                                          %of activity within which gate is oepn. dt is -1 or 1, 
                                          %representing the necessary sign of the derivative
            gateVal = curGate.get_vals();
            gateDt = curGate.get_dt();
            v1 = min(gateCondition(1:2)); v2 = max(gateCondition(1:2));

            if ~(gateVal >= v1 && gateVal <= v2) %is oscillation in the appropriate phase?
                    gateOpen = 0;
            elseif length(gateCondition) == 3 %if in appropriate interval, check for dt condition
                if (gateDt*gateCondition(3))<0
                    gateOpen = 0;
                end
            end
        end
        
        function [openTimes, closeTimes] = getGateIntervals(obj)
            if isempty(obj.gateLog)
                warning('Gate values were not logged!')
            end
            indices=1:numel(obj.gateLog);
           
            shiftForward=horzcat(99,obj.gateLog(1:end-1));
            shiftBackward=horzcat(obj.gateLog(2:end),99);
            
            openTimes=indices(obj.gateLog==1 & shiftForward==0);
            closeTimes=indices(obj.gateLog==1 & shiftBackward==0);
        end;
          
        function obj = plotGates(obj)
            [openTimes, closeTimes] = getGateIntervals(obj);
            hold on
            openx=zeros(2,numel(openTimes));
            openy=zeros(2,numel(openTimes));
            openy(2,:)=1;
            for time_i = 1:numel(openTimes)
                openx(:,time_i) = [openTimes(time_i),openTimes(time_i),]';
            end
            line(openx,openy, 'Color', 'g', 'LineStyle', '--', 'LineWidth', 1);
            
            
            closex=zeros(2,numel(closeTimes));
            closey=zeros(2,numel(closeTimes));
            closey(2,:)=1;
            for time_i = 1:numel(closeTimes)
                closex(:,time_i) = [closeTimes(time_i),closeTimes(time_i),]';
            end
            line(closex,closey, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
          
                
        end
        
        function obj = flip(obj)
            obj.weights = obj.weightsTemp;
        end
        
        function obj = scaleWeights(obj,factor)
            obj.weights = obj.weights.*factor;
            obj.weightsTemp = obj.weights;
        end
        
        function obj = addMod(obj,modulator) %modulator should be a layer (not a name)
            obj.modulators{end+1} = modulator;
        end
        
        function obj = startLearning(obj)
            obj.learnOn = true;
        end
        
        function obj = stopLearning(obj)
            obj.learnOn = false;
        end
        
        function obj = startLogging(obj)
            %right now this assumes you want to log the whole session
            global endTime dt
            obj.logOn = true;
            obj.weightLog = zeros(size(obj.weights,1),size(obj.weights,2),endTime/dt);
            if ~isempty(obj.timeTrace)
                obj.traceLog = zeros(endTime/dt,size(obj.weights,2));
                obj.sourceTraceLog = zeros(endTime/dt,size(obj.weights,1));
            end
            
            if ~isempty(obj.learnGate)
                obj.gateLog=zeros(1,endTime/dt);
            end
        end
        
        function value = get_source(obj)
            value = obj.source;
        end
        
        function value = get_weights(obj)
            value = obj.weights;
        end
        
        function obj = set_weights(obj,weightMat)
            obj.weights = weightMat;
            obj.weightsTemp = obj.weights;
        end
        
        function obj = set_learnGate(obj, gateInfo)
            obj.learnGate = gateInfo;
        end
                   
        function values = get_weightLog(obj,format, times)
            global dt
            
            if nargin>2
                valuesTemp = obj.weightLog(:,:,round(times(1)/dt):round(times(2)/dt));
            else
                valuesTemp = obj.weightLog(:,:,:);
            end
          
            if nargin>1
                if strcmp(format,'2d')
                    valuesTemp = reshape(valuesTemp,numel(valuesTemp(:,:,1)),size(valuesTemp,3))';
                end
            else
                valuesTemp = obj.weightLog;
            end
            values = valuesTemp;
        end

        function value=get_traceLog(obj,times) %times=[startTime,stopTime]
            global dt
            if isempty(obj.traceLog)
                error('Trace log was not recorded')
            end
            if nargin>1
                value=obj.traceLog(round(times(1)/dt):round(times(2)/dt),:);
            else
                value=obj.traceLog;
            end
        end
            
        
  
    end  
    
end