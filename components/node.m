    
% Nodes are somewhat similar to layers in that they have "activity" values,
% have outputs, and can connect to layers via projections, but they are governed
% by arbitrary functions or code. They typically provide control or timing 
% signals 
%     
% 
% Properties
% vals - vector of values, analagous to activities
% valsLog - log of values with time on the first dimension
% dvdt - vector of value time derivatives
% dvdtLog - log of time derivatives with time on the first dimension
% logOn - boolean specifying whether values are being logged
% updateFunc - optional property specifying how vals evolve over time (alternatively vals can be hard-set by the external script)
% inputs - cell array of layers or nodes connecting to node (no weights)


% Main Methods
% update - update vals according to updateFunc using the sum of all inputs
% getValsLog(obj,times) - get the value log. times argument optional, is a
%   2-element vector specifying the beginning and end of the interval over
%   which you want data
%
%specUpdate function and related properties removed 4/12/17

classdef node < handle
    properties
        name
        vals
        valsLog=[]
        logOn
        updateFunc=@(x) x;
        inputs={};
        dvdt
        dvdtLog=[]
        pulseVals =[]
    end
    
    methods
        
        function obj=node(name,size,updateFunc,inputs)
            if ~strcmp(name(1:2),'n_')
                error('Node names must start with "n_"')
            end
            obj.name=name;
            obj.vals=zeros(1,size);
            obj.dvdt=zeros(1,size);
            if nargin>2
                obj.updateFunc=updateFunc;
                obj.inputs=inputs;
            end
       
        end
        
        
        function obj = pulse(obj, pulseParams)
            newVals = pulseParams{1};
            duration = pulseParams{2};
            obj.set_vals(newVals);
            obj.pulseVals = [duration, 0];
        end
            
        
        function obj=update(obj)
            global dt t
            if ischar(obj.updateFunc)
                if strcmp('osc', obj.updateFunc)
                    obj.oscUpdate();
                else
                    obj.vals=eval(obj.updateFunc);
                end
            elseif numel(obj.inputs)>0
                sumInput=zeros(size(obj.vals));
                for nodeInput=obj.inputs
                    nodeInput=nodeInput{1};
                    if strcmp(nodeInput.name(1:2),'n_')
                        sumInput=sumInput+nodeInput.get_vals();
                    else
                        sumInput=sumInput+nodeInput.get_act();
                    end
                end
                obj.vals=obj.updateFunc(obj.vals,sumInput);
            else                
                obj.vals=obj.updateFunc(obj.vals);
                              
            end
            
            if ~isempty(obj.pulseVals)
                %pulseVals = [unit, desired duration, duration so far]
                obj.pulseVals(2) = obj.pulseVals(2) + dt;
                if obj.pulseVals(2) >= obj.pulseVals(1)
                    obj.pulseVals = [];
                    obj.set_vals(0);
                end
            end

            if t>dt & obj.logOn
                if ndims(obj.vals)>1
                    lastVals=obj.valsLog(round((t-dt)/dt),:);
                else
                    lastVals=obj.valsLog(round((t-dt)/dt));
                end
                if strcmp('osc', obj.updateFunc) && (abs(lastVals-obj.vals)>.4)
                    obj.dvdt=0; %right when oscNode turns on (and it jumps from 0 to 1), dvdt=0
                else                
                    obj.dvdt=(obj.vals-lastVals)/dt;
                end
            else
                obj.dvdt=zeros(1,numel(obj.vals));
            end

            
            
            if obj.logOn       
                obj.valsLog(round(t/dt),:)=obj.vals;
                obj.dvdtLog(round(t/dt),:)=obj.dvdt;
            end
        end
        
        function obj=startLogging(obj)
            global endTime dt
            obj.logOn=true;
            obj.valsLog=zeros(endTime/dt,numel(obj.vals));
            obj.dvdtLog=zeros(endTime/dt,numel(obj.vals));
        end
        
        function obj=scaleVals(obj,factor)
            obj.vals=obj.vals.*factor;
        end
        
        function obj=set_vals(obj,newVals)
            if numel(newVals)~=numel(obj.vals)
                if numel(newVals)==1 && (newVals <= numel(obj.vals)) %if input is just a scalar, treat it as an index specifying which value is 1                 
                    valVec=zeros(1,numel(obj.vals));
                    
                    if newVals > 0
                        valVec(newVals)=1;
                    end
                    obj.vals=valVec;
                else
                    error('Node input wrong size')
                end
            else
                obj.vals=newVals;
            end
        end
                    
        function vals=get_vals(obj)
            vals=obj.vals;
        end
        
        function vals=get_dt(obj)
            vals=obj.dvdt;
        end
          
        function value=get_valsLog(obj,times)
            global dt
            if nargin>1
                value=obj.valsLog(round(times(1)/dt):round(times(2)/dt),:);
            else
                value=obj.valsLog;
            end
        end
        
        function value=get_dvdtLog(obj,times)
            global dt
            if nargin>1
                value=obj.dvdtLog(round(times(1)/dt):round(times(2)/dt),:);
            else
                value=obj.dvdtLog;
            end
        end
        
    end   
end
