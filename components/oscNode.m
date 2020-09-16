%Oscillatory nodes are a subclass of nodes that have one value and
%oscillate at a frequency of 2pi * timeFactor. They can be reset by
%setting 

classdef oscNode < node
    properties
        offset=0
        timeFactor=1
        on=1;    
    end
    
    methods
        function obj = oscNode(name,timeFactor)
            obj=obj@node(name,1, 'osc',[]);
            obj.timeFactor=timeFactor;
        end
        
        function obj=oscUpdate(obj)
            global t
            if obj.on
                obj.vals=cos((t-obj.offset)*obj.timeFactor);
            else
                obj.vals=0;
            end
        end
        
        function obj=set_offset(obj,offset)
            obj.offset=offset;
        end
        
        function obj=turnOff(obj)
            obj.on=0;
        end
        
        function obj=turnOn(obj)
            global t
            obj.on=1;
            obj.offset=t;
        end
        
    end
end