classdef kDamper < kEffect
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kDamper(inName,inSystem,inDim,inPower)
            inBaseFrame=inSystem.Space.RootFrame;
            inDirFrame=inBaseFrame;
            
            obj@kEffect(inName,inSystem,inBaseFrame,inDirFrame);
            obj.setType(inDim,inPower);
            
            obj.System.Damper.reg(obj);
        end
        
        function obj=setProp(obj,inParameter,inVector)
            if(numel(inParameter)==1)
                inParameter=sym(inParameter*eye(obj.Dimension,obj.Dimension));
            elseif(numel(inParameter)==obj.Dimension)
                inParameter=sym(diag(inParameter));
            end
                        
            obj.setProperty(inVector,-inParameter,inVector);
        end
    end
end
