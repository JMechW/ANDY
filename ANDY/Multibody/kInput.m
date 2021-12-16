classdef kInput < kHVar
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kInput(inName,inSys,inDes)
            obj@kHVar(inName,inSys,inDes);
            inSys.Input.reg(inDes,obj);
        end
    end
end
