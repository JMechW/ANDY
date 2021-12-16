classdef kCont < kNHVar
% Author: Jiamin Wang; Updated: 2021-12-15;

    methods(Access=public)
        function obj=kCont(inName,inSys,inDes)
            obj@kNHVar(inName,inSys,inDes,2);
            inSys.Continuous.reg(obj);
            obj.InitVal=sym([0;0]);
            obj.Expression=[];
        end
        
        function obj=setInitVal(obj,inInitVal)
            if(numel(inInitVal)~=obj.Order)
                error(obj.msgStr('Error','The kCoord must be 2nd Order Differentiable!'));
            else
                obj.InitVal=reshape(inInitVal,[],1);
            end
        end
        
        function obj=setExpr(obj,varargin)
            error(obj.msgStr('Error','Coordinates Don''t Have Expression!'));
        end
    end
end
