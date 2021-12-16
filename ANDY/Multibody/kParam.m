classdef kParam < kHVar
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        Value;
    end
    
    methods(Access=public)
        function obj=kParam(inName,inSys,inDes,varargin)
            if nargin==4
                inVal=varargin{1};
            else
                inVal=0;
            end
            obj@kHVar(inName,inSys,inDes);
            inSys.Parameter.reg(inDes,obj);
            
            if(isa(inVal,'double'))
                obj.Value=inVal;
            else
                error(obj.msgStr('Error','Value for Constant should be a double!'));
            end
        end
        
        function output=val(obj)
            output=obj.Value;
        end
    end
end
