classdef kHVar < jObj & sym
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        System;
    end
    
    methods(Access=public)
        function obj=kHVar(inName,inSys,inDes)
            obj@jObj(inDes);
            obj@sym(inName,'real');
            
            if ~isempty(strfind(inName,'__'))
                error(obj.msgStr('Error','''__''is forbidden in Symbolic Variable Naming!'));
            end
            
            if(isa(inSys,'kSystem'))
                obj.System=inSys;
                obj.System.Model.setInit(false);
            else
                error(obj.msgStr('Error','InSys is not jSys Object!'));
            end
        end
    end
end
