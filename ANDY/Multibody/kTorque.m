classdef kTorque < kEffect
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        Body;
        Value=zeros(3,1);
    end
    
    methods(Access=public)
        function obj=kTorque(inName,inBody,varargin)
            if nargin==3
                inBaseFrame=varargin{1};
                inDirFrame=inBaseFrame;
            elseif nargin==4
                inBaseFrame=varargin{1};
                inDirFrame=varargin{2};
            else
                inBaseFrame=inBody.BaseFrame;
                inDirFrame=inBaseFrame;
            end 
            
            obj@kEffect(inName,inBody.System,inBaseFrame,inDirFrame);
            obj.setType(3,1);
%             
%             obj.Parameter=obj.ReferenceFrame.rootRotMat;
%             obj.DirVector=obj.BaseFrame.rootAngVel;
            
            obj.Body=inBody;
            obj.Body.Torque.reg(obj);
        end
        
        function obj=setProp(obj,inVal)
            if(isequal(size(inVal),[3 1]))&&(isa(inVal,'sym'))
                obj.Value=inVal;
            else
                error(obj.msgStr('Error','The Torque Value need to be a 3*1 sym!'));
            end
            
            obj.setProperty(obj.BaseFrame.getAngSym(1),sym(eye(3,3)),inVal);
        end
    end
end
