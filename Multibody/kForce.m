classdef kForce < kEffect
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        Body;
        Value=zeros(3,1);
    end
    
    methods(Access=public)
        function obj=kForce(inName,inBody,varargin)
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
            
%             obj.Parameter=obj.ReferenceFrame.rootRotMat;
%             obj.DirVector=obj.BaseFrame.rootTransVel;
            
            obj.Body=inBody;
            obj.Body.Force.reg(obj);
        end
        
        function obj=setProp(obj,inVal)
            if(isequal(size(inVal),[3 1]))&&(isa(inVal,'sym'))
                obj.Value=inVal;
            else
                error(obj.msgStr('Error','The Force Value need to be a 3*1 sym!'));
            end
            
%             obj.setProperty(obj.DirVector,obj.Parameter,inVal);
            obj.setProperty(obj.BaseFrame.getTransSym(1),sym(eye(3,3)),inVal);
        end
    end
end
