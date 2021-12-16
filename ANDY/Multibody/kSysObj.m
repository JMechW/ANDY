classdef kSysObj < jObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    %Element in the System which are established based on the Coordinate
    %Frames
    properties(SetAccess=protected)
        InvolvedFrame=[];
        
        System;
        TimeVar;
        
        BaseFrame;
        ReferenceFrame;
        
        CompileInfo;
    end
    
    methods(Access=public)
        function obj=kSysObj(inName,inSys,inBaseFrame,inRefFrame)
            obj@jObj(inName);
            if isa(inSys,'kSystem')
                obj.System=inSys;
                obj.TimeVar=obj.System.TimeVar;
                obj.System.Model.setInit(false);
            else
                error(obj.msgStr('Error','The Second Input Need to be kSys!'));
            end
            
            if isa(inBaseFrame,'kFrame')
                obj.BaseFrame=inBaseFrame;
            elseif ischar(inBaseFrame)
                obj.BaseFrame=obj.System.Space.Node.get(inBaseFrame);
            else
                error(obj.msgStr('Error','The Second Input Need to be <kFrame> or the NameTag of <kFrame>!'));
            end
            
            if isa(inRefFrame,'kFrame')
                obj.ReferenceFrame=inRefFrame;
            elseif ischar(inRefFrame)
                obj.ReferenceFrame=obj.System.Space.Node.get(inRefFrame);
            else
                error(obj.msgStr('Error','The Third Input Need to be <kFrame> or the NameTag of <kFrame>!'));
            end
        end
        
        function obj=setInvolvedFrame(obj,inVarList)
            for ii=1:numel(inVarList)
                curVar=char(inVarList(ii));
                curFrame=getSymTag(curVar,'FRAME');
                if ~isempty(curFrame)
                    obj.InvolvedFrame(end+1)=str2num(curFrame);
                end
            end
            obj.InvolvedFrame=unique(obj.InvolvedFrame);
        end
    end
end
