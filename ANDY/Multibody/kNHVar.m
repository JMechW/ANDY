classdef kNHVar < jObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        System;
        TimeVar;
        Sym;
        
        Order;
        InitVal=[];
        Expression=sym(0);
    end
    
    methods(Access=public)
        function obj=kNHVar(inName,inSys,inDes,inOrder)
            obj@jObj(inDes);
            
            if ~isempty(strfind(inName,'__'))
                error(obj.msgStr('Error','''__''is forbidden in Symbolic Variable Naming!'));
            end
            obj.Sym=sym(inName,'real');
            
            if(isa(inSys,'kSystem'))
                obj.System=inSys;
                obj.TimeVar=obj.System.TimeVar;
                obj.System.Model.setInit(false);
            else
                error(obj.msgStr('Error','Link for Coordinate Induction should be <kLink>!'));
            end
            
            if(inOrder>=1)
                obj.Order=floor(inOrder);
                obj.InitVal=zeros(obj.Order,1);
                obj.System.setOrder(inOrder);
            else
                error(obj.msgStr('Error','The Order of the System need to be a Positive Integer!'));
            end
        end
        
        function output=dot(obj,inOrd)
            if (obj.Order==0)
                if (inOrd==obj.Order)
                    output=sym(obj.Sym);
                    return;
                else
                    error(obj.msgStr('error','Time Derivative Order should be Zero!'));
                end
            end
            
            if(floor(inOrd)<0)
                error(obj.msgStr('Error','Time Derivative Order must be non-negative integer!'));
            elseif(inOrd>obj.Order)
                error(obj.msgStr('Error','Time Derivative Order must be no higher than Smoothness Order!'));
            else
                output=sym(strcat(char(obj.Sym),'__d',char(obj.TimeVar),'_',num2str(floor(inOrd)),'_'),'real');
            end
        end
        
        function output=val(obj,inOrd)
            if (obj.Order==0)
                if (inOrd==obj.Order)
                    output=sym(obj.Expression);
                    return;
                else
                    error(obj.msgStr('error','Time Derivative Order should be Zero!'));
                end
            else
                if(floor(inOrd)<0)
                    error(obj.msgStr('Error','Time Derivative Order must be non-negative integer!'));
                elseif(inOrd>obj.Order)
                    error(obj.msgStr('Error','Time Derivative Order must be no higher than Smoothness Order!'));
                elseif(inOrd==obj.Order)
                    output=sym(obj.Expression);
                else
                    output=obj.InitVal(floor(inOrd)+1);
                end
            end
        end
        
        function obj=setExpr(obj,inExpr)
            if(isa(inExpr,'sym'))
                obj.Expression=inExpr;
            else
                error(obj.msgStr('Error','The expression input should be <sym>!'));
            end
        end
        
        function obj=setInit(obj,inInit)
            if numel(inInit)==obj.Order
                obj.InitVal=reshape(sym(inInit),[],1);
            else
                error(obj.msgStr('Error','The Initial Value should be set according to the order'));
            end                
        end
    end
end
