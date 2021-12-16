classdef kPot < kEffect
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
    end
    
    methods(Access=public)
        function obj=kPot(inName,inSystem,inDim,inOrder)
            inBaseFrame=inSystem.Space.RootFrame;
            inDirFrame=inBaseFrame;
            
            obj@kEffect(inName,inSystem,inBaseFrame,inDirFrame);
            obj.setType(inDim,inOrder);
            
            obj.System.Potential.reg(obj);
        end
        
        function obj=setProp(obj,inParameter,inVector)
            dim=obj.Dimension;
            if(numel(inParameter)==1)
                inParameter=sym(inParameter*eye(dim,dim));
            elseif(numel(inParameter)==dim)
                inParameter=sym(diag(inParameter));
            end
            
            inDirVector=pDiff(inVector,obj.System.TimeVar);
            [NHSym,NHExpr]=obj.System.getNHSignalVector();
            NHdt=[];
            if ~isempty(NHSym)
                NHdt=pDiff(NHSym,obj.System.TimeVar);
            end
            inDirVector=jSimplify(subs(inDirVector,NHdt,NHExpr));
                        
            obj.setProperty(inDirVector,-inParameter,inVector);
        end
    end
end
