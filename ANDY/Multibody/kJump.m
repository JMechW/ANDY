classdef kJump < jEdge
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        RewriteWarning=false;
    end
    
    methods(Access=public)
        function obj=kJump(inName,inFlow1,inFlow2)
            obj@jEdge(inName,inFlow1,inFlow2);
        end
        
        function obj=setRewriteWarning(obj,inBool)
            obj.RewriteWarning=inBool;
        end
        
        function obj=makeGuardReset(obj,inGuard,inRemap)
            if(obj.RewriteWarning)
                ui=input(obj.msgStr('Warning','Making new jump file will erase existing jump file data, do you want to continue? (''Y'' for YES, anything else for NO) '),'s');
                if(~strcmp(ui,'Y'))
                    return;
                end
            end
            
            paramSym=obj.Net.Parameter(:,1);
            paramVal=obj.Net.Parameter(:,2);
            t=obj.Net.System.TimeVar;
            q=obj.Net.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Net.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Net.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.Net.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.Net.NHSignal(:,1);
            end
            l=obj.Net.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
            end
            
            fileDir=obj.Net.Directory;
            cd(fileDir);
            
            funcName=obj.jumpTagStr([]);
            funcid=fopen(strcat(funcName,'.m'),'w');
            tmpContent=fReadLine('guardReset.tm');
            tmpContent=strrep(tmpContent,'FUNCNAME_',obj.jumpTagStr([]));
            tmpContent=strrep(tmpContent,'FROMFLOW_',num2str(obj.EdgeNumber(1)));
            tmpContent=strrep(tmpContent,'TOFLOW_',num2str(obj.EdgeNumber(2)));
            tmpContent=strrep(tmpContent,'SYSTEM_',num2str(obj.Net.System.tagStr('System')));
            
            endNodeCons=obj.EndNode.CompileInfo.ConsContent;
            tmpContent=strrep(tmpContent,'CONSNUM_',strcat('[',num2str(endNodeCons),']'));
            
            condName=obj.jumpTagStr('Condition_');
            condExpr=subs(inGuard,paramSym,paramVal);
            tmpContent=strrep(tmpContent,'CONDITION_',condName);
            matlabFunction(condExpr,'File',strcat(condName,'.mx'),'Vars',{t,q,p,u,s,l});
            
            resetBook={};
            resetFunc={};
            if((isa(inRemap,'cell'))&&(~isempty(inRemap)))
                exprSize=size(inRemap);
                if(exprSize(2)==2)
                    for ii=1:exprSize(1)
                        realExpr=subs(inRemap{ii,2},paramSym,paramVal);
                        isinQ=find(ismember(q,inRemap{ii,1}));
                        isinP=find(ismember(p,inRemap{ii,1}));
                        isinS=find(ismember(s,inRemap{ii,1}));
                        varStr=[];
                        if ~isempty(isinQ)
                            varStr=strcat('q(',num2str(isinQ),')');
                        elseif ~isempty(isinP)
                            varStr=strcat('p(',num2str(isinP),')');
                        elseif ~isempty(isinS)
                            varStr=strcat('s(',num2str(isinS),')');
                        else
                            error(obj.msgStr('Error','Invalid Variable for Reset Map!'));
                        end
                        resetMapperName=obj.jumpTagStr(strcat('Remapper_',num2str(ii),'_'));
                        resetBook=[resetBook; ...
                                   strcat(varStr,'=',resetMapperName,'(t,q,p,u,s,l);');
                                   ];
                        matlabFunction(realExpr,'File',...
                                       strcat(resetMapperName,'.mx'),...
                                       'Vars',{t,q,p,u,s,l});
                        resetFunc=[resetFunc;{fReadLine(strcat(resetMapperName,'.mx'))}];
                    end
                else
                    error(obj.msgStr('Error','The Input should be a cell with format {<state> <sym>;...}!'));
                end
            end
            
            fWriteLine(funcid,tmpContent);
            fWriteLine(funcid,resetBook,1);
            fWriteLine(funcid,[fReadLine(strcat(condName,'.mx'));'end'],1);
            for ii=1:numel(resetFunc)
                fWriteLine(funcid,[resetFunc{ii};'end'],1);
            end
            fWriteLine(funcid,{'end'});
            fclose(funcid);
            
            delete('*.mx');
            cd('..');
        end
        
        function obj=editJump(obj)
            fileDir=obj.Net.Directory;
            cd(fileDir);
            funcName=obj.jumpTagStr([]);
            edit(strcat(funcName,'.m'));
            cd('..');
        end
        
        function output=jumpTagStr(obj,inStr)
            output=obj.Net.System.tagStr(strcat(inStr,'Jump_',num2str(obj.EdgeNumber(1)),'_',num2str(obj.EdgeNumber(2))));
        end
    end
end
