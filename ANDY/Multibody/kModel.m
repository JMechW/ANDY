classdef kModel < jNet
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        System;
        Directory;
        IsPrecompile=true;
        IsInitialized=false;
        
        Parameter;
        Continuous;
        Discrete;
        Input;
        NHSignal;
        
        BaseEOM;
        BaseCons;
        
        CompileInfo;
        
        CountLimit=100;
    end
    
    methods(Access=public)
        function obj=kModel(inName,inSys)
            obj@jNet(inName,'kFlow','kJump');
            obj.System=inSys;
            obj.setDir(true);
            obj.Directory=strcat(obj.System.tagStr('Dynamics'));
            if(~exist(obj.Directory))
                mkdir(obj.Directory);
            end
            cd(obj.Directory);
            obj.Directory=pwd;
            disp(obj.msgStr('Notification',strcat('System Dynamics will be saved in ./',(obj.System.tagStr('Dynamics')))));
            disp(obj.msgStr('Notification','Please save the <jump> function in this directory'));
            cd('..');
        end
        
        function obj=setInit(obj,inBool)
            obj.IsInitialized=inBool;
        end
        
        function obj=setCountLimit(obj,inNum)
            if(inNum<1)
                error(obj.msgStr('Error','Count Limit should be positive Integer!'));
            end
            obj.CountLimit=floor(abs(inNum));
        end
        
        function obj=setPrecompile(obj,inBool)
            if inBool>0
                obj.IsPrecompile=true;
            else
                obj.IsPrecompile=false;
            end
        end
        
        function obj=Init(obj,varargin)
            if nargin==2
                simpStyle=varargin{1};
            else
                simpStyle='Fast';
            end
            disp(obj.msgStr('Notification',strcat('Compiling Option:',simpStyle)));
            
            [param,val]=obj.System.getParamVector();
            obj.Parameter=[param,val];
            obj.Discrete=obj.System.getDiscVector();
            obj.Input=obj.System.getInputVector();
            obj.Continuous=[obj.System.getContVector(0);obj.System.getContVector(1)];
            [Sym,Val]=obj.System.getNHSignalVector();
            obj.NHSignal=[Sym,Val];
            
            obj.BaseEOM=obj.System.genBaseEOM(simpStyle); %Formally Comipling EOM Information
            obj.BaseCons=obj.System.genBaseCons(simpStyle); %Formally Comipling Constraint Information
            obj.CompileInfo=obj.System.genCompileInfo(); 
            
            cd(obj.Directory);
            BaseEOM=obj.BaseEOM;
            BaseCons=obj.BaseCons;
            CompileInfo=obj.CompileInfo;
            save(strcat(obj.System.tagStr('BaseEOM'),'.mat'),'BaseEOM');
            save(strcat(obj.System.tagStr('BaseCons'),'.mat'),'BaseCons');
            save(strcat(obj.System.tagStr('CompileInfo'),'.mat'),'CompileInfo');
            cd('..');
            
            obj.IsInitialized=true;
        end
        
        function obj=makeInputDynamics(obj,varargin)
            fileDir=obj.Directory;
            cd(fileDir);            
            
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.NHSignal(:,1);
            end
            
            Info=obj.CompileInfo.Input;
            Number=numel(Info.InvolvedFrame);
            
            JacBook=cell(Number,1);
            EffBook=cell(Number,1);
            InJacBook=cell(Number,1);
            FVJacBook=cell(Number,1);
            
            caseList=num2cell((1:Number).');
            processList=cell(Number,1);
            for ii=1:Number
                processList{ii}={strcat('SubFrame=[',num2str(Info.InvolvedFrame{ii}),'];');
                                 strcat('SubSubs=0;');
                                 strcat('if SubFrame(1)~=0');
                                 strcat('    SubSubs=[TransDis_Global(:,SubFrame);Vel_Global(:,SubFrame);Quat_Global(:,SubFrame)];');
                                 strcat('end');};
                if Info.ActionFrame{ii}==0
                    processList{ii}=[processList{ii};...
                                     strcat('InputFrameVecJac=','InputFVJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                     strcat('SubJac=','InputJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                     strcat('SubEff=','InputEff_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                     strcat('if SubFrame(1)~=0');
                                     strcat('    JacSubs=Jac_Global(:,:,SubFrame);');
                                     strcat('    for sfCount=1:numel(SubFrame)');
                                     strcat('        SubJac=SubJac+InputFrameVecJac(:,:,sfCount)*JacSubs(:,:,sfCount);');
                                     strcat('    end');
                                     strcat('end');];
                elseif Info.ActionFrame{ii}>0 %Force
                    processList{ii}=[processList{ii};...
                                     strcat('ActFrame=[',num2str(Info.ActionFrame{ii}),'];');
                                     strcat('RefFrame=[',num2str(Info.ReferenceFrame{ii}),'];');
                                     strcat('SubJac=','Jac_Global(1:3,:,ActFrame);');
                                     strcat('SubEff=','TF_Global(1:3,1:3,RefFrame)*','InputEff_',num2str(ii),'(t,q,p,u,s,SubSubs);');];
                else %Torque
                    processList{ii}=[processList{ii};...
                                     strcat('ActFrame=[',num2str(Info.ActionFrame{ii}),'];');
                                     strcat('RefFrame=[',num2str(Info.ReferenceFrame{ii}),'];');
                                     strcat('SubJac=','Jac_Global(4:6,:,-ActFrame);');
                                     strcat('SubEff=','TF_Global(1:3,1:3,RefFrame)*','InputEff_',num2str(ii),'(t,q,p,u,s,SubSubs);');];
                end
                processList{ii}=[processList{ii};...
                                 strcat('SubGF=','SubJac.''','*SubEff;');
                                 strcat('SubInputJac=','SubJac.''*InputEffJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                 ];
                             
                if Info.ActionFrame{ii}~=0
                    JacToMx=sym(0);
                    matlabFunction(subs(JacToMx,paramSym,paramVal),...
                               'File',strcat('ForceJac_',num2str(ii),'.mx'),...
                               'Sparse',false,'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                    JacBook(ii)={(fReadLine(strcat('ForceJac_',num2str(ii),'.mx')))};
                else
                    JacToMx=Info.Jacobian{ii};
                    matlabFunction(subs(JacToMx,paramSym,paramVal),...
                               'File',strcat('ForceJac_',num2str(ii),'.mx'),...
                               'Sparse',false,'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                    JacBook(ii)={(fReadLine(strcat('ForceJac_',num2str(ii),'.mx')))};
                end
                
                matlabFunction(subs(Info.Effect{ii},paramSym,paramVal),...
                               'File',strcat('InputEff_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                matlabFunction(subs(Info.InputJacobian{ii},paramSym,paramVal),...
                               'File',strcat('InputEffJac_',num2str(ii),'.mx'),...
                               'Sparse',false,'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                matlabFunction(subs(Info.FrameVecJacobian{ii},paramSym,paramVal),...
                               'File',strcat('InputFVJac_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                EffBook(ii)={fReadLine(strcat('InputEff_',num2str(ii),'.mx'))};
                InJacBook(ii)={(fReadLine(strcat('InputEffJac_',num2str(ii),'.mx')))};
                FVJacBook(ii)={fReadLine(strcat('InputFVJac_',num2str(ii),'.mx'))};
            end
            
            SwitchCase={};
            if ~isempty(caseList)
                SwitchCase=fGenSwitchList('FCount',caseList,processList);
            end
            
            forceFuncName=obj.System.tagStr('Input');
            forceFuncID=fopen(strcat(forceFuncName,'.m'),'w');
            forceFuncContent=fReadLine('inputDynamics.tm');
            forceFuncContent=strrep(forceFuncContent,'FUNCNAME_',forceFuncName);
            forceFuncContent=strrep(forceFuncContent,'FORCENUM_',num2str(Number));
            forceSwitchPos=find(ismember(forceFuncContent,'%SWITCHCASE_'));
            fWriteLine(forceFuncID,forceFuncContent(1:forceSwitchPos));
            if ~isempty(SwitchCase)
                fWriteLine(forceFuncID,SwitchCase,2);
            end
            fWriteLine(forceFuncID,forceFuncContent(forceSwitchPos+1:end));
            for ii=1:numel(JacBook)
                fWriteLine(forceFuncID,[JacBook{ii};'end'],1);
            end
            for ii=1:numel(EffBook)
                fWriteLine(forceFuncID,[EffBook{ii};'end'],1);
            end
            for ii=1:numel(InJacBook)
                fWriteLine(forceFuncID,[InJacBook{ii};'end'],1);
            end
            for ii=1:numel(FVJacBook)
                fWriteLine(forceFuncID,[FVJacBook{ii};'end'],1);
            end
            fWriteLine(forceFuncID,{'end'});
            fclose(forceFuncID);
            
            delete('*.mx');
            cd('..')
        end
        
        function obj=makeForceDynamics(obj,varargin)
            fileDir=obj.Directory;
            cd(fileDir);            
            
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.NHSignal(:,1);
            end
            
            Info=obj.CompileInfo.External;
            Number=numel(Info.InvolvedFrame);
            
            JacBook=cell(Number,1);
            EffBook=cell(Number,1);
            FVJacBook=cell(Number,1);
            
            caseList=num2cell((1:Number).');
            processList=cell(Number,1);
            for ii=1:Number
                processList{ii}={strcat('SubFrame=[',num2str(Info.InvolvedFrame{ii}),'];');
                                 strcat('SubSubs=0;');
                                 strcat('if SubFrame(1)~=0');
                                 strcat('    SubSubs=[TransDis_Global(:,SubFrame);Vel_Global(:,SubFrame);Quat_Global(:,SubFrame)];');
                                 strcat('end');};
                if Info.ActionFrame{ii}==0
                    processList{ii}=[processList{ii};...
                                     strcat('ForceFrameVecJac=','ForceFVJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                     strcat('SubJac=','ForceJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                     strcat('SubEff=','ForceEff_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                     strcat('if SubFrame(1)~=0');
                                     strcat('    JacSubs=Jac_Global(:,:,SubFrame);');
                                     strcat('    for sfCount=1:numel(SubFrame)');
                                     strcat('        SubJac=SubJac+ForceFrameVecJac(:,:,sfCount)*JacSubs(:,:,sfCount);');
                                     strcat('    end');
                                     strcat('end');];
                elseif Info.ActionFrame{ii}>0 %Force
                    processList{ii}=[processList{ii};...
                                     strcat('ActFrame=[',num2str(Info.ActionFrame{ii}),'];');
                                     strcat('RefFrame=[',num2str(Info.ReferenceFrame{ii}),'];');
                                     strcat('SubJac=','Jac_Global(1:3,:,ActFrame);');
                                     strcat('SubEff=','TF_Global(1:3,1:3,RefFrame)*','ForceEff_',num2str(ii),'(t,q,p,u,s,SubSubs);');];
                else %Torque
                    processList{ii}=[processList{ii};...
                                     strcat('ActFrame=[',num2str(Info.ActionFrame{ii}),'];');
                                     strcat('RefFrame=[',num2str(Info.ReferenceFrame{ii}),'];');
                                     strcat('SubJac=','Jac_Global(4:6,:,-ActFrame);');
                                     strcat('SubEff=','TF_Global(1:3,1:3,RefFrame)*','ForceEff_',num2str(ii),'(t,q,p,u,s,SubSubs);');];
                end
                processList{ii}=[processList{ii};...
                                 strcat('SubGF=','SubJac.''','*SubEff;');
                                 ];
                             
                if Info.ActionFrame{ii}~=0
                    JacToMx=sym(0);
                    matlabFunction(subs(JacToMx,paramSym,paramVal),...
                               'File',strcat('ForceJac_',num2str(ii),'.mx'),...
                               'Sparse',false,'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                    JacBook(ii)={(fReadLine(strcat('ForceJac_',num2str(ii),'.mx')))};
                else
                    JacToMx=Info.Jacobian{ii};
                    matlabFunction(subs(JacToMx,paramSym,paramVal),...
                               'File',strcat('ForceJac_',num2str(ii),'.mx'),...
                               'Sparse',false,'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                    JacBook(ii)={(fReadLine(strcat('ForceJac_',num2str(ii),'.mx')))};
                end
                             
                matlabFunction(subs(Info.Effect{ii},paramSym,paramVal),...
                               'File',strcat('ForceEff_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                matlabFunction(subs(Info.FrameVecJacobian{ii},paramSym,paramVal),...
                               'File',strcat('ForceFVJac_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                           
                EffBook(ii)={fReadLine(strcat('ForceEff_',num2str(ii),'.mx'))};
                FVJacBook(ii)={fReadLine(strcat('ForceFVJac_',num2str(ii),'.mx'))};
            end
            
            SwitchCase={};
            if ~isempty(caseList)
                SwitchCase=fGenSwitchList('FCount',caseList,processList);
            end
            
            forceFuncName=obj.System.tagStr('Force');
            forceFuncID=fopen(strcat(forceFuncName,'.m'),'w');
            forceFuncContent=fReadLine('forceDynamics.tm');
            forceFuncContent=strrep(forceFuncContent,'FUNCNAME_',forceFuncName);
            forceFuncContent=strrep(forceFuncContent,'FORCENUM_',num2str(Number));
            forceSwitchPos=find(ismember(forceFuncContent,'%SWITCHCASE_'));
            fWriteLine(forceFuncID,forceFuncContent(1:forceSwitchPos));
            if ~isempty(SwitchCase)
                fWriteLine(forceFuncID,SwitchCase,2);
            end
            fWriteLine(forceFuncID,forceFuncContent(forceSwitchPos+1:end));
            for ii=1:numel(JacBook)
                fWriteLine(forceFuncID,[JacBook{ii};'end'],1);
            end
            for ii=1:numel(EffBook)
                fWriteLine(forceFuncID,[EffBook{ii};'end'],1);
            end
            for ii=1:numel(FVJacBook)
                fWriteLine(forceFuncID,[FVJacBook{ii};'end'],1);
            end
            fWriteLine(forceFuncID,{'end'});
            fclose(forceFuncID);
            
            delete('*.mx');
            cd('..')
        end
        
        function obj=makeInertialDynamics(obj,varargin)
            fileDir=obj.Directory;
            cd(fileDir);            
            
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.NHSignal(:,1);
            end
            
            inertialInfo=obj.CompileInfo.Inertial;
            
            MassFuncName=obj.System.tagStr('Mass');
            MomentFuncName=obj.System.tagStr('Moment');
            matlabFunction(subs(inertialInfo.Mass,paramSym,paramVal),...
                           'File',strcat(MassFuncName,'.mx'),...
                           'Vars',{t,q,p,u,s});
            matlabFunction(subs(inertialInfo.Moment,paramSym,paramVal),...
                           'File',strcat(MomentFuncName,'.mx'),...
                           'Sparse',false,'Vars',{t,q,p,u,s});
            MassFuncContent=fReadLine(strcat(MassFuncName,'.mx'));
            MomentFuncContent=(fReadLine(strcat(MomentFuncName,'.mx')));
            
            InertFuncName=obj.System.tagStr('Inertial');
            InertFuncID=fopen(strcat(InertFuncName,'.m'),'w');
            InertFuncContent=fReadLine('inertialDynamics.tm');
            InertFuncContent=strrep(InertFuncContent,'FUNCNAME_',InertFuncName);
            InertFuncContent=strrep(InertFuncContent,'MASS__',MassFuncName);
            InertFuncContent=strrep(InertFuncContent,'MOMENT__',MomentFuncName);
            InertFuncContent=strrep(InertFuncContent,'BASEFRAME__',strcat('[',num2str(inertialInfo.BaseFrame),']'));
            fWriteLine(InertFuncID,InertFuncContent);
            fWriteLine(InertFuncID,[MassFuncContent;'end'],1);
            fWriteLine(InertFuncID,[MomentFuncContent;'end'],1);
            fWriteLine(InertFuncID,{'end'});
            fclose(InertFuncID);
            
            delete('*.mx');
            cd('..')
        end
        
        function obj=makeConstraintDynamics(obj,varargin)
            fileDir=obj.Directory;
            cd(fileDir);            
            
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.NHSignal(:,1);
            end
            
            Info=obj.CompileInfo.Constraint;
            Number=numel(Info.InvolvedFrame);
            
            JacBook=cell(Number,1);
            CorBook=cell(Number,1);
            GFBook=cell(Number,1);
            FVJacBook=cell(Number,1);
            
            caseList=num2cell((1:Number).');
            processList=cell(Number,1);
            for ii=1:Number
                processList{ii}={strcat('SubFrame=[',num2str(Info.InvolvedFrame{ii}),'];');
                                 strcat('SubSubs=0;');
                                 strcat('if SubFrame(1)~=0');
                                 strcat('    SubSubs=[TransDis_Global(:,SubFrame);Vel_Global(:,SubFrame);Quat_Global(:,SubFrame)];');
                                 strcat('end');
                                 strcat('ConsFrameVecJac=','ConsFVJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                 strcat('SubConsJac=','ConsJac_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                 strcat('SubConsCor=','ConsCor_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                 strcat('SubConsGF=','ConsGF_',num2str(ii),'(t,q,p,u,s,SubSubs);');
                                 strcat('if SubFrame(1)~=0');
                                 strcat('    JacSubs=Jac_Global(:,:,SubFrame);');
                                 strcat('    CorSubs=Cor_Global(:,SubFrame);');
                                 strcat('    for sfCount=1:numel(SubFrame)');
                                 strcat('        SubConsJac=SubConsJac+ConsFrameVecJac(sfCount,:)*JacSubs(:,:,sfCount);');
                                 strcat('        SubConsCor=SubConsCor+ConsFrameVecJac(sfCount,:)*CorSubs(:,sfCount);');
                                 strcat('    end');
                                 strcat('end');
                                 };
                matlabFunction(subs(Info.Jacobian{ii},paramSym,paramVal),...
                               'File',strcat('ConsJac_',num2str(ii),'.mx'),...
                               'Sparse',false,'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                matlabFunction(subs(Info.Coriolis{ii},paramSym,paramVal),...
                               'File',strcat('ConsCor_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                matlabFunction(subs(Info.GFFactor{ii},paramSym,paramVal),...
                               'File',strcat('ConsGF_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                matlabFunction(subs(Info.FrameVecJacobian{ii},paramSym,paramVal),...
                               'File',strcat('ConsFVJac_',num2str(ii),'.mx'),...
                               'Vars',{t,q,p,u,s,Info.SubsVec{ii}});
                JacBook(ii)={(fReadLine(strcat('ConsJac_',num2str(ii),'.mx')))};
                CorBook(ii)={fReadLine(strcat('ConsCor_',num2str(ii),'.mx'))};
                GFBook(ii)={fReadLine(strcat('ConsGF_',num2str(ii),'.mx'))};
                FVJacBook(ii)={fReadLine(strcat('ConsFVJac_',num2str(ii),'.mx'))};
            end
            
            SwitchCase={};
            if ~isempty(caseList)
                SwitchCase=fGenSwitchList('ConsCount',caseList,processList);
            end
            
            consFuncName=obj.System.tagStr('Constraint');
            consFuncID=fopen(strcat(consFuncName,'.m'),'w');
            consFuncContent=fReadLine('constraintDynamics.tm');
            consFuncContent=strrep(consFuncContent,'FUNCNAME_',consFuncName);
            consFuncContent=strrep(consFuncContent,'CONSNUM_',num2str(Number));
            consSwitchPos=find(ismember(consFuncContent,'%SWITCHCASE_'));
            fWriteLine(consFuncID,consFuncContent(1:consSwitchPos));
            if ~isempty(SwitchCase)
                fWriteLine(consFuncID,SwitchCase,2);
            end
            fWriteLine(consFuncID,consFuncContent(consSwitchPos+1:end));
            for ii=1:numel(JacBook)
                fWriteLine(consFuncID,[JacBook{ii};'end'],1);
            end
            for ii=1:numel(CorBook)
                fWriteLine(consFuncID,[CorBook{ii};'end'],1);
            end
            for ii=1:numel(GFBook)
                fWriteLine(consFuncID,[GFBook{ii};'end'],1);
            end
            for ii=1:numel(FVJacBook)
                fWriteLine(consFuncID,[FVJacBook{ii};'end'],1);
            end
            fWriteLine(consFuncID,{'end'});
            fclose(consFuncID);
            
            delete('*.mx');
            cd('..')
        end
        
        function obj=makeSystemDynamics(obj,varargin)
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.NHSignal(:,1);
            end
            
            kineType='sym';
            if nargin>1
            	kineType=varargin{1};
            end
            
            kineContent={};
            kineName={};
            if strcmp(kineType,'sym')||strcmp(kineType,'symbolic')
                kineName={obj.System.tagStr('symKinematics')};
                kineContent=fReadLine(strcat(obj.System.tagStr('symKinematics'),'.m'));
            elseif strcmp(kineType,'num')||strcmp(kineType,'numerical')
                kineName={obj.System.tagStr('numKinematics')};
                kineContent=fReadLine(strcat(obj.System.tagStr('numKinematics'),'.m'));
            else
                error(obj.msgStr('Error','Config Input Should be Either ''sym'' or ''num''!'))
            end
            
            obj.makeInertialDynamics;
            obj.makeForceDynamics;
            obj.makeInputDynamics;
            obj.makeConstraintDynamics;
            
            fileDir=obj.Directory;
            cd(fileDir);
            
            sysFuncName=obj.System.tagStr('System');
            sysFuncID=fopen(strcat(sysFuncName,'.m'),'w');
            sysFuncContent=fReadLine('systemDynamics.tm');
            sysFuncContent=strrep(sysFuncContent,'FUNCNAME_',sysFuncName);
            sysFuncContent=strrep(sysFuncContent,'KINEMATICS_',kineName);
            sysFuncContent=strrep(sysFuncContent,'INERTIAL_',obj.System.tagStr('Inertial'));
            sysFuncContent=strrep(sysFuncContent,'FORCE_',obj.System.tagStr('Force'));
            sysFuncContent=strrep(sysFuncContent,'INPUT_',obj.System.tagStr('Input'));
            sysFuncContent=strrep(sysFuncContent,'CONSTRAINT_',obj.System.tagStr('Constraint'));
            fWriteLine(sysFuncID,sysFuncContent);
            fWriteLine(sysFuncID,kineContent,1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Inertial'),'.m')),1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Force'),'.m')),1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Input'),'.m')),1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Constraint'),'.m')),1);
            fWriteLine(sysFuncID,{'end'});
            fclose(sysFuncID);
            
            if(obj.IsPrecompile)

                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);

                codegen(strcat(sysFuncName,'.m'),'-args',{tArg,qArg,pArg,uArg,sArg},'-jit');
                
            end
            cd('..');
        end
        
        %Added to make controller property calculations easier
        function obj=makeSystemDynamicsForControl(obj,flowNum,varargin)
            t=obj.System.TimeVar;
            q=obj.Continuous;
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
                sExpr=sym(0);
            else
                s=obj.NHSignal(:,1);
                sExpr=(subs(obj.NHSignal(:,2),paramSym,paramVal));
            end
            
            kineType='sym';
            if nargin>1
            	kineType=varargin{1};
            end
            
            
            kineContent={};
            kineName={};
            if strcmp(kineType,'sym')||strcmp(kineType,'symbolic')
                kineName={obj.System.tagStr('symKinematics')};
                kineContent=fReadLine(strcat(obj.System.tagStr('symKinematics'),'.m'));
            elseif strcmp(kineType,'num')||strcmp(kineType,'numerical')
                kineName={obj.System.tagStr('numKinematics')};
                kineContent=fReadLine(strcat(obj.System.tagStr('numKinematics'),'.m'));
            else
                error(obj.msgStr('Error','Config Input Should be Either ''sym'' or ''num''!'))
            end
            
%             obj.makeInertialDynamics;
%             obj.makeForceDynamics;
%             obj.makeInputDynamics;
%             obj.makeConstraintDynamics;
            
            fileDir=obj.Directory;
            cd(fileDir);

            %Notice that this part is added to generate controller
            %propertie
            nhName=obj.System.tagStr('NonholonomicSignal');
            matlabFunction(sExpr,'File',strcat(nhName,'.m'),'Vars',{t,q,p,u,s});
            

            flowList=obj.Node.get(obj.Node.Content);
            ConsContent={strcat('ConsContent_=[',num2str(flowList{flowNum}.CompileInfo.ConsContent),'];');
                                 };


            sysFuncName=strcat(obj.System.tagStr('ControlModel'),'_f',num2str(flowNum));
            sysFuncID=fopen(strcat(sysFuncName,'.m'),'w');
            sysFuncContent=fReadLine('systemDynamicsCtrl.tm');
            sysFuncContent=strrep(sysFuncContent,'FUNCNAME_',sysFuncName);
            sysFuncContent=strrep(sysFuncContent,'KINEMATICS_',kineName);
            sysFuncContent=strrep(sysFuncContent,'INERTIAL_',obj.System.tagStr('Inertial'));
            sysFuncContent=strrep(sysFuncContent,'FORCE_',obj.System.tagStr('Force'));
            sysFuncContent=strrep(sysFuncContent,'INPUT_',obj.System.tagStr('Input'));
            sysFuncContent=strrep(sysFuncContent,'CONSTRAINT_',obj.System.tagStr('Constraint'));
            sysFuncContent=strrep(sysFuncContent,'CONSCONTENT_',ConsContent);
            
            %Notice that this part is added to generate controller
            %propertie
            sysFuncContent=strrep(sysFuncContent,'NHSIGNAL_',nhName); 
            
            fWriteLine(sysFuncID,sysFuncContent);
            fWriteLine(sysFuncID,kineContent,1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Inertial'),'.m')),1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Force'),'.m')),1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Input'),'.m')),1);
            fWriteLine(sysFuncID,fReadLine(strcat(obj.System.tagStr('Constraint'),'.m')),1);
            
            %Notice that this part is added to generate controller
            %properties
            fWriteLine(sysFuncID,[fReadLine(strcat(obj.System.tagStr('NonholonomicSignal'),'.m'));'end'],1);
            
            
            fWriteLine(sysFuncID,{'end'});
            fclose(sysFuncID);
            
            if(obj.IsPrecompile)

                libdir = strcat("./",obj.System.tagStr('CtrlLib'),'_f',num2str(flowNum),'/');
                mkdir(libdir);
                
                tArg=0;
                hArg=0;
                qdt0Arg=zeros(numel(q)/2,1);
                qdt1Arg=zeros(numel(q)/2,1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);
                simFlagArg=int8(0);

                codegen(strcat(sysFuncName,'.m'),'-d',libdir,"-config:lib",'-args',{tArg,hArg,qdt0Arg,qdt1Arg,pArg,uArg,sArg,simFlagArg});
                codegen(strcat(sysFuncName,'.m'),'-args',{tArg,hArg,qdt0Arg,qdt1Arg,pArg,uArg,sArg,simFlagArg},'-jit');
                
            end
            cd('..');
        end
        
        
        function obj=makeFlowDynamics(obj)
            fileDir=obj.Directory;
            cd(fileDir);
            
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
                sExpr=sym(0);
            else
                s=obj.NHSignal(:,1);
                sExpr=(subs(obj.NHSignal(:,2),paramSym,paramVal));
            end
            l=obj.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
            end
            
            nhName=obj.System.tagStr('NonholonomicSignal');
            matlabFunction(sExpr,'File',strcat(nhName,'.mx'),'Vars',{t,q,p,u,s});
            
            flowList=obj.Node.get(obj.Node.Content);
            Number=numel(flowList);
            
            caseList=num2cell((1:Number).');
            processList=cell(Number,1);
            for ii=1:Number
                processList{ii}={strcat('ConsContent=[',num2str(flowList{ii}.CompileInfo.ConsContent),'];');
                                 };
            end
            
            SwitchCase={};
            if ~isempty(caseList)
                SwitchCase=fGenSwitchList('flow',caseList,processList);
            end
            
            flowFuncName=obj.System.tagStr('Flow');
            flowFuncID=fopen(strcat(flowFuncName,'.m'),'w');
            flowFuncContent=fReadLine('flow.tm');
            flowFuncContent=strrep(flowFuncContent,'FUNCNAME_',flowFuncName);
            flowFuncContent=strrep(flowFuncContent,'SYSTEM_',obj.System.tagStr('System'));
            flowFuncContent=strrep(flowFuncContent,'NHSIGNAL_',nhName);
            flowSwitchPos=find(ismember(flowFuncContent,'%SWITCHCASE_'));
            fWriteLine(flowFuncID,flowFuncContent(1:flowSwitchPos));
            if ~isempty(SwitchCase)
                fWriteLine(flowFuncID,SwitchCase,1);
            end
            fWriteLine(flowFuncID,flowFuncContent(flowSwitchPos+1:end));
            fWriteLine(flowFuncID,[fReadLine(strcat(obj.System.tagStr('NonholonomicSignal'),'.mx'));'end'],1);
            fWriteLine(flowFuncID,fReadLine(strcat(obj.System.tagStr('System'),'.m')),1);
            fWriteLine(flowFuncID,{'end'});
            fclose(flowFuncID);
            
            if(obj.IsPrecompile)

                fArg=0;
                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);
                lArg=zeros(numel(l),1);

                codegen(strcat(flowFuncName,'.m'),'-args',{fArg,tArg,qArg,pArg,uArg,sArg,lArg},'-jit');
            end
            
            delete('*.mx');
            cd('..');
        end
        
        function obj=makeJumpDynamics(obj)
            fileDir=obj.Directory;
            cd(fileDir);
            
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
            else
                s=obj.NHSignal(:,1);
            end
            l=obj.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
            end
            
            flowList=obj.Node.get(obj.Node.Content);
            caseList=num2cell((1:numel(obj.Node.Content)).');
            
            jumpProcessList=cell(numel(flowList),1);
            jumpBook={};
            for ii=1:numel(flowList)
                curFlow=flowList{ii};
                nodeJumpList=curFlow.StartEdge.get(curFlow.StartEdge.Content);
                if(isempty(nodeJumpList))
                    jumpProcessList{ii,1}={'%No Jump Here'};
                else
                    curJumpProcess=cell(numel(nodeJumpList),1);
                    for jj=1:numel(nodeJumpList)
                        curJump=nodeJumpList{jj};
                        curJumpProcess{jj,1}={strcat('[act,flow,count,q,p,s]=',curJump.jumpTagStr([]),'(act,flow,count,t,q,p,u,s,l);')};
                        jumpBook=[jumpBook;{fReadLine(strcat(curJump.jumpTagStr([]),'.m'))}];
                    end
                    jumpProcessList{ii,1}=curJumpProcess;
                end
            end
            SwitchCase=fGenSwitchList('flow',caseList,jumpProcessList);
            
            jumpFuncName=obj.System.tagStr('Jump');
            jumpFuncID=fopen(strcat(jumpFuncName,'.m'),'w');
            jumpFuncContent=fReadLine('jump.tm');
            jumpFuncContent=strrep(jumpFuncContent,'FUNCNAME_',jumpFuncName);
            jumpFuncContent=strrep(jumpFuncContent,'COUNTLIMIT_',num2str(obj.CountLimit));
            jumpSwitchPos=find(ismember(jumpFuncContent,'%SWITCHCASE_'));
            fWriteLine(jumpFuncID,jumpFuncContent(1:jumpSwitchPos));
            if ~isempty(SwitchCase)
                fWriteLine(jumpFuncID,SwitchCase,1);
            end
            fWriteLine(jumpFuncID,jumpFuncContent(jumpSwitchPos+1:end));
            fWriteLine(jumpFuncID,fReadLine(strcat(obj.System.tagStr('System'),'.m')),1);
            for ii=1:numel(jumpBook)    
                fWriteLine(jumpFuncID,[jumpBook{ii}],2);
            end
            fWriteLine(jumpFuncID,{'end'});
            fclose(jumpFuncID);
            
            if(obj.IsPrecompile)

                fArg=0;
                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);
                lArg=zeros(numel(l),1);
                
                codegen(strcat(jumpFuncName,'.m'),'-args',{fArg,tArg,qArg,pArg,uArg,sArg,lArg},'-jit');
            end
            
            cd('..');
            addpath(genpath(pwd));
        end
        
        function obj=makeDynamics(obj,varargin)
            type='sym';
            if(nargin>1)
                type=varargin{1};
            end
            obj.makeSystemDynamics(type);
            obj.makeFlowDynamics();
            obj.makeJumpDynamics();
            obj.makeModelInfo();
            
            toc;
        end
        
        function obj=makeModelInfo(obj)
            paramSym=obj.Parameter(:,1);
            paramVal=obj.Parameter(:,2);
            t=obj.System.TimeVar;
            q=obj.Continuous;
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            p=obj.Discrete;
            if isempty(p)
                p=sym('DISCRETE');
            end
            u=obj.Input;
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(obj.NHSignal)
                s=sym('NHSIGNAL');
                sExpr=sym(0);
            else
                s=obj.NHSignal(:,1);
                sExpr=(subs(obj.NHSignal(:,2),paramSym,paramVal));
            end
            l=obj.BaseCons.ConsVector;
            if isempty(l)
                l=sym('CONSSYM');
            end
            
            ModelInfo.Name=obj.System.NameTag;
            
            ModelInfo.Constants.Symbol=paramSym;
            ModelInfo.Constants.Value=paramVal;
            
            ModelInfo.Variables.Time=t;
            ModelInfo.Variables.Continuous=q;
            ModelInfo.Variables.Discrete=p;
            ModelInfo.Variables.Input=u;
            ModelInfo.Variables.Nonholonomic.Symbol=s;
            ModelInfo.Variables.Nonholonomic.Expression=sExpr;
            
            ModelInfo.Dynamics=obj.BaseEOM;
            ModelInfo.Dynamics.InputJacobian=jacobian(ModelInfo.Dynamics.InputForce,u);
            ModelInfo.Constraints=obj.BaseCons;
            
            flowList=obj.Node.get(obj.Node.Content);
            flowConstraint={};
            for ii=1:numel(flowList)
                flowConstraint=[flowConstraint;{flowList{ii}.CompileInfo.ConsContent}];
            end
            ModelInfo.Hybrid.Flow=flowConstraint;
            
            ModelInfo.CustomInfo=obj.System.CustomInfo;
            
            cd(obj.Directory);
            save(strcat(obj.System.tagStr('ModelInfo'),'.mat'),'ModelInfo');
            cd('..');
        end
    end
end
