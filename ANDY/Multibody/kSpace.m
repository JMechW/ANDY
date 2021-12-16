classdef kSpace < jNet
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        System;
        TimeVar;
        RootFrame;
        
        DataSize;
        
        Directory;
        IsPrecompile=true;
    end
    
    methods(Access=public)
        
        function obj=kSpace(inName,inSys)
            obj@jNet(inName,'kFrame','kLink');
            obj.System=inSys;
            obj.TimeVar=inSys.TimeVar;
            obj.RootFrame=obj.genNode('WORLD').setRoot(true);
            obj.setDir(true); 
            obj.Directory=strcat(obj.System.tagStr('Kinematics'));
            if(~exist(obj.Directory))
                mkdir(obj.Directory);
            end
            cd(obj.Directory);
            obj.Directory=pwd;
            disp(obj.msgStr('Notification',strcat('System Kinematics will be saved in ./',(obj.System.tagStr('Kinematics')))));
            disp(obj.msgStr('Notification','Please put any .stl files related to visualization in the model'));
            cd('..')
        end
           
        function obj=setPrecompile(obj,inBool)
            if inBool>0
                obj.IsPrecompile=true;
            else
                obj.IsPrecompile=false;
            end
        end
            
        function obj=makeSymKinematics(obj)
            %Use this when calculating simple or sparse kinematics
            cd(obj.Directory);
            
            [t,c,q,p,s,u]=obj.System.getSymbolVectors();
            paramSym=c(:,1);
            paramVal=c(:,2);
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            if isempty(p)
                p=sym('DISCRETE__');
            end
            if isempty(u)
                u=sym('INPUT__');
            end
            if isempty(s)
                s=sym('NHSIGNAL__');
            else
                s=s(:,1);
            end
            
            frameList=obj.Node.get(obj.Node.Content);
            TFList=sym(zeros(4,4,numel(frameList)));
            VelList=sym(zeros(6,1,numel(frameList)));
            CorAccList=sym(zeros(6,1,numel(frameList)));
            JacobianList=sym(zeros(6,numel(q)/2,numel(frameList)));
            TFPath=cell(numel(frameList),2);
            
            TFBook=cell(numel(TFList(1,1,:)),1);
            VelBook=cell(numel(TFList(1,1,:)),1);
            CorBook=cell(numel(TFList(1,1,:)),1);
            JacBook=cell(numel(TFList(1,1,:)),1);
            for ii=1:numel(frameList)
                disp(obj.msgStr('Message',strcat('Generating Symbolic Kinematics of Frame: ',frameList{ii}.NameTag,'...')));
                if ~frameList{ii}.IsRoot
                    TFList(:,:,ii)=(frameList{ii}.rootTF());
                    VelList(:,:,ii)=([frameList{ii}.rootTransVel();frameList{ii}.rootAngVel()]);
                    CorAccList(:,:,ii)=([frameList{ii}.rootCorTransAcc();frameList{ii}.rootCorAngAcc()]);
                    JacobianList(:,:,ii)=([frameList{ii}.rootTransJacobian();frameList{ii}.rootAngJacobian()]);
                else
                    TFList(:,:,ii)=eye(4,4);
                    VelList(:,:,ii)=zeros(6,1);
                    CorAccList(:,:,ii)=zeros(6,1);
                    JacobianList(:,:,ii)=zeros(6,numel(q)/2);
                end
                
                matlabFunction(subs(TFList(:,:,ii),paramSym,paramVal),'File',strcat('TF_',num2str(ii),'.mx'),'Vars',{t,q,p,u,s});
                matlabFunction(subs(VelList(:,:,ii),paramSym,paramVal),'File',strcat('Vel_',num2str(ii),'.mx'),'Vars',{t,q,p,u,s});
                matlabFunction(subs(CorAccList(:,:,ii),paramSym,paramVal),'File',strcat('Cor_',num2str(ii),'.mx'),'Vars',{t,q,p,u,s});
                matlabFunction(subs(JacobianList(:,:,ii),paramSym,paramVal),'File',strcat('Jac_',num2str(ii),'.mx'),'Sparse',false,'Vars',{t,q,p,u,s});
                
                TFBook(ii)={fReadLine(strcat('TF_',num2str(ii),'.mx'))};
                VelBook(ii)={fReadLine(strcat('Vel_',num2str(ii),'.mx'))};
                CorBook(ii)={fReadLine(strcat('Cor_',num2str(ii),'.mx'))};
                JacBook(ii)={(fReadLine(strcat('Jac_',num2str(ii),'.mx')))};
                
                TFPath(ii,:)={frameList{ii}.NameTag,frameList{ii}.rootChain};
            end
            save(strcat(obj.System.tagStr('Path'),'.mat'),'TFPath');
            
            caseList=num2cell((1:numel(frameList)).');
            TFProcessList=cell(numel(caseList)+1,1);
            VelProcessList=cell(numel(caseList)+1,1);
            CorProcessList=cell(numel(caseList)+1,1);
            JacProcessList=cell(numel(caseList)+1,1);
            for ii=1:numel(TFList(1,1,:))
                TFProcessList{ii}={strcat('output=TF_',num2str(ii),'(t,q,p,u,s);')};
                VelProcessList{ii}={strcat('output=Vel_',num2str(ii),'(t,q,p,u,s);')};
                CorProcessList{ii}={strcat('output=Cor_',num2str(ii),'(t,q,p,u,s);')};
                JacProcessList{ii}={strcat('output=Jac_',num2str(ii),'(t,q,p,u,s);')};
            end
            TFProcessList{end}={strcat('output=zeros([',num2str(size(TFList(:,:,1))),']);')};
            VelProcessList{end}={strcat('output=zeros([',num2str(size(VelList(:,:,1))),']);')};
            CorProcessList{end}={strcat('output=zeros([',num2str(size(CorAccList(:,:,1))),']);')};
            JacProcessList{end}={strcat('output=zeros([',num2str(size(JacobianList(:,:,1))),']);')};
            TFSwitchCase=fGenSwitchList('frame',caseList,TFProcessList);
            VelSwitchCase=fGenSwitchList('frame',caseList,VelProcessList);
            CorSwitchCase=fGenSwitchList('frame',caseList,CorProcessList);
            JacSwitchCase=fGenSwitchList('frame',caseList,JacProcessList);
            
            TFMapName=obj.System.tagStr('TFMap');
            TFMapID=fopen(strcat(TFMapName,'.m'),'w');
            TFMapContent=fReadLine('frameMap.tm');
            TFMapContent=strrep(TFMapContent,'FUNCNAME_',TFMapName);
            TFMapSwitchPos=find(ismember(TFMapContent,'%SWITCHCASE_'));
            fWriteLine(TFMapID,TFMapContent(1:TFMapSwitchPos));
            fWriteLine(TFMapID,TFSwitchCase,2);
            fWriteLine(TFMapID,TFMapContent(TFMapSwitchPos+1:end));
            for ii=1:numel(TFBook)
                fWriteLine(TFMapID,[TFBook{ii};'end'],1);
            end
            fWriteLine(TFMapID,{'end'});
            fclose(TFMapID);
            
            VelMapName=obj.System.tagStr('VelMap');
            VelMapID=fopen(strcat(VelMapName,'.m'),'w');
            VelMapContent=fReadLine('frameMap.tm');
            VelMapContent=strrep(VelMapContent,'FUNCNAME_',VelMapName);
            VelMapSwitchPos=find(ismember(VelMapContent,'%SWITCHCASE_'));
            fWriteLine(VelMapID,VelMapContent(1:VelMapSwitchPos));
            fWriteLine(VelMapID,VelSwitchCase,2);
            fWriteLine(VelMapID,VelMapContent(VelMapSwitchPos+1:end));
            for ii=1:numel(VelBook)
                fWriteLine(VelMapID,[VelBook{ii};'end'],1);
            end
            fWriteLine(VelMapID,{'end'});
            fclose(VelMapID);
            
            CorMapName=obj.System.tagStr('CorMap');
            CorMapID=fopen(strcat(CorMapName,'.m'),'w');
            CorMapContent=fReadLine('frameMap.tm');
            CorMapContent=strrep(CorMapContent,'FUNCNAME_',CorMapName);
            CorMapSwitchPos=find(ismember(CorMapContent,'%SWITCHCASE_'));
            fWriteLine(CorMapID,CorMapContent(1:CorMapSwitchPos));
            fWriteLine(CorMapID,CorSwitchCase,2);
            fWriteLine(CorMapID,CorMapContent(CorMapSwitchPos+1:end));
            for ii=1:numel(CorBook)
                fWriteLine(CorMapID,[CorBook{ii};'end'],1);
            end
            fWriteLine(CorMapID,{'end'});
            fclose(CorMapID);
            
            JacMapName=obj.System.tagStr('JacMap');
            JacMapID=fopen(strcat(JacMapName,'.m'),'w');
            JacMapContent=fReadLine('frameMap.tm');
            JacMapContent=strrep(JacMapContent,'FUNCNAME_',JacMapName);
            JacMapSwitchPos=find(ismember(JacMapContent,'%SWITCHCASE_'));
            fWriteLine(CorMapID,JacMapContent(1:JacMapSwitchPos));
            fWriteLine(CorMapID,JacSwitchCase,2);
            fWriteLine(CorMapID,JacMapContent(JacMapSwitchPos+1:end));
            for ii=1:numel(JacBook)
                fWriteLine(JacMapID,[JacBook{ii};'end'],1);
            end
            fWriteLine(JacMapID,{'end'});
            fclose(JacMapID);
            
            TFFuncName=obj.System.tagStr('symTF');
            TFFuncID=fopen(strcat(TFFuncName,'.m'),'w');
            TFFuncContent=fReadLine('symTF.tm');
            TFFuncContent=strrep(TFFuncContent,'FUNCNAME_',TFFuncName);
            TFFuncContent=strrep(TFFuncContent,'FRAMENUM_',num2str(numel(frameList)));
            TFFuncContent=strrep(TFFuncContent,'TFMAP_',TFMapName);
            fWriteLine(TFFuncID,TFFuncContent);
            fWriteLine(TFFuncID,fReadLine(strcat(TFMapName,'.m')),1);
            fWriteLine(TFFuncID,{'end'});
            fclose(TFFuncID);
            
            KineFuncName=obj.System.tagStr('symKinematics');
            KineFuncID=fopen(strcat(KineFuncName,'.m'),'w');
            KineFuncContent=fReadLine('symKinematics.tm');
            KineFuncContent=strrep(KineFuncContent,'FUNCNAME_',KineFuncName);
            KineFuncContent=strrep(KineFuncContent,'FRAMENUM_',num2str(numel(frameList)));
            KineFuncContent=strrep(KineFuncContent,'TFMAP_',TFMapName);
            KineFuncContent=strrep(KineFuncContent,'VELMAP_',VelMapName);
            KineFuncContent=strrep(KineFuncContent,'CORMAP_',CorMapName);
            KineFuncContent=strrep(KineFuncContent,'JACMAP_',JacMapName);
            fWriteLine(KineFuncID,KineFuncContent);
            fWriteLine(KineFuncID,fReadLine(strcat(TFMapName,'.m')),1);
            fWriteLine(KineFuncID,fReadLine(strcat(VelMapName,'.m')),1);
            fWriteLine(KineFuncID,fReadLine(strcat(CorMapName,'.m')),1);
            fWriteLine(KineFuncID,fReadLine(strcat(JacMapName,'.m')),1);
            fWriteLine(KineFuncID,{'end'});
            fclose(KineFuncID);
                
            tfArg=(zeros(4,4,numel(frameList)));
            transDisArg=(zeros(3,numel(frameList)));
            velArg=(zeros(6,numel(frameList)));
            corArg=(zeros(6,numel(frameList)));
            jacobianArg=(zeros(6,numel(q)/2,numel(frameList)));
            
            obj.DataSize.TF=tfArg;
            obj.DataSize.TransDis=transDisArg;
            obj.DataSize.Vel=velArg;
            obj.DataSize.Cor=corArg;
            obj.DataSize.Jac=jacobianArg;
            
            if(obj.IsPrecompile)
                delete(gcp('nocreate'));
                frameArg=0;
                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);

%                 codegen(strcat(TFMapName,'.m'),'-args',{frameArg,tArg,qArg,pArg,uArg,sArg});
%                 codegen(strcat(VelMapName,'.m'),'-args',{frameArg,tArg,qArg,pArg,uArg,sArg});
%                 codegen(strcat(CorMapName,'.m'),'-args',{frameArg,tArg,qArg,pArg,uArg,sArg});
%                 codegen(strcat(JacMapName,'.m'),'-args',{frameArg,tArg,qArg,pArg,uArg,sArg});
                codegen(strcat(TFFuncName,'.m'),'-args',{tArg,qArg,pArg,uArg,sArg},'-jit');
                codegen(strcat(KineFuncName,'.m'),'-args',{tArg,qArg,pArg,uArg,sArg},'-jit');
            end
            
            delete('*.mx');
            cd('..');
        end
        
        function obj=makeNumKinematics(obj)
            %Use this when calculating high level serial kinematics
            cd(obj.Directory);
            
            [t,c,q,p,s,u]=obj.System.getSymbolVectors();
            paramSym=c(:,1);
            paramVal=c(:,2);
            if isempty(q)
                error(obj.msgStr('Error','System must possess continuous state!'))
            end
            if isempty(p)
                p=sym('DISCRETE');
            end
            if isempty(u)
                u=sym('INPUT');
            end
            if isempty(s)
                s=sym('NHSIGNAL');
            else
                s=s(:,1);
            end
            
            frameList=obj.Node.get(obj.Node.Content);
            TFList=sym(zeros(4,4,numel(frameList)));
            VelList=sym(zeros(6,1,numel(frameList)));
            RotQuatList=sym(zeros(4,1,numel(frameList)));
            CorAccList=sym(zeros(6,1,numel(frameList)));
            JacobianList=sym(zeros(6,numel(q)/2,numel(frameList)));
            TFPath=cell(numel(frameList),2);
            for ii=1:numel(frameList)
                disp(obj.msgStr('Message',strcat('Generating Numerical Kinematics of Frame: ',frameList{ii}.NameTag,'...')));
                if ~frameList{ii}.IsRoot
                    prevlink=frameList{ii}.EndEdge.get();
                    TFList(:,:,ii)=sym(prevlink{1}.FTF());
                    VelList(:,:,ii)=[prevlink{1}.TransVel;prevlink{1}.AngVel];
                    RotQuatList(:,:,ii)=[prevlink{1}.RotQuat];
                    CorAccList(:,:,ii)=[prevlink{1}.CorTransAcc;prevlink{1}.CorAngAcc];
                    JacobianList(:,:,ii)=[prevlink{1}.TransJacobian;prevlink{1}.AngJacobian];
                else
                    TFList(:,:,ii)=eye(4,4);
                    VelList(:,:,ii)=zeros(6,1);
                    RotQuatList(:,:,ii)=[1;0;0;0];
                    CorAccList(:,:,ii)=zeros(6,1);
                    JacobianList(:,:,ii)=zeros(6,numel(q)/2);
                end
                TFPath(ii,:)={frameList{ii}.NameTag,frameList{ii}.rootChain};
            end
            save(strcat(obj.System.tagStr('Path'),'.mat'),'TFPath');
            
            caseList=num2cell((1:numel(frameList)).');
            processList=cell(numel(caseList),1);
            for ii=1:numel(TFList(1,1,:))
                processList{ii}={strcat('framePath=[',num2str(TFPath{ii,2}),'];')};
            end
            SwitchCase=fGenSwitchList('frame',caseList,processList);
            
            VelList=reshape(VelList,6,numel(frameList));
            RotQuatList=reshape(RotQuatList,4,numel(frameList));
            CorAccList=reshape(CorAccList,6,numel(frameList));
            TFList=reshape(TFList,4,4*numel(frameList));
            JacobianList=reshape(JacobianList,6,numel(q)/2*numel(frameList));
            
            matlabFunction(subs(VelList,paramSym,paramVal),'File','linkVel.m','Vars',{t,q,p,u,s});
            matlabFunction(subs(RotQuatList,paramSym,paramVal),'File','linkRotQuat.m','Vars',{t,q,p,u,s});
            matlabFunction(subs(CorAccList,paramSym,paramVal),'File','linkCorAcc.m','Vars',{t,q,p,u,s});
            matlabFunction(subs(TFList,paramSym,paramVal),'File','linkTF.m','Sparse',false,'Vars',{t,q,p,u,s});
            matlabFunction(subs(JacobianList,paramSym,paramVal),'File','linkJacobian.m','Sparse',false,'Vars',{t,q,p,u,s});
            save(strcat(obj.System.tagStr('Path'),'.mat'),'TFPath');
            
            linkTFContent=(fReadLine('linkTF.m'));
            linkVelContent=(fReadLine('linkVel.m'));
            linkRotQuatContent=(fReadLine('linkRotQuat.m'));
            linkCorAccContent=(fReadLine('linkCorAcc.m'));
            linkJacobianContent=(fReadLine('linkJacobian.m'));
            
            frameFuncName=obj.System.tagStr('frameKinematics');
            frameFuncID=fopen(strcat(frameFuncName,'.m'),'w');
            frameFuncContent=fReadLine('frameKinematics.tm');
            frameFuncContent=strrep(frameFuncContent,'FUNCNAME_',frameFuncName);
            frameFuncContent=strrep(frameFuncContent,'FRAMENUM_',num2str(numel(frameList)));
            frameSwitchPos=find(ismember(frameFuncContent,'%SWITCHCASE_'));
            fWriteLine(frameFuncID,frameFuncContent(1:frameSwitchPos));
            fWriteLine(frameFuncID,SwitchCase,1);
            fWriteLine(frameFuncID,frameFuncContent(frameSwitchPos+1:end));
            fWriteLine(frameFuncID,{'end'});
            fclose(frameFuncID);  
            
            tfFuncName=obj.System.tagStr('numTF');
            tfFuncID=fopen(strcat(tfFuncName,'.m'),'w');
            tfFuncContent=fReadLine('numTF.tm');
            tfFuncContent=strrep(tfFuncContent,'FUNCNAME_',tfFuncName);
            tfFuncContent=strrep(tfFuncContent,'FRAMENUM_',num2str(numel(frameList)));
            tfSwitchPos=find(ismember(tfFuncContent,'%SWITCHCASE_'));
            fWriteLine(tfFuncID,tfFuncContent(1:tfSwitchPos));
            fWriteLine(tfFuncID,SwitchCase,2);
            fWriteLine(tfFuncID,tfFuncContent(tfSwitchPos+1:end));
            fWriteLine(tfFuncID,linkTFContent,1);
            fWriteLine(tfFuncID,{'end'},1);      
            fWriteLine(tfFuncID,{'end'});
            fclose(tfFuncID);  
            
            linkFuncName=obj.System.tagStr('linkKinematics');
            linkFuncID=fopen(strcat(linkFuncName,'.m'),'w');
            linkFuncContent=fReadLine('linkKinematics.tm');
            linkFuncContent=strrep(linkFuncContent,'FUNCNAME_',linkFuncName);
            linkFuncContent=strrep(linkFuncContent,'FRAMENUM_',num2str(numel(frameList)));
            fWriteLine(linkFuncID,linkFuncContent);   
            fWriteLine(linkFuncID,linkTFContent,1);      
            fWriteLine(linkFuncID,{'end'},1);
            fWriteLine(linkFuncID,linkVelContent,1);      
            fWriteLine(linkFuncID,{'end'},1);
            fWriteLine(linkFuncID,linkRotQuatContent,1);      
            fWriteLine(linkFuncID,{'end'},1);
            fWriteLine(linkFuncID,linkCorAccContent,1);      
            fWriteLine(linkFuncID,{'end'},1);
            fWriteLine(linkFuncID,linkJacobianContent,1);      
            fWriteLine(linkFuncID,{'end'},1);
            fWriteLine(linkFuncID,{'end'});
            fclose(linkFuncID);
            
            kineFuncName=obj.System.tagStr('numKinematics');
            kineFuncID=fopen(strcat(kineFuncName,'.m'),'w');
            kineFuncContent=fReadLine('numKinematics.tm');
            kineFuncContent=strrep(kineFuncContent,'FUNCNAME_',kineFuncName);
            kineFuncContent=strrep(kineFuncContent,'FRAMENUM_',num2str(numel(frameList)));
            kineSwitchPos=find(ismember(kineFuncContent,'%SWITCHCASE_'));  
            fWriteLine(kineFuncID,kineFuncContent(1:kineSwitchPos));
            fWriteLine(kineFuncID,SwitchCase,2);
            fWriteLine(kineFuncID,kineFuncContent(kineSwitchPos+1:end));   
            fWriteLine(kineFuncID,linkTFContent,1);      
            fWriteLine(kineFuncID,{'end'},1);
            fWriteLine(kineFuncID,linkVelContent,1);      
            fWriteLine(kineFuncID,{'end'},1);
            fWriteLine(linkFuncID,linkRotQuatContent,1);      
            fWriteLine(linkFuncID,{'end'},1);
            fWriteLine(kineFuncID,linkCorAccContent,1);      
            fWriteLine(kineFuncID,{'end'},1);
            fWriteLine(kineFuncID,linkJacobianContent,1);      
            fWriteLine(kineFuncID,{'end'},1);
            fWriteLine(kineFuncID,{'end'});
            fclose(kineFuncID);
            
            tfArg=(zeros(4,4,numel(frameList)));
            transDisArg=(zeros(3,numel(frameList)));
            velArg=(zeros(6,numel(frameList)));
            corArg=(zeros(6,numel(frameList)));
            rotQuatArg=(zeros(4,numel(frameList)));
            jacobianArg=(zeros(6,numel(q)/2,numel(frameList)));
            
            obj.DataSize.TF=tfArg;
            obj.DataSize.TransDis=transDisArg;
            obj.DataSize.Vel=velArg;
            obj.DataSize.Cor=corArg;
            obj.DataSize.rotQuat=rotQuatArg;
            obj.DataSize.Jac=jacobianArg;
            
            if(obj.IsPrecompile)
                delete(gcp('nocreate'));
                frameArg=0;
                tArg=0;
                qArg=zeros(numel(q),1);
                pArg=zeros(numel(p),1);
                uArg=zeros(numel(u),1);
                sArg=zeros(numel(s),1);

%                 codegen(strcat(frameFuncName,'.m'),'-args',{frameArg,tArg,qArg,pArg,uArg,sArg,tfArg,velArg,corArg,jacobianArg},'-jit');
%                 codegen(strcat(linkFuncName,'.m'),'-args',{tArg,qArg,pArg,uArg,sArg},'-jit');
                codegen(strcat(tfFuncName,'.m'),'-args',{tArg,qArg,pArg,uArg,sArg},'-jit');
                codegen(strcat(kineFuncName,'.m'),'-args',{tArg,qArg,pArg,uArg,sArg},'-jit');
            end
            
            cd('..');
        end
    end
        
    methods (Access=protected)
    end
end
