function [CollectGF,CollectInputJac]=FUNCNAME_(t,q,p,u,s,TF_Global,TransDis_Global,Vel_Global,Jac_Global,Quat_Global)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    CollectGF=zeros(numel(q)/2,1);
    CollectInputJac=zeros(numel(q)/2,numel(u));
    ForceNum=FORCENUM_;
    if ForceNum>0
        CollectGF=zeros(numel(q)/2,ForceNum);
        CollectInputJac=zeros(numel(q)/2,numel(u),ForceNum);
    end

    SubGF=zeros(numel(q)/2,1);
    SubInputJac=zeros(numel(q)/2,numel(u));

    for FCount=1:ForceNum

%SWITCHCASE_

        CollectGF(:,FCount)=SubGF;
        CollectInputJac(:,:,FCount)=SubInputJac;
    end
