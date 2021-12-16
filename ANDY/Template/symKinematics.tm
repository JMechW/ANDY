function [frameTF,frameVel,frameCorAcc,frameJacobian,frameTransDis]=FUNCNAME_(t,q,p,u,s)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    FrameNum=FRAMENUM_;
    DOF=numel(q)/2;
    
    frameTF=zeros(4,4,FrameNum);
    frameTransDis=zeros(3,FrameNum);
    frameVel=zeros(6,FrameNum);
    frameCorAcc=zeros(6,FrameNum);
    frameJacobian=zeros(6,DOF,FrameNum);
    for ii=1:FrameNum
        frameTF(:,:,ii)=TFMAP_(ii,t,q,p,u,s);
        frameTransDis(:,ii)=frameTF(1:3,4,ii);
        frameVel(:,ii)=VELMAP_(ii,t,q,p,u,s);
        frameCorAcc(:,ii)=CORMAP_(ii,t,q,p,u,s);
        frameJacobian(:,:,ii)=JACMAP_(ii,t,q,p,u,s);
    end
