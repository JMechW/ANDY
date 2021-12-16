function [TF,Vel,CorAcc,Jacobian,RotQuat]=FUNCNAME_(t,q,p,u,s)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    TF=linkTF(t,q,p,u,s);
    Vel=linkVel(t,q,p,u,s);
    CorAcc=linkCorAcc(t,q,p,u,s);
    Jacobian=linkJacobian(t,q,p,u,s);
    RotQuat=linkRotQuat(t,q,p,u,s);

    FrameNum=FRAMENUM_;
    DOF=numel(q)/2;
    
    TF=reshape((TF),[4,4,FrameNum]);
    Jacobian=reshape((Jacobian),[6,DOF,FrameNum]);
