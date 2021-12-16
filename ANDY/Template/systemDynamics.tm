function [TF,Vel,Cor,Jac,TransDis,Quat,MM,GFI,GFF,GFU,JacU,JacCons,CorCons,GFCons,CenMat,CorMatLeft,CorMatRight]=FUNCNAME_(t,q,p,u,s)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    
    [TF,Vel,Cor,Jac,TransDis,Quat]=KINEMATICS_(t,q,p,u,s);
    [MM,GFI,CenMat,CorMatLeft,CorMatRight]=INERTIAL_(t,q,p,u,s,TF,Vel,Cor,Jac,true);
    [GFF]=FORCE_(t,q,p,u,s,TF,TransDis,Vel,Jac,Quat);
    [GFU,JacU]=INPUT_(t,q,p,u,s,TF,TransDis,Vel,Jac,Quat);
    [JacCons,CorCons,GFCons]=CONSTRAINT_(t,q,p,u,s,TransDis,Vel,Cor,Jac,Quat);
