function [frameTF,frameVel,frameCorAcc,frameJacobian]=FUNCNAME_(frame,t,q,p,u,s,TF,Vel,CorAcc,Jacobian)
%% Author: Jiamin Wang; Updated: 2021-12-15;
    
    DOF=numel(q)/2;
    
    FramePath=0;
    thisTF=eye(4,4);
    thisTransVel=zeros(3,1);
    thisCorTransAcc=zeros(3,1);
    thisTransJacobian=zeros(3,DOF);
    thisAngVel=zeros(3,1);
    thisCorAngAcc=zeros(3,1);
    thisAngJacobian=zeros(3,DOF);

%SWITCHCASE_

    for jj=numel(FramePath):-1:1
        curNum=FramePath(jj);
        curTF=TF(:,:,curNum);
        curTransVel=Vel(1:3,curNum);
        curAngVel=Vel(4:6,curNum);
        curCorTransAcc=CorAcc(1:3,curNum);
        curCorAngAcc=CorAcc(4:6,curNum);
        curTransJacobian=Jacobian(1:3,:,curNum);
        curAngJacobian=Jacobian(4:6,:,curNum);

        curRotMat=curTF(1:3,1:3);
        thisTransDis=curRotMat*thisTF(1:3,4);

        curRadVel=cross(curAngVel,thisTransDis);
        curRotAngVel=curRotMat*thisAngVel;
        thisRotTransVel=curRotMat*thisTransVel;

        thisCorTransAcc=curCorTransAcc...
                        +cross(curCorAngAcc,thisTransDis)...
                        +curRotMat*thisCorTransAcc...
                        +2*cross(curAngVel,thisRotTransVel)...
                        +cross(curAngVel,curRadVel);
        thisCorAngAcc=  curCorAngAcc...
                        +curRotMat*thisCorAngAcc...
                        +cross(curAngVel,curRotAngVel);

        thisTransVel=   curTransVel...
                        +thisRotTransVel...
                        +curRadVel;
        thisAngVel=curAngVel+curRotAngVel;

        thisTransJacobian=  curTransJacobian...
                            +curRotMat*thisTransJacobian...
                            -skew3(thisTransDis)*curAngJacobian;
        thisAngJacobian=curAngJacobian+curRotMat*thisAngJacobian;

        thisTF=curTF*thisTF;
    end

    frameTF=thisTF;
    frameVel=[thisTransVel;thisAngVel];
    frameCorAcc=[thisCorTransAcc;thisCorAngAcc];
    frameJacobian=[thisTransJacobian;thisAngJacobian];
    
    function output=skew3(w)
        output=[0 -w(3) w(2) ; w(3) 0 -w(1) ; -w(2) w(1) 0 ];
    end
