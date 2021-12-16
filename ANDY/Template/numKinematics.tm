function [frameTF,frameVel,frameCorAcc,frameJacobian,frameTransDis,frameRotQuat]=FUNCNAME_(t,q,p,u,s)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    LinkTF=linkTF(t,q,p,u,s);
    LinkVel=linkVel(t,q,p,u,s);
    LinkCorAcc=linkCorAcc(t,q,p,u,s);
    LinkJacobian=linkJacobian(t,q,p,u,s);
    LinkRotQuat=linkRotQuat(t,q,p,u,s);

    FrameNum=FRAMENUM_;
    DOF=numel(q)/2;
    
    LinkTF=reshape((LinkTF),[4,4,FrameNum]);
    LinkJacobian=reshape((LinkJacobian),[6,DOF,FrameNum]);
    
    frameTF=zeros(4,4,FrameNum);
    frameTransDis=zeros(3,FrameNum);
    frameVel=zeros(6,FrameNum);
    frameCorAcc=zeros(6,FrameNum);
    frameJacobian=zeros(6,DOF,FrameNum);
    frameRotQuat=zeros(4,FrameNum);
    
    frameTF(:,:,1)=LinkTF(:,:,1);
    frameTransDis(:,1)=frameTF(1:3,4,1);
    frameRotQuat(:,1)=LinkRotQuat(:,1);
    frameVel(:,1)=LinkVel(:,1);
    frameCorAcc(:,1)=LinkCorAcc(:,1);
    frameJacobian(:,:,1)=LinkJacobian(:,:,1);

    frameCheckList=zeros(1,FrameNum);
    frameCheckList(1)=1;

    for frame=1:FrameNum
        framePath=0;

%SWITCHCASE_

        [frameCheckList,LinkTF,frameTF,LinkVel,frameVel,LinkCorAcc,frameCorAcc,LinkJacobian,frameJacobian,frameTransDis,LinkRotQuat,frameRotQuat]=frameKineRecur(framePath,frameCheckList,LinkTF,frameTF,LinkVel,frameVel,LinkCorAcc,frameCorAcc,LinkJacobian,frameJacobian,frameTransDis,LinkRotQuat,frameRotQuat);

    end

    function output_=localCrossFunc(aa_,bb_)
        output_=[aa_(2) * bb_(3) - aa_(3) * bb_(2); aa_(3) * bb_(1) - aa_(1) * bb_(3) ; aa_(1) * bb_(2) - aa_(2) * bb_(1) ];
    end

    function output_=skew3(w_)
        output_=[0 -w_(3) w_(2) ; w_(3) 0 -w_(1) ; -w_(2) w_(1) 0 ];
    end

    function p_ = quatMultiply(q_, r_)
        p_ = [q_(1).*r_(1) - q_(2).*r_(2) - q_(3).*r_(3) - q_(4).*r_(4);
             q_(1).*r_(2) + r_(1).*q_(2) + q_(3).*r_(4) - q_(4).*r_(3);
             q_(1).*r_(3) + r_(1).*q_(3) + q_(4).*r_(2) - q_(2).*r_(4);
             q_(1).*r_(4) + r_(1).*q_(4) + q_(2).*r_(3) - q_(3).*r_(2)];
    end

    function [frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_,RotQuat_,frameRotQuat_]=frameKineRecur(framePath_,frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_,RotQuat_,frameRotQuat_)
        curNum_=framePath_(end);
        if(~frameCheckList_(curNum_))
            preNum_=framePath_(end-1);
            if(frameCheckList_(preNum_))
                frameTF_(:,:,curNum_)=frameTF_(:,:,preNum_)*TF_(:,:,curNum_);
                frameTransDis_(:,curNum_)=frameTF_(1:3,4,curNum_);
                frameRotQuat_(:,curNum_)=quatMultiply(frameRotQuat_(:,preNum_),RotQuat_(:,curNum_));
                frameRotQuat_(:,curNum_)=frameRotQuat_(:,curNum_)./sqrt(frameRotQuat_(1,curNum_)^2 + ...
                                         frameRotQuat_(2,curNum_)^2 + frameRotQuat_(3,curNum_)^2 + frameRotQuat_(4,curNum_)^2);

                curTransVel=Vel_(1:3,curNum_);
                curAngVel=Vel_(4:6,curNum_);
                curCorTransAcc=CorAcc_(1:3,curNum_);
                curCorAngAcc=CorAcc_(4:6,curNum_);
                curTransJacobian=Jacobian_(1:3,:,curNum_);
                curAngJacobian=Jacobian_(4:6,:,curNum_);

                preTransVel=frameVel_(1:3,preNum_);
                preAngVel=frameVel_(4:6,preNum_);
                preCorTransAcc=frameCorAcc_(1:3,preNum_);
                preCorAngAcc=frameCorAcc_(4:6,preNum_);
                preTransJacobian=frameJacobian_(1:3,:,preNum_);
                preAngJacobian=frameJacobian_(4:6,:,preNum_);
                
                preRotMat=frameTF_(1:3,1:3,preNum_);
                preRotCurTransDis=preRotMat*TF_(1:3,4,curNum_);
                

                frameVel_(1:3,curNum_)=preTransVel+preRotMat*curTransVel+localCrossFunc(preAngVel,preRotCurTransDis);
                frameVel_(4:6,curNum_)=preAngVel+preRotMat*curAngVel;

                frameCorAcc_(1:3,curNum_)=preCorTransAcc+localCrossFunc(preAngVel,localCrossFunc(preAngVel,preRotCurTransDis))...
                                        +preRotMat*curCorTransAcc+localCrossFunc(preCorAngAcc,preRotCurTransDis)...
                                        +2*localCrossFunc(preAngVel,preRotMat*curTransVel);
                frameCorAcc_(4:6,curNum_)=preCorAngAcc+preRotMat*curCorAngAcc+localCrossFunc(preAngVel,preRotMat*curAngVel);

                %skewTransDis=[0 -preRotCurTransDis(3) preRotCurTransDis(2) ; preRotCurTransDis(3) 0 -preRotCurTransDis(1) ; -preRotCurTransDis(2) preRotCurTransDis(1) 0 ];
                skewTransDis=skew3(preRotCurTransDis);

                frameJacobian_(1:3,:,curNum_)=preTransJacobian+preRotMat*curTransJacobian-skewTransDis*preAngJacobian;
                frameJacobian_(4:6,:,curNum_)=preAngJacobian+preRotMat*curAngJacobian;

                frameCheckList_(curNum_)=1;
            else
                [frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_,RotQuat_,frameRotQuat_]=frameKineRecur(framePath_(1:end-1),frameCheckList_,TF_,frameTF_,Vel_,frameVel_,CorAcc_,frameCorAcc_,Jacobian_,frameJacobian_,frameTransDis_,RotQuat_,frameRotQuat_);
            end
        end
    end
