function [inertM,inertGF,inertCenMat,inertCorMatLeft,inertCorMatRight]=FUNCNAME_(t,q,p,u,s,TF_Global,Vel_Global,Cor_Global,Jac_Global,Full_Flag_)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    BaseFrameList=BASEFRAME__;
    MassList=MASS__(t,q,p,s,u);
    if Full_Flag_
        MomentList=reshape(MOMENT__(t,q,p,s,u),[3 3 numel(MassList)]);
    end
    
    inertCorMatLeft=zeros(numel(q)/2,6*numel(MassList));
    inertCorMatRight=zeros(6*numel(MassList),numel(q)/2);
    inertM=zeros(numel(q)/2,numel(q)/2,numel(MassList));
    inertGF=zeros(numel(q)/2,numel(MassList));
    inertCenMat=zeros(numel(q)/2,numel(q)/2,numel(MassList));


    for bodyNum=1:numel(MassList)
        if Full_Flag_
            rotor=TF_Global(1:3,1:3,BaseFrameList(bodyNum));

            m=MassList(bodyNum);
            I=rotor*MomentList(:,:,bodyNum)*rotor.';

            w=Vel_Global(4:6,BaseFrameList(bodyNum));
            a=Cor_Global(1:3,BaseFrameList(bodyNum));
            alpha=Cor_Global(4:6,BaseFrameList(bodyNum));
        end

        vJacobian=Jac_Global(1:3,:,BaseFrameList(bodyNum));
        wJacobian=Jac_Global(4:6,:,BaseFrameList(bodyNum));
        
        inertCorMatRight(6*bodyNum+(-5:0),:)=[vJacobian;wJacobian];

        if Full_Flag_
            % thisInertCorMatLeft=[m*vJacobian;I*wJacobian].';
            % thisInertCorMatRight=[vJacobian;wJacobian]; %Take the time derivative of this to calculate coriolis numerically
            inertCorMatLeft(:,6*bodyNum+(-5:0))=[m*vJacobian;I*wJacobian].';
            inertCenMat(:,:,bodyNum)=- wJacobian.'* skew3_InertDynamicsOnly_(I*w) * wJacobian; % On the same side of Mqddot
            % inertM(:,:,bodyNum)=thisInertCorMatLeft*thisInertCorMatRight;
            % inertGF(:,bodyNum)= - thisInertCorMatLeft*[a;alpha] - wJacobian.'* cross(w,I*w); % on the different side of Mqddot
            inertM(:,:,bodyNum)=vJacobian.'*m*vJacobian+wJacobian.'*I*wJacobian;
            inertGF(:,bodyNum)=- vJacobian.'*m*a - wJacobian.'*(I*alpha+cross(w,I*w)); % on the different side of Mqddot
        end
    end

    function output_=skew3_InertDynamicsOnly_(w_)
        output_=[0 -w_(3) w_(2) ; w_(3) 0 -w_(1) ; -w_(2) w_(1) 0 ];
    end
