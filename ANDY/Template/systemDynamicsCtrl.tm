function [TransDis,Quat,Vel,JacOut,MM,GFF,   JacDotOut,CorMat,CenMat,JacUTranspose,JacCons,JacConsDot,    qddt,sdt,ll]=FUNCNAME_(t,hStep,qdt0,qdt1,p,u,s,simFlag_)
%% Author: Jiamin Wang; Updated: 2021-12-15;


    qDim = numel(qdt0); uDim = numel(u);
    q = [qdt0;qdt1];
    sdt = NHSIGNAL_(t,q,p,u,s);

    [TF,Vel,Cor,Jac,TransDis,Quat]=KINEMATICS_(t,q,p,u,s);
    [MM,GFI,CenMat,CorMatLeft,~]=INERTIAL_(t,q,p,u,s,TF,Vel,Cor,Jac,true); %Turn the Full Flag to true
    [GFF]=FORCE_(t,q,p,u,s,TF,TransDis,Vel,Jac,Quat);
    [GFU,JacU]=INPUT_(t,q,p,u,s,TF,TransDis,Vel,Jac,Quat);
    [JacCons,CorCons,GFCons]=CONSTRAINT_(t,q,p,u,s,TransDis,Vel,Cor,Jac,Quat);

    MM = sum(MM,3); % Sum up inertia matrix of all bodies, this is needed for both cases below
    GFF = sum(GFF,2);

    JacSize = size(Jac);
    JacOut = zeros(JacSize(1)*JacSize(3),JacSize(2));
    JacDotOut = zeros(JacSize(1)*JacSize(3),JacSize(2));
    

    CONSCONTENT_
    consNum_ = numel(ConsContent_);
    consFlag_ = int8(1);
    ll = zeros(consNum_,1);
    
    if ConsContent_(1) == 0
        ConsContent_(1)=1;
        consFlag_ = int8(0);
    end
    JacCons=JacCons(ConsContent_,:);
    CorCons=CorCons(ConsContent_,1);
    GFCons=GFCons(ConsContent_,1);

    if consFlag_ == 0
        JacCons=JacCons*0;
        CorCons=CorCons*0;
        GFCons=GFCons*0;
    end

    % simFlag_ determines if the functioned is called for simulation or contoller design
    if simFlag_ > 0
        % if it is simulation, solve the constrained dynamics without outputing 
        % property matrices

        GFNonCons = GFF + sum(GFI,2) + sum(GFU,2);
        if consFlag_
            GaGa = JacCons*(MM\JacCons.');
            ll =  - GaGa\(JacCons*(MM\GFNonCons)+CorCons); %Lagrange Multipler
            GFTotal = GFNonCons + JacCons.'*(ll - GaGa\GFCons); % map the generalized force to the constrained manifold
            qddt = MM\GFTotal;
        else
            qddt = MM\GFNonCons;
        end
        
        JacDot = zeros(JacSize(1),JacSize(2),JacSize(3));
        JacConsSize = size(JacCons);
        JacConsDot = zeros(JacConsSize(1),JacConsSize(2));
        CorMat = zeros(qDim,qDim);
        CenSize = size(CenMat);
        CenMat = zeros(CenSize(1),CenSize(2));
        JacUTranspose = zeros(qDim,uDim);
    else
        % if it is for controller design, calculate property matrices
        qddt = zeros(qDim,1);

        JacUTranspose=sum(JacU,3); % Sum up constraint Jacobian Matrices of all inputs
        CenMat=sum(CenMat,3); % Sum up all centripetal matrices'
        
        % Calculate position changes after a short step, this is because jacobian 
        % matrices only contains q but not qdot
        tNext = t + hStep;
        qNext = [qdt0+qdt1*hStep;qdt1];
        sNext = s + sdt*hStep;

        [TFNext,VelNext,CorNext,JacNext,~,QuatNext]=KINEMATICS_(tNext,qNext,p,u,sNext);
        JacDot = (JacNext - Jac)/ hStep;

         %Turn the Full Flag to false; also, here the we simply use the inertial dynamics function as a selector of CorMatRightDot from JacDot 
        [~,~,~,~,CorMatRightDot]=INERTIAL_(tNext,qNext,p,u,sNext,TFNext,VelNext,CorNext,JacDot,false);
        CorMat = CorMatLeft*CorMatRightDot;

        [JacConsNext,~,~]=CONSTRAINT_(tNext,qNext,p,u,sNext,TransDis,VelNext,CorNext,JacNext,QuatNext);
        JacConsDot = (JacConsNext(ConsContent_,:) - JacCons) / hStep;

        if consFlag_ == 0
            JacConsDot = JacCons*0;
        end

        for iii_ = 1:JacSize(3)
            JacOut((6*(iii_-1)+1):6*(iii_),:) = Jac(:,:,iii_);
            JacDotOut((6*(iii_-1)+1):6*(iii_),:) = JacDot(:,:,iii_);

        end


    end

    


    

