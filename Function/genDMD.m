function Out=genDMD(XSet,YSet,rdDimRange,UVec,Sigma,VVec)
% A function that generates the matrix for DMD, Exact DMD, and Optimal DMD
% Refer to Sparsity-promoting dynamic mode decomposition by Jovanovic et.
% al.

    if any(size(XSet)~=size(YSet))
        error('X and Y should have the same size')
    end
    
    if isempty(UVec)||isempty(Sigma)||isempty(VVec)
        [UVec,Sigma,VVec]=svd(XSet,'econ');
    end
    TotalEng=trace(Sigma);
    
    if isempty(rdDimRange)||any(rdDimRange>rank(Sigma))||any(rdDimRange<=0)||(rank(rdDimRange)>1)
        rdDimRange=rank(Sigma);
        warning('No POD used.')
    end
    
    Out=cell(numel(rdDimRange),1);
    for ii=1:numel(rdDimRange)
        rdDim=rdDimRange(ii);
        
        Out1.UVec=UVec(:,1:rdDim);
        Out1.Sigma=Sigma(1:rdDim,1:rdDim);
        Out1.VVec=VVec(:,1:rdDim);
        Out1.EngRatio=trace(Out1.Sigma)/TotalEng;

        Out1.BB=(YSet*Out1.VVec)/(Out1.Sigma);
        Out1.KK=Out1.UVec'*Out1.BB;
        [Out1.KKEigVec,KKEigMat]=eig(Out1.KK);
        Out1.KKEigVal=diag(KKEigMat);
        Out1.Phi{1}=Out1.UVec*Out1.KKEigVec;
        Out1.Phi{2}=Out1.BB*Out1.KKEigVec;
        Out1.Phi{3}=Out1.UVec*Out1.KKEigVec;
        Out1.b0{1}=pinv(Out1.Phi{1})*YSet(:,end);
        Out1.b0{2}=pinv(Out1.Phi{2})*YSet(:,end);

        %% This Part Calculates Optimal DMD 1
        snapNum=size(XSet,2);
        VanderMat=Out1.KKEigVal.^[0:1:(snapNum-1)];

        LL=Out1.KKEigVec;
        RR=VanderMat;
        GG=Out1.Sigma*Out1.VVec';

        PP=(LL'*LL).*conj(RR*RR');
        qq=conj(diag(RR*GG'*LL));
        ss=trace(Out1.Sigma'*Out1.Sigma);

        PPchol=chol(PP,'lower');
        optAmp=diag((PPchol')\(PPchol\qq));

        Out1.b0{3}=optAmp*Out1.KKEigVal.^(snapNum);
        Out1.OptParam={PP qq ss};
        
        Out{ii}=Out1;
    end
end

