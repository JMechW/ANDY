function [modelState]=ekfStep(modelFunc,modelState)
    
    pp=modelState.pp;
    xx=modelState.xx;
    uu=modelState.uu;
    PPxx=modelState.PPxx;
    QQ=modelState.QQ;
    RR=modelState.RR;
    
    QQBase=modelState.QQBase;
    RRBase=modelState.RRBase;

    xxHat=modelFunc.xxPlus(pp,xx,uu);
    Fxx=modelFunc.Fxx(pp,xx,uu);
    Fvv=modelFunc.Fvv(pp,xx,uu);
    
    modelState.yyHat=modelFunc.yy(pp,xxHat,uu);
    Hxx=modelFunc.Hxx(pp,xxHat,uu);
    Hww=modelFunc.Hww(pp,xxHat,uu);
    
    PPxxHat=Fxx*PPxx*Fxx.'+Fvv*QQ*Fvv.'+QQBase;
    PPyy=Hxx*PPxxHat*Hxx.'+Hww*RR*Hww.'+RRBase;
    PPxy=PPxxHat*Hxx.';
    
    GG=PPxy/PPyy;
    xx=xxHat-GG*modelState.yyHat;
    modelState.xx=modelFunc.xxPost(pp,xx,uu);
    modelState.PPxx=PPxxHat-GG*PPyy*GG.';
    
end