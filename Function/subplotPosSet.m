function vec=subplotPosSet(rowMax,colMax,inNum,rRatio,cRatio,rboarder,cboarder)
    row=floor((inNum-1)/colMax)+1;
    col=rem(inNum-1,colMax)+1;
    
    figH=(1-rboarder*2)/(rowMax*(1+2*rRatio)-1*rRatio);
    figW=(1-cboarder*2)/(colMax*(1+2*cRatio)-1*cRatio);
    
    vec=[figW*(col-1)+2*(col-1)*cRatio*figW+cRatio*figW+cboarder,figH*(rowMax-row)+2*(rowMax-row)*rRatio*figH+rRatio*figH+rboarder,figW,figH];
end