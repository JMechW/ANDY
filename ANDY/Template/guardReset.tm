function [act,flow,count,q,p,s]=FUNCNAME_(act,flow,count,t,q,p,u,s,l)
%% Author: Jiamin Wang; Updated: 2021-12-15;


    if(act)%If jump occured in previous jump function, return.
        return;
    end
    
    if((flow==FROMFLOW_)&&CONDITION_(t,q,p,u,s,l))%Condition to determine jump occurence.
        act=1;%Flag of jumpping
        flow=TOFLOW_;%The flow after jumping
        count=count+1;%Counter for Jumping occurence at the same moment
    else
        act=0;
        %count=0;
        return;
    end
    
    [TF,Vel,Cor,Jac,TransDis,Quat,MM,GFI,GFF,GFU,JacU,JacCons,CorCons,GFCons]=SYSTEM_(t,q,p,u,s);

    MM=sum(MM,3);
    GFI=sum(GFI,2);
    GFF=sum(GFF,2);
    GFU=sum(GFU,2);
    
    %Here are the Pre-Impact Mapping

    %Impact Mapping
    ConsContent=CONSNUM_;
    if ConsContent(1)~=0
        qSize=numel(q);
        qdot=q(qSize/2+1:qSize);
        partialJacCons=JacCons(ConsContent,:);
        q(qSize/2+1:qSize)=qdot-(MM\(partialJacCons.'*((partialJacCons*(MM\partialJacCons.'))\partialJacCons)))*qdot;
    end
%RESET_

    %Here are the Post-Impact Mapping
