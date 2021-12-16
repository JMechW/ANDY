function [flow,flowTraj,q,p,s]=FUNCNAME_(flow,t,q,p,u,s,l)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    act=0;
    count=0;
    flowTraj=zeros(COUNTLIMIT_+1,1);
    while(1)
        flowTraj(count+1)=flow;
        act=0;
%SWITCHCASE_
        if(act==0)
            break;
        end
        if(count>COUNTLIMIT_)
            flow=-flow;%Enter zeno or other infinite jump problem, quit and give an error.
            break;
        end
    end
