function [dq,ds,l]=FUNCNAME_(flow,t,q,p,u,s,l)
%% Dynamic flow switching map (Toolbox Internal Use)
%% 
%% Author: Jiamin Wang; Updated: 2021-12-15;

    dq=q*0;
    ds=s*0;
%SWITCHCASE_
