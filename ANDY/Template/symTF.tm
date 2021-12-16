function [frameTF]=FUNCNAME_(t,q,p,u,s)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    FrameNum=FRAMENUM_;
    
    frameTF=zeros(4,4,FrameNum);
    
    for ii=1:FrameNum
        frameTF(:,:,ii)=TFMAP_(ii,t,q,p,u,s);
    end
