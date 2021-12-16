function [frameTF]=FUNCNAME_(t,q,p,u,s)
%% Author: Jiamin Wang; Updated: 2021-12-15;

    TFcalc=linkTF(t,q,p,u,s);
    FrameNum=FRAMENUM_;

    TF=reshape(TFcalc,[4,4,FrameNum]);
    frameTF=zeros(4,4,FrameNum);

    frameTF(:,:,1)=TF(:,:,1);

    frameCheckList=zeros(1,FrameNum);
    frameCheckList(1)=1;

    for frame=2:FrameNum
        framePath=0;
        %thisTF=eye(4,4);
        
%SWITCHCASE_

        [frameCheckList,TF,frameTF]=frameTFRecur(framePath,frameCheckList,TF,frameTF);

        %{
        for jj=numel(framePath):-1:1
            curNum=framePath(jj);
            curTF=TF(:,:,curNum);

            thisTF=curTF*thisTF;
        end
        
        frameTF(:,:,frame)=thisTF;
        %}
    end

    function [frameCheckList_,TF_,frameTF_]=frameTFRecur(framePath_,frameCheckList_,TF_,frameTF_)
        curNum_=framePath_(end);
        if(~frameCheckList_(curNum_))
            preNum_=framePath_(end-1);
            if(frameCheckList_(preNum_))
                frameTF_(:,:,curNum_)=frameTF_(:,:,preNum_)*TF_(:,:,curNum_);
                frameCheckList_(curNum_)=1;
            else
                [frameCheckList_,TF_,frameTF_]=frameTFRecur(framePath_(1:end-1),frameCheckList_,TF_,frameTF_);
            end
        end
    end
