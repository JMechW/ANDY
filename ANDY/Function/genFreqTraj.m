function [trajOut0, trajOut1, trajOut2]=genFreqTraj(inOrder,inTime,inAmp,inFreq)

% Author: Jiamin Wang; Updated: 2021-12-15;


freqNum=numel(inFreq);
tSpan=reshape(inTime,1,[]);
dataNum=numel(tSpan);
freqArray=2*reshape(inFreq,[],1)*pi;

trajOut0=zeros(inOrder,dataNum);
trajOut1=zeros(inOrder,dataNum);
trajOut2=zeros(inOrder,dataNum);

    for ii=1:inOrder
        
        ampArray=inAmp.*(rand([freqNum,2])-0.5);
        
        traj0=(ampArray(:,1).')*([sin(freqArray.*tSpan)])+(ampArray(:,2).')*([cos(freqArray.*tSpan)]);
        traj1=(ampArray(:,1).')*(freqArray.*[cos(freqArray.*tSpan)])-(ampArray(:,2).')*(freqArray.*[sin(freqArray.*tSpan)]);
        traj2=-(ampArray(:,1).')*(freqArray.*freqArray.*[sin(freqArray.*tSpan)])-(ampArray(:,2).')*(freqArray.*freqArray.*[cos(freqArray.*tSpan)]);
        
        trajOut0(ii,:)=traj0;
        trajOut1(ii,:)=traj1;
        trajOut2(ii,:)=traj2;

    end

end
