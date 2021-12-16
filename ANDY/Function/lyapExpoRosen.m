function [lambda,MLE] = lyapExpoRosen(x,fs,imbedDim,delayStep,exRange,lineStyle,lineWidth)
% fs - is the sampling rate of the discrete time series
% imbedDim - is the embedding dimension, basically how many delayed states
% the system has.
% delayStep - is the time delay between each delay state, in most cases it
% should be uniform. delaystep can be determined by autocorrelation, at the
% point where the curve first go through the horizontal axis to the negative
% meanPeriod - is used to filter out point that are not considered on the
% near trajectory. Usually determined by the median of the magnitude spectrum.
% exRange - expansion range
    if isempty(lineStyle)
        lineStyle='r';
    end
    if isempty(lineWidth)
        lineWidth=2;
    end
    
    [pp,ff]=pspectrum(x,fs);
    meanFreq=sum(pp.*ff)/sum(pp);
    meanPeriod=(fs/meanFreq);
    
    
    [Y]=genDelaySeries(x,delayStep,imbedDim); %Phase space reconstruction
    YDim=size(Y);
    
    minDist=zeros(YDim(1),1);
    nearPos=zeros(YDim(1),1);
    
    for ii=1:YDim(1)
        x0=ones(YDim(1),1)*Y(ii,:);
        dist=sqrt(sum((Y-x0).^2,2));
        for jj=1:YDim(1)
            if abs(jj-ii)<=meanPeriod
                dist(jj)=1e10;
            end
        end
        [minDist(ii),nearPos(ii)]=min(dist);
    end
    
    lambda=zeros(numel(exRange(1):exRange(2)),1);
    
    for kk=exRange(1):exRange(2)
        evolve=0;
        vld=0;
        maxInd=YDim(1)-kk;
        for jj=1:maxInd
            if nearPos(jj)<=maxInd
                dist_k=sqrt(sum((Y(jj+kk,:)-Y(nearPos(jj)+kk,:)).^2,2));
                if (dist_k~=0)&&(minDist(jj)~=0)
                    evolve=evolve+(log(dist_k)-log(minDist(jj)))*fs;
                    vld=vld+1;
                end
            end
        end
        if vld>0
            lambda(kk-exRange(1)+1)=evolve/vld;
        end
    end
    
    plot((exRange(1):exRange(2))/fs,lambda,lineStyle,'LineWidth',2);
    MLE=polyfit(exRange(1):exRange(2),lambda.',1);
end