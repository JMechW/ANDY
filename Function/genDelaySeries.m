function [y] = genDelaySeries(x,delayStep,segNum)
    xSize=size(x); %require column dimension to be time dimension
    segDim=xSize(2)-(segNum-1)*delayStep;
    
    y=zeros(segDim,segNum*xSize(1));
    for ii=1:segNum
        xExtract=x(:,(0:(segDim-1))+(ii-1)*delayStep+1).';
        y(:,((ii-1)*xSize(1)+1):ii*xSize(1))=xExtract;
    end
end

