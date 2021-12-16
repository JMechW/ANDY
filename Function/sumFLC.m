function out1=sumFLC(inAng,inAmp)
% out1 is a column vector, inAmp have the same row number as out1, first
% half of inAmp corresponds to the sin(), second half corresponds to cos()
    inAng=reshape(inAng,1,[]);
    out1=sum(inAmp(:,1:end/2).*(sin(inAng))+inAmp(:,end/2+1:end).*(cos(inAng)),2);
    
end