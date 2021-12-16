function [angPos]=vecRotAng(vecPos)
    % Pitch-Roll Sequence YX
    % cos(Pitch)>0 cos(Roll)>0
    vecSize=size(vecPos);
    angPos=zeros(2,vecSize(2));
    angPos(2,:)=atan(-vecPos(2,:)./vecPos(3,:));
    angPos(1,:)=atan(vecPos(1,:)./(vecPos(3,:)./(cos(angPos(2)))));
end