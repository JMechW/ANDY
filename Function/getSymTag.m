function output=getSymTag(inSym,inChar)
% output=getSymTag(inSym,inChar)
% Get the tag information of <inChar> from a symbolic variable <inSym>.
% Note that the tag notion is "__$Tag$_$Info$_". A symbolic variable may
% have multiple tags.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    output=[];

    symChar=char(inSym);
    tagChar=strcat('__',inChar,'_');
    tagLength=length(tagChar);
    
    tagStart=strfind(symChar,tagChar);
    if(numel(tagStart)~=1)
        return;
    end
    tagEnd=tagStart+tagLength;
    
    tagContent=symChar(tagEnd:end);
    tagContentEnd=strfind(tagContent,'_');
    tagContentEnd=tagContentEnd(1)+numel(symChar)-numel(tagContent);
    
    output=symChar(tagEnd:tagContentEnd-1);
end