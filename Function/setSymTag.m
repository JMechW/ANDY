function output=setSymTag(inSym,inChar,inContent)
% output=setSymTag(inSym,inChar,inContent)
% Search for tag <inChar> and set tag infomation of <inContent> to a
% symbolic variable <inSym>.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    output=[];
    
    if isa(inContent,'sym')
        inContent=char(inContent);
    elseif isa(inContent,'double')
        inContent=num2str(inContent);
    end
    
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
    
    output=sym(strcat(symChar(1:tagEnd-1),inContent,symChar(tagContentEnd:end)));
end