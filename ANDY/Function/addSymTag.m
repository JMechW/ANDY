function out1=addSymTag(in1,inTag,inInfo)
% Author: Jiamin Wang; Updated: 2021-12-15;


    out1=sym(strcat(char(in1),'_',char(inTag),'_',char(inInfo),'_'),'real');
end
