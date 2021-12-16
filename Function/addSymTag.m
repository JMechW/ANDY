function out1=addSymTag(in1,inTag,inInfo)
    out1=sym(strcat(char(in1),'_',char(inTag),'_',char(inInfo),'_'),'real');
end