function Content=fReadLine(fileName)
% Content=fReadLine(fileName)
% A wrapper function to read file text lines into a cell output from the
% name of the file <fileName> (make sure that the file exists in the
% current path!).

% Author: Jiamin Wang; Updated: 2021-12-15;


    Content={};
    id=fopen(fileName,'r');
    while(1)
        Content=cat(1,Content,fgetl(id));
        if(Content{end}==-1)
            Content=reshape({Content{1:end-1}},[],1); 
            break;
        end
    end
    fclose(id);
end
