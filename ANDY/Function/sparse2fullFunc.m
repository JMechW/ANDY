function outContent=sparse2fullFunc(inContent)
% outContent=sparse2fullFunc(inContent)
% A function used to convert a function that outputs a single matrix from
% sparse matrix structure to normal matrix structure (since sparse matrix
% is not supported by codegen)

% Author: Jiamin Wang; Updated: 2021-12-15;


    replaceContent={
                    'out1=zeros(outSize_(1),outSize_(2));';
                    'for eleNum_=1:length(elementList_)';
                    '    out1(elementRow_(eleNum_),elementCol_(eleNum_))=elementList_(eleNum_);';
                    'end';
                    };

    outputStatement=inContent{end};
    begin=strfind(outputStatement,'([');
    midpoint=strfind(outputStatement,'],');
    final=strfind(outputStatement,');');
    rowdata=outputStatement(begin+1:midpoint(1));
    coldata=outputStatement(midpoint(1)+2:midpoint(2));
    elementdata=outputStatement(midpoint(2)+2:midpoint(3));
    sizedata=strcat('[',outputStatement(midpoint(3)+2:final-1),']');
    outContent=[...
                inContent(1:end-1);
                {
                    strcat('outSize_=',sizedata,';');
                    strcat('elementRow_=',rowdata,';');
                    strcat('elementCol_=',coldata,';');
                    strcat('elementList_=',elementdata,';');
                    };...
                replaceContent];
    outContent=strrep(outContent,'sparse','');
end
