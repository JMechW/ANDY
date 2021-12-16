function outContent=sparse2fullFuncFillNonZero(inContent)
% outContent=sparse2fullFunc(inContent)
% A function used to convert a function that outputs a single matrix from
% sparse matrix structure to normal matrix structure (since sparse matrix
% is not supported by codegen)

% Author: Jiamin Wang; Updated: 2021-12-15;


    replaceContent={
                    'out1(outSize(1),outSize(2))=0;';
                    'for eleNum=1:length(elementList)';
                    '    out1(elementRow(eleNum),elementCol(eleNum))=elementList(eleNum);';
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
                    strcat('outSize=',sizedata,';');
                    strcat('elementRow=',rowdata,';');
                    strcat('elementCol=',coldata,';');
                    strcat('elementList=',elementdata,';');
                    };...
                replaceContent];
    outContent=strrep(outContent,'sparse','');
end
