function matlab2Julia(funcList,finalName,filePath,isSparse)
% Only support 1D array inputs and single output
    
    funcNum=numel(funcList)/3;
    funcBook=cell(funcNum,1);
    
    for ii=1:funcNum
%         filename=strcat(filePath,funcList{ii,1},"_",finalName,".mx");
        filename=strcat(filePath,funcList{ii,1},".mx");
        inputArg=funcList{ii,3};
        inArgNum=numel(inputArg);
        
        matlabFunction(funcList{ii,2},"File",filename,"Sparse",true,"Vars",inputArg);
        funcBook(ii)={sparse2fullFunc(fReadLine(filename))};
        funcBook(ii)={strrep(funcBook{ii},",:)","]")};
        for jj=1:inArgNum
            inputStringName=strcat("in",num2str(jj));
            funcBook(ii)={strrep(funcBook{ii},strcat(inputStringName,"("),strcat(inputStringName,"["))};
        end
        funcBook(ii)={strrep(funcBook{ii},".*","*")};
        funcBook(ii)={strrep(funcBook{ii},".^","^")};
        funcBook(ii)={strrep(funcBook{ii},"./","/")};
        funcBook(ii)={strrep(funcBook{ii},"outSize(1)","outSize[1]")};
        funcBook(ii)={strrep(funcBook{ii},"outSize(2)","outSize[2]")};
        funcBook(ii)={strrep(funcBook{ii},"out1(","out1[")};
        funcBook(ii)={strrep(funcBook{ii},"(eleNum)","[eleNum]")};
        funcBook(ii)={strrep(funcBook{ii},"eleNum])","eleNum]]")};
        funcBook(ii)={strrep(funcBook{ii},"%","#")};
        
        funcBook{ii}(1)=strrep(funcBook{ii}(1),"out1 = ","");
        
        funcBook{ii}(1)=strrep(funcBook{ii}(1),",","::vType,");
        funcBook{ii}(1)=strrep(funcBook{ii}(1),")","::vType) where vType<:fVecUnion{fType} where fType");

        funcBook(ii)={[funcBook{ii};"return out1;";"end"]};
    end

    finalID=fopen(strcat(filePath,finalName,'.jl'),'w');
    fWriteLine(finalID,["__precompile__();";...
                        "";...
                        strcat("module ",finalName)]);
    fWriteLine(finalID,strcat("#functions were converted from matlab!"));
    fWriteLine(finalID,[...
                        "using LinearAlgebra;";"using SparseArrays;";"const fTypeUnion=Union{Float32,Float64};";...
                        "const fVecUnion=Union{Array{fType,1},Array{fType,2},SparseMatrixCSC{fType,Int64},SparseVector{fType,Int64}} where fType<:fTypeUnion;";]);
    
                    
    for ii=1:funcNum
        fWriteLine(finalID,funcBook{ii}(1));
        fWriteLine(finalID,funcBook{ii}(2:end-6),1);
        if isSparse
            fWriteLine(finalID,[...
                                "out1=convert(SparseMatrixCSC{fType,Int64},sparse([elementRow;outSize[1]],[elementCol;outSize[2]],[elementList;0]));"],1)
        else
            fWriteLine(finalID,["elementList=convert(Array{fType,1},elementList);"...
                                "out1=convert(Array{fType,2},spzeros(outSize[1],outSize[2]));"],1)
            fWriteLine(finalID,funcBook{ii}(end-4:end-2),1);
        end
        fWriteLine(finalID,funcBook{ii}(end-1),1);
        fWriteLine(finalID,funcBook{ii}(end));
        
%         argLine=strcat("const argNum_",funcList{ii,1},"_",finalName,'=[');
%         outLine=strcat("const outNum_",funcList{ii,1},"_",finalName,'=',num2str(numel(funcList{ii,2})),";");
        argLine=strcat("const argNum_",funcList{ii,1},'=[');
        outLine=strcat("const outNum_",funcList{ii,1},'=',num2str(numel(funcList{ii,2})),";");
        inputArg=funcList{ii,3};
        inArgNum=numel(inputArg);
        for jj=1:inArgNum
            argLine=strcat(argLine,num2str(numel(inputArg{jj})),";");
        end
        argLine=strcat(argLine,"];");
        fWriteLine(finalID,[argLine;outLine]);
        
    end
    
    
%     exportLine=strcat("const funcList_",finalName,'=[');
    exportLine=strcat("const funcList=[");
    for ii=1:funcNum
%         exportLine=strcat(exportLine,funcList{ii,1},"_",finalName,";");
        exportLine=strcat(exportLine,funcList{ii,1},";");
    end
    exportLine=strcat(exportLine,"];");
    fWriteLine(finalID,"");
    fWriteLine(finalID,exportLine);
    fWriteLine(finalID,"");
    fWriteLine(finalID,strcat("end #",finalName));

    fclose(finalID);
    delete(strcat(filePath,"*.mx"));
end