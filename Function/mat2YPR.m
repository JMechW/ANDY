function ypr=mat2YPR(inMat)

    ypr=zeros(3,1);
    
    if isa(inMat,"sym")
        ypr=ypr*sym(0);
%         ypr(1)=atan(inMat(3,2)/inMat(3,3));
%         ypr(2)=atan(-inMat(3,1)/sqrt(inMat(3,2).^2+inMat(3,3).^2));
%         ypr(3)=atan(inMat(2,1)/inMat(1,1))
    end
%     else
        ypr(1)=atan2(inMat(3,2),inMat(3,3));
        ypr(2)=asin(-inMat(3,1));
        ypr(3)=atan2(inMat(2,1),inMat(1,1));
%     end
end