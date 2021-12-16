function quat=mat2Quat(mat)
    trace=mat(1,1)+mat(2,2)+mat(3,3);
    quat=zeros(4,1);
    if isa(mat,'sym')
        S=2*sqrt(1+trace);
        quat=sym(0)*zeros(4,1);
        quat(1)=S/4;
        quat(2)=(mat(3,2)-mat(2,3))/S;
        quat(3)=(mat(1,3)-mat(3,1))/S;
        quat(4)=(mat(2,1)-mat(1,2))/S;
    elseif trace>0
        S=2*sqrt(1+trace);
        quat(1)=S/4;
        quat(2)=(mat(3,2)-mat(2,3))/S;
        quat(3)=(mat(1,3)-mat(3,1))/S;
        quat(4)=(mat(2,1)-mat(1,2))/S;
    elseif (mat(1,1)>mat(2,2))&&(mat(1,1)>mat(3,3))
        S=2*sqrt(1+mat(1,1)-mat(2,2)-mat(3,3));
        quat(1)=(mat(3,2)-mat(2,3))/S;
        quat(2)=S/4;
        quat(3)=(mat(1,2)+mat(2,1))/S;
        quat(4)=(mat(1,3)+mat(3,1))/S;
    elseif (mat(2,2)>mat(3,3))
        S=2*sqrt(1-mat(1,1)+mat(2,2)-mat(3,3));
        quat(1)=(mat(1,3)-mat(3,1))/S;
        quat(2)=(mat(2,1)+mat(1,2))/S;
        quat(3)=S/4;
        quat(4)=(mat(2,3)+mat(3,2))/S;
    else
        S=2*sqrt(1-mat(1,1)-mat(2,2)+mat(3,3));
        quat(1)=(mat(2,1)-mat(1,2))/S;
        quat(2)=(mat(3,1)+mat(1,3))/S;
        quat(3)=(mat(3,2)+mat(2,3))/S;
        quat(4)=S/4;
    end
end