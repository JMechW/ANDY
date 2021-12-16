function [out0,out1]=sparseSmoothFunc_Sum(order,sigma,magma,xx,xxbb,xxAlpha,xxrr)
    if order<=0
        error("l-0 minimization is not supported; for l-1 minimization, use order=0.5!");
    elseif numel(order)>1
        error("order should be a scalar!");
    end
    
    scaledDifference=xxAlpha.*(xx-xxbb);
    medTerm1=(scaledDifference.^2 + xxrr.^2);
    medTerm2=medTerm1.^(order);
    
    out0 = (sum(medTerm2 - xxrr.^(2*order))+sigma^2)^0.5-sigma;
    out1 = magma*reshape(((1/(out0+sigma)))*((order).*(xxAlpha).*(medTerm2./medTerm1).*scaledDifference),1,[]);
    out0 = magma*out0;
    
end