function outVal=magThreshholding(inVal,inBar)

    outVal=heaviside(abs(inVal)-inBar).*inVal;
    
end