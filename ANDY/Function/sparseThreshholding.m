function outVal=magThreshholding(inVal,inBar)

% Author: Jiamin Wang; Updated: 2021-12-15;

    outVal=heaviside(abs(inVal)-inBar).*inVal;
    
end
