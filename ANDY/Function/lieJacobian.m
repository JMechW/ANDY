function [output,jac]=lieJacobian(x,h,f)
% output=lieJacobian(x,h,f)
% Calculate the lie jacobian as "output = (dh/dx)*f"

% Author: Jiamin Wang; Updated: 2021-12-15;


    jac=jacobian(h,x);
    output=jac*f;
end
