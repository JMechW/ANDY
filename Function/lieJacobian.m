function [output,jac]=lieJacobian(x,h,f)
% output=lieJacobian(x,h,f)
% Calculate the lie jacobian as "output = (dh/dx)*f"

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    jac=jacobian(h,x);
    output=jac*f;
end