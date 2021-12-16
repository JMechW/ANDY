function [y1]=rk4(Func,h,t0,y0,u0)
% [y1]=rk4(Func,h,t0,y0,u0)
% Runge-Kutta 4th method outputing state [y1] at t+dt by giving ODE <func>,
% dt <h>, t <t0>, state <y0> and input <u0> at t respectively.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    k_1 = Func(t0,y0,u0);
    k_2 = Func(t0+0.5*h,y0+0.5*h*k_1,u0);
    k_3 = Func((t0+0.5*h),(y0+0.5*h*k_2),u0);
    k_4 = Func((t0+h),(y0+k_3*h),u0);

    y1 = y0 + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;  % main equation
    t1 = t0 + h;
end
