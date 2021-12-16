function [q1,s1,l1,q1d,s1d]=rk4Hybrid(Func,h,flow,t0,q0,p0,u0,s0,l0)
% [flow,t1,q1,p0,u0,s1,l1]=rk4Hybrid(Func,h,flow,t0,q0,p0,u0,s0,l0)
% Runge-Kutta 4th method that output the result continuous state [q1],
% nonholonomic signal [s1] and lagrangian multiplier [l1] at time t+dt from
% a Hybrid ODE Flow state machine <Func> by giving flow mode <flow>, dt <h>,
% t <t0>, continuous state <q0>, discrete state <p0>, input <u0>, signal
% <s0> and lagrangian multiplier [l0] at time t respectively.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    [dq1,ds1,ll1] = Func(flow,t0,q0,p0,u0,s0,l0);
    [dq2,ds2,ll2] = Func(flow,t0+0.5*h,q0+0.5*h*dq1,p0,u0,s0+0.5*h*ds1,ll1);
    [dq3,ds3,ll3] = Func(flow,t0+0.5*h,q0+0.5*h*dq2,p0,u0,s0+0.5*h*ds2,ll2);
    [dq4,ds4,ll4] = Func(flow,t0+h,q0+h*dq3,p0,u0,s0+h*ds3,ll3);

    q1d=(1/6)*(dq1+2*dq2+2*dq3+dq4);
    s1d=(1/6)*(ds1+2*ds2+2*ds3+ds4);
    q1 = q0 + q1d*h;  % main equation
    s1 = s0 + s1d*h;  % nonholonomic equation
    l1 = (1/6)*(ll1+2*ll2+2*ll3+ll4);
%     t1 = t0 + h;
end
