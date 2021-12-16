function [QuatData] = ypr2Quat(yprData)
% [QuatData] = ypr2Quat(yprData)
% Converts Yaw-Pitch-Roll to Quaternion

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

roll=yprData(1,1);pitch=yprData(2,1); yaw=yprData(3,1);

QuatData=quatMultiply([cos(yaw/2);0;0;sin(yaw/2)],quatMultiply([cos(pitch/2);0;sin(pitch/2);0],[cos(roll/2);sin(roll/2);0;0]));

end