function [YPR] = quat2YPR(inQuat)
% [YPR] = quat2YPR(inQuat)
% transfer quaternion to Yaw-Pitch-Roll (ZYX) angles

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    qw=inQuat(1);
    qx=inQuat(2);
    qy=inQuat(3);
    qz=inQuat(4);

    YPR = [  atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 );
    asin( -2*(qx.*qz-qw.*qy) );
    atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 );];
    
%     YPR = [ atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 ); 
%         asin( -2*(qx.*qz-qw.*qy) );
%         atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 );];
end

