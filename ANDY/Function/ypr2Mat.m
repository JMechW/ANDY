function RPYMat=ypr2Mat(yprData)
% RPYMat=ypr2Mat(yprData)
% Converts Yaw-Pitch-Roll to Quaternion
% (vector presentation transformed from child frame to parent frame).

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    roll=yprData(1,1);pitch=yprData(2,1); yaw=yprData(3,1);

    YawMat=[cos(yaw) sin(yaw) 0;-sin(yaw) cos(yaw) 0;0 0 1;];
    PitchMat=[cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch);];
    RollMat=[1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll);];
    RPYMat=RollMat*PitchMat*YawMat;
    RPYMat=RPYMat.';
end
