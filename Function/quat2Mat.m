function QuatMat=quat2Mat(inQuat)
% QuatMat=quat2Mat(inQuat)
% A function that convert Quaternion <inQuat> to Rotation Matrix <QuatMat>.
% From Child to Parent

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    qw=inQuat(1);
    qx=inQuat(2);
    qy=inQuat(3);
    qz=inQuat(4);
    QuatMat=...
    [...
    1-2*qy^2-2*qz^2,  2*qx*qy-2*qz*qw,  2*qx*qz+2*qy*qw;...
    2*qx*qy+2*qz*qw,  1-2*qx^2-2*qz^2,  2*qy*qz-2*qx*qw;...
    2*qx*qz-2*qy*qw,  2*qy*qz+2*qx*qw,  1-2*qx^2-2*qy^2;...
    ];
end
