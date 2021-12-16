function angVelJac=ypr2AngVelJac(ypr)
% Author: Jiamin Wang; Updated: 2021-12-15;

    
    roll=ypr(1);
    pitch=ypr(2);
    yaw=ypr(3);

    angVelJac=...
    [1 0 -sin(pitch); ...
    0 cos(roll) cos(pitch)*sin(roll); ...
    0 -sin(roll) cos(pitch)*cos(roll)];
    
end
