function angVelJac=ypr2AngVelJac(ypr)
    
    roll=ypr(1);
    pitch=ypr(2);
    yaw=ypr(3);

    angVelJac=...
    [1 0 -sin(pitch); ...
    0 cos(roll) cos(pitch)*sin(roll); ...
    0 -sin(roll) cos(pitch)*cos(roll)];
    
end