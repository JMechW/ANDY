PathSetup;
PathSetup;

%Declare Time Variable and System
t=sym('t','real');
SB=kSystem('SwingBar',t);
SB.Space.setPrecompile(true);
SB.Model.setPrecompile(true);

%Declare Coordinate Frames
[Bar1End,Bar1COM,BallJoint,Bar2End,Bar2COM,Bar3End,Bar3COM]...
=SB.Space.genNode({'Bar1End';'Bar1COM';'BallJoint';'Bar2End';'Bar2COM';'Bar3End';'Bar3COM'});

%Declare Constant Parameters
C=SB.genParam({
             'g' 'Gravity Acceleration' 9.81;
             'm1' 'Bar 1 Mass' 1;
             'l1' 'Bar 1 Length' 1;
             'm2' 'Bar 2 Mass' 1;
             'l2' 'Bar 2 Length' 1;
             'm3' 'Bar 3 Mass' 0.5;
             'l3' 'Bar 3 Length' 0.5;
             });
           
%Declare System Inputs 
U=SB.genDisc({
             'T' 'Bar 1 Torque';
             'b' 'Bar Damper';
             });
            
%Declare Continuous States (Generalized Coordinates) 
[q1,qx,qy,qz]...
=SB.genCont({
            'q1' 'Pan Bar Angle';
            'qx' 'Ball Joint X Rotation';
            'qy' 'Ball Joint Y Rotation';
            'qz' 'Ball Joint Z Rotation';
            });

%Declare Kinematic Links
[Root2Bar1End,Bar1End2Bar1COM,Bar1End2BallJoint,BallJoint2Bar2End,Bar2End2Bar2COM,Bar2End2Bar3End,Bar3End2Bar3COM]...
=SB.Space.genEdge({
            'Root2Bar1End' SB.Space.RootFrame.NameTag 'Bar1End';
            'Bar1End2Bar1COM' 'Bar1End' 'Bar1COM';
            'Bar1End2BallJoint' 'Bar1End' 'BallJoint';
            'BallJoint2Bar2End' 'BallJoint' 'Bar2End';
            'Bar2End2Bar2COM' 'Bar2End' 'Bar2COM';
            'Bar2End2Bar3End' 'Bar2End' 'Bar3End';
            'Bar3End2Bar3COM' 'Bar3End' 'Bar3COM';
            });
Root2Bar1End.setDH([0;q1.dot(0);C.l1;0]).genProp;
Bar1End2Bar1COM.setDH([0;0;-0.5*C.l1;0]).genProp;
Bar1End2BallJoint.setAngVel([qx.dot(1);qy.dot(1);qz.dot(1)],'bj').genProp;
BallJoint2Bar2End.setDH([-C.l2;0;0;0]).genProp;
Bar2End2Bar2COM.setDH([0.5*C.l2;0;0;0]).genProp;
Bar2End2Bar3End.setDH([0;0;C.l3;0]).genProp;
Bar3End2Bar3COM.setDH([0;0;-C.l3*0.5;0]).genProp;
SB.Space.plotGraph(1);
SB.Space.makeNumKinematics();

%% Dynamics Part

%Declare Bodies and Set Inertial Properties
[Bar1,Bar2,Bar3]...
=SB.genBody({
            'Bar1' 'Bar1COM';
            'Bar2' 'Bar2COM';
            'Bar3' 'Bar3COM';
            });
Bar1.setProp(C.m1,diag([0;1/12*C.m1*(C.l1)^2;1/12*C.m1*(C.l1)^2]));
Bar2.setProp(C.m2,diag([1/12*C.m2*(C.l2)^2;1/12*C.m2*(C.l2)^2;0]));
Bar3.setProp(C.m3,diag([0;1/12*C.m3*(C.l3)^2;1/12*C.m3*(C.l3)^2]));
Gravity1=Bar1.genForce({'Gravity1' Bar1COM SB.Space.RootFrame}).setProp([0;0;-C.m1*C.g]);
Gravity2=Bar2.genForce({'Gravity2' Bar2COM SB.Space.RootFrame}).setProp([0;0;-C.m2*C.g]);
Gravity3=Bar3.genForce({'Gravity3' Bar3COM SB.Space.RootFrame}).setProp([0;0;-C.m3*C.g]);
Torque1=Bar1.genTorque({'Torque1' Bar1COM SB.Space.RootFrame}).setProp([0;0;U.T]);

%Declare Dampers and Set Damper Properties
Bar1Damper=SB.genDamper({'Bar1Damper' 3 1});
Bar1Damper.setProp(U.b,Bar1.BaseFrame.getTransSym(1));
Bar2Damper=SB.genDamper({'Bar2Damper' 3 1});
Bar2Damper.setProp(U.b,Bar2.BaseFrame.getTransSym(1));
Bar3Damper=SB.genDamper({'Bar3Damper' 3 1});
Bar3Damper.setProp(U.b,Bar3.BaseFrame.getTransSym(1));

%Modeling Generation Initialization
[oMode]=SB.Model.Init().genNode({'Open KChain'});
oMode.genFlowEOM({});
SB.Model.plotGraph(2);
SB.Model.makeDynamics('num');
SB.Model.makeSystemDynamicsForControl('num');

Simulation;