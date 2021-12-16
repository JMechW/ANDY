PathSetup;
PathSetup;

%% Declare Time Variable and System
t=sym('t','real');
PW=kSystem('PassiveWalker',t);

%% Declare Constant Parameters
C=PW.genParam({
             'g' 'Gravity Acceleration' 9.81;
             'm1' 'Leg Thigh Mass' 10;
             'l1' 'Leg Thigh Length' 0.5;
             'm2' 'Leg Shank Mass' 5;
             'l2' 'Leg Shank Length' 0.5;
             'M' 'Body Mass' 56;
             });
                     
%% Declare Continuous States (Generalized Coordinates) 
[qx,qy,qi1,qi2,qo1,qo2]...
=PW.genCont({
            'qx' 'Body X Displacement';
            'qy' 'Body Y Displacement';
            'qi1' 'Inner Leg Thigh Angle';
            'qi2' 'Inner Leg Shank Angle';
            'qo1' 'Outer Leg Thigh Angle';
            'qo2' 'Outer Leg Shank Angle';
            });
        
%% Declare System Inputs 
U=PW.genDisc({
             'a' 'Slope Angle';
             'b' 'Global Damping Coefficient';
             'c' 'Global Soft Constraint Factor';
             'ba' 'bend angle addition';
             });
         
P=PW.genDisc({
             'fx' 'Fix Foot X';
             'la' 'lock Angle';
             });
        
%% Declare Coordinate Frames
[GroundFrame,WallFrame,BodyFrame,...
 LiKnee,LoKnee,LiFoot,LoFoot,...
 LiThighCOM,LoThighCOM,LiShankCOM,LoShankCOM]...
=PW.Space.genNode({'GroundFrame';'WallFrame';'BodyFrame';
                   'LiKnee';'LoKnee';'LiFoot';'LoFoot';
                   'LiThighCOM';'LoThighCOM';'LiShankCOM';'LoShankCOM';
                 });

%% Declare Kinematic Links between Coordinate Frames
[WORLD2Ground,Ground2Wall,WORLD2Body,Body2LiKnee,Body2LoKnee,LiKnee2LiFoot,LoKnee2LoFoot,...
 LiKnee2LiThighCOM,LoKnee2LoThighCOM,LiFoot2LiShankCOM,LoFoot2LoShankCOM,]...
=PW.Space.genEdge({
            'WORLD To Ground' PW.Space.RootFrame.NameTag 'GroundFrame';
            'Ground To Wall' 'GroundFrame' 'WallFrame';
            'WORLD To Body' PW.Space.RootFrame.NameTag 'BodyFrame';
            'Body To LiKnee' 'BodyFrame' 'LiKnee';
            'Body To LoKnee' 'BodyFrame' 'LoKnee';
            'LiKnee To LiFoot' 'LiKnee' 'LiFoot';
            'LoKnee To LoFoot' 'LoKnee' 'LoFoot';
            'LiKnee To LiThighCOM' 'LiKnee' 'LiThighCOM';
            'LoKnee To LoThighCOM' 'LoKnee' 'LoThighCOM';
            'LiFoot To LiShankCOM' 'LiFoot' 'LiShankCOM';
            'LoFoot To LoShankCOM' 'LoFoot' 'LoShankCOM';
            });
PW.Space.plotGraph(1);
WORLD2Ground.setAxang([U.a;0;0;1]).genProp;
Ground2Wall.setAxang(sym([-pi/2;1;0;0])).genProp;
WORLD2Body.setTransDis([qx.dot(0);qy.dot(0);0]).genProp;
Body2LiKnee.setDH([0;qi1.dot(0)+pi/2;-C.l1;0]).genProp;
Body2LoKnee.setDH([0;qo1.dot(0)+pi/2;-C.l1;0]).genProp;
LiKnee2LiFoot.setDH([0;qi2.dot(0)-qi1.dot(0);-C.l2;0]).genProp;
LoKnee2LoFoot.setDH([0;qo2.dot(0)-qo1.dot(0);-C.l2;0]).genProp;
LiKnee2LiThighCOM.setDH([0;0;0.6*C.l1;0]).genProp;
LoKnee2LoThighCOM.setDH([0;0;0.6*C.l1;0]).genProp;
LiFoot2LiShankCOM.setDH([0;0;0.44*C.l2;0]).genProp;
LoFoot2LoShankCOM.setDH([0;0;0.44*C.l2;0]).genProp;
PW.Space.makeNumKinematics();

%% Declare Bodies and Set Inertial Properties
[body,li1,li2,lo1,lo2]...
=PW.genBody({
            'body' 'BodyFrame';
            'li1' 'LiThighCOM';
            'li2' 'LiShankCOM';
            'lo1' 'LoThighCOM';
            'lo2' 'LoShankCOM';
            });
body.setProp(C.M);
li1.setProp(C.m1);
li2.setProp(C.m2);
lo1.setProp(C.m1);
lo2.setProp(C.m2);
GBody=body.genForce({'GBody' BodyFrame GroundFrame}).setProp([0;-C.M*C.g;0]);
Gi1=li1.genForce({'Gi1' LiThighCOM GroundFrame}).setProp([0;-C.m1*C.g;0]);
Go1=lo1.genForce({'Go1' LoThighCOM GroundFrame}).setProp([0;-C.m1*C.g;0]);
Gi2=li2.genForce({'Gi2' LiShankCOM GroundFrame}).setProp([0;-C.m2*C.g;0]);
Go2=lo2.genForce({'Go2' LoShankCOM GroundFrame}).setProp([0;-C.m2*C.g;0]);

%% Damper
JointDamper=PW.genDamper({'JointDamper' 6 1});
JointDamper.setProp(sym(U.b*eye(6,6)),[qx.dot(1);qy.dot(1);qi1.dot(1);qi2.dot(1);qo1.dot(1);qo2.dot(1)]);

%% Constraints
[iKneeLock,iFootLockX,iFootLockY,oKneeLock,oFootLockX,oFootLockY]...
=PW.genCons({'iKneeLock' 'ik';
             'iFootLockX' 'ihx';
             'iFootLockY' 'ihy';
             'oKneeLock' 'ok';
             'oFootLockX' 'ohx';
             'oFootLockY' 'ohy';
           });
iKneeLock.setHCons(qi1.dot(0)-qi2.dot(0),U.c);
iFootLockX.setHCons(jSimplify([1 0 0]*(LiFoot.getTransSym(0)-[P.fx;0;0])),U.c);
iFootLockY.setHCons(jSimplify([0 1 0]*(LiFoot.getTransSym(0)-[P.fx;0;0])),U.c);
oKneeLock.setHCons(qo1.dot(0)-qo2.dot(0),U.c);
oFootLockX.setHCons(jSimplify([1 0 0]*(LoFoot.getTransSym(0)-[P.fx;0;0])),U.c);
oFootLockY.setHCons(jSimplify([0 1 0]*(LoFoot.getTransSym(0)-[P.fx;0;0])),U.c);

%% Modeling Generation Initialization
PW.Model.Init();
[IFixOOpen,IFixOLock,IBendOLock,IOpenOFix,ILockOFix,ILockOBend]=...
PW.Model.genNode({'IFixOOpen';'IFixOLock';'IBendOLock';'IOpenOFix';'ILockOFix';'ILockOBend'});
[lockOKnee,bendIKnee,fixOFoot,lockIKnee,bendOKnee,fixIFoot]=...
PW.Model.genEdge({
                'Lock Outer Knee' 'IFixOOpen' 'IFixOLock';
                'Bend Inner Knee' 'IFixOLock' 'IBendOLock';
                'Fix Outer Foot' 'IBendOLock' 'IOpenOFix';
                'Lock Inner Knee' 'IOpenOFix' 'ILockOFix';
                'Bend Outer Knee' 'ILockOFix' 'ILockOBend';
                'Fix Inner Foot' 'ILockOBend' 'IFixOOpen';
                });
PW.Model.plotGraph(2);
IFixOOpen.genFlowEOM({'iKneeLock';'iFootLockX';'iFootLockY'});
IFixOLock.genFlowEOM({'oKneeLock';'iKneeLock';'iFootLockX';'iFootLockY'});
IBendOLock.genFlowEOM({'oKneeLock';'iFootLockX';'iFootLockY'});
IOpenOFix.genFlowEOM({'oKneeLock';'oFootLockX';'oFootLockY'});
ILockOFix.genFlowEOM({'iKneeLock';'oKneeLock';'oFootLockX';'oFootLockY'});
ILockOBend.genFlowEOM({'iKneeLock';'oFootLockX';'oFootLockY'});
lockOKnee.makeGuardReset(qo2.dot(0)>=qo1.dot(0),...
                        {qo2.dot(0), qo1.dot(0);
                         P.la,       qi1.dot(0);});
bendIKnee.makeGuardReset((qi1.dot(0)-P.la+U.ba)<=0,...
                        {});
fixOFoot.makeGuardReset(([0 1 0]*LoFoot.rootTransDis())<=0,...
                        {qy.dot(0),  [C.l1,C.l2]*[cos(qo1.dot(0));cos(qo2.dot(0))];
                         P.fx,       [1 0 0]*LoFoot.rootTransDis();});
lockIKnee.makeGuardReset(qi2.dot(0)>=qi1.dot(0),...
                        {qi2.dot(0), qi1.dot(0);
                         P.la,       qo1.dot(0);});
bendOKnee.makeGuardReset((qo1.dot(0)-P.la+U.ba)<=0,...
                        {});
fixIFoot.makeGuardReset(([0 1 0]*LiFoot.rootTransDis())<=0,...
                        {qy.dot(0),  [C.l1,C.l2]*[cos(qi1.dot(0));cos(qi2.dot(0))];
                         P.fx,       [1 0 0]*LiFoot.rootTransDis();});
PW.Model.makeDynamics('num');


[contSym]=PW.Model.Continuous;
[paramSym]=PW.Model.Parameter(:,1);
[paramVal]=PW.Model.Parameter(:,2);
Frame=[LoFoot.rootTransDis,LoKnee.rootTransDis,BodyFrame.rootTransDis,LiKnee.rootTransDis,LiFoot.rootTransDis].';
FrameFunc=matlabFunction(subs(Frame(:,1:2),paramSym,paramVal),'Vars',{contSym});
freq=1000;
h=1/freq;
tspan=0:h:6.5;
flow=1;
initAnglePos=0.4309;
initAngVel=-0.1391;
slopeAngle=0.35;
contState=[-sin(initAnglePos);cos(initAnglePos);initAnglePos;initAnglePos;0.5608;-0.2439;...
           -initAngVel*cos(initAnglePos)*sqrt(9.81);-initAngVel*sin(initAnglePos)*sqrt(9.81);initAngVel*sqrt(9.81);initAngVel*sqrt(9.81);-0.0139*sqrt(9.81);0.6033*sqrt(9.81)];
discState=[[slopeAngle;0;100;0.063];0;0];
input=0;
consForce=zeros(6,1);
curPos=FrameFunc(contState)*[cos(slopeAngle),-sin(slopeAngle);sin(slopeAngle),cos(slopeAngle)];
road=[-0.5+discState(1),2.5+discState(1)].'.*[cos(slopeAngle),-sin(slopeAngle)];
close all;
figHandle=figure(3)
hold on
roadHandle=plot([road(:,1)],[road(:,2)],'b','LineWidth',3);
handle=plot([curPos(:,1)],[curPos(:,2)],'LineWidth',3);
% axis('image');
axe=gca;
axe.set('XLim',[-0.5 3]);
axe.set('YLim',[-1 1.25]);
axe.set('DataAspectRatio',[1 1 1]);

flowTrajRec=diag(flow)*ones(1,numel(tspan));
contRec=diag(contState)*ones(12,numel(tspan));
discRec=diag(discState)*ones(6,numel(tspan));
posXRec=diag(curPos(:,1))*ones(5,numel(tspan));
posYRec=diag(curPos(:,2))*ones(5,numel(tspan));

errorRec=zeros(2,numel(tspan));

tic
for ii=2:numel(tspan)
    flow0=flow;
    [flow,~,contState,discState,~]=Jump_PassiveWalker(flow,tspan(ii),contState,discState,input,0,consForce);
    if(flow<0)
        break;
    end
    if(flow~=flow0)&&((flow==4)||(flow==1))
        disp(contState)
    end
    [contState,~,consForce]=rk4Hybrid(@Flow_PassiveWalker,h,flow,tspan(ii),contState,discState,input,0,consForce);
    curPos=FrameFunc(contState)*[cos(slopeAngle),-sin(slopeAngle);sin(slopeAngle),cos(slopeAngle)];
    
    flowTrajRec(:,ii)=flow;
    contRec(:,ii)=contState;
    discRec(:,ii)=discState;
    posXRec(:,ii)=curPos(:,1);
    posYRec(:,ii)=curPos(:,2);
    
    if(1<=flow)&&(flow<=3)
        errorRec(:,ii)=curPos(5,:).'-[cos(slopeAngle)*discState(1);-sin(slopeAngle)*discState(1)];
    else
        errorRec(:,ii)=curPos(1,:).'-[cos(slopeAngle)*discState(1);-sin(slopeAngle)*discState(1)];
    end
    
    if(rem(ii,30)==0)
        handle.set('XData',[curPos(:,1)],'YData',[curPos(:,2)]);
        drawnow
        pause(0.03)
    end
end
hold off
toc

figure(4)
hold on
ii=1;
hh(1)=plot(posXRec(1:3,ii),posYRec(1:3,ii),':or','LineWidth',1.5,'MarkerSize',4);
hh(2)=plot(posXRec(3:5,ii),posYRec(3:5,ii),':*b','LineWidth',1.5,'MarkerSize',3);
for ii=1:350:numel(tspan)
    plot(posXRec(1:3,ii),posYRec(1:3,ii),':or','LineWidth',1.5);
    plot(posXRec(3:5,ii),posYRec(3:5,ii),':*b','LineWidth',1.5);
end
aax0=hh(1).Parent;
set(aax0,'FontName','Times New Roman','FontWeight','bold','FontSize',12)
axis('image');
h1=plot([road(:,1)],[road(:,2)],':g','LineWidth',3);
h2=plot([posXRec(3,:)],[posYRec(3,:)],'-.c','LineWidth',3);
h3=plot([posXRec(1,:)],[posYRec(1,:)],'--m','LineWidth',3);
h4=plot([posXRec(5,:)],[posYRec(5,:)],'k','LineWidth',3);
legend([h1,h2,h3,h4,hh(1),hh(2)],{'Slope','Body Trajectory','Foot O Trajectory','Foot I Trajectory','Outer Leg','Inner Leg'},'location','eastoutside')
title('Trajectory of Passive Walker in World Frame');
xlabel('X (m)');
ylabel('Y (m)');
hold off

figure(5)
axx1=subplot(3,1,1:2)
hold on
plot(tspan(1:33:end),contRec(3,1:33:end),'k','LineWidth',2);
plot(tspan(1:33:end),contRec(4,1:33:end),'-*r','LineWidth',1);
plot(tspan(1:33:end),contRec(5,1:33:end),'-ob','LineWidth',1);
plot(tspan(1:33:end),contRec(6,1:33:end),':g','LineWidth',3);
legend({'Qi1','Qi2','Qo1','Qo1'},'location','southoutside','orientation','horizontal')
title('Trajectory of Angles');
xlabel('Time (s)');
ylabel('Angle (rad)');
set(axx1,'FontName','Times New Roman','FontWeight','bold')
axis([0,5,-0.3,0.6])
hold off
axx2=subplot(3,1,3)
hold on
plot(tspan(1:end),flowTrajRec(1,1:end),'LineWidth',2);
title('Trajectory of Jumps');
xlabel('Time (s)');
ylabel('Flow Numer');
set(axx2,'FontName','Times New Roman','FontWeight','bold')
axis('tight')
hold off
%}