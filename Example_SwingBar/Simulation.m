[contSym]=SB.Model.Continuous;
[sigSym]=SB.Model.NHSignal(:,1);
[paramSym]=SB.Model.Parameter(:,1);
[paramVal]=SB.Model.Parameter(:,2);
Frame=[Bar1End.rootTransDis,Bar2End.rootTransDis,Bar3End.rootTransDis].';
FrameFunc=matlabFunction(subs(Frame(:,1:3),paramSym,paramVal),'Vars',{contSym,sigSym});
freq=1000;
h=1/freq;
tspan=0:h:30;
disc=[1;0.2];
odeState=[0;0;0;0;-5;0;0;0];
stateRecord=zeros(8,numel(tspan));
sigState=ypr2Quat([0 pi/2 0].');
sigRecord=zeros(4,numel(tspan));
posRecordX=zeros(3,numel(tspan));
posRecordY=zeros(3,numel(tspan));
posRecordZ=zeros(3,numel(tspan));
curPos=FrameFunc(odeState,sigState);
figure(3)
handle=plot3([0;curPos(:,1)],[0;curPos(:,2)],[0;curPos(:,3)],'k','LineWidth',10);
axis([-3,3,-3,3,-2,2]);
stateRecord(:,1)=odeState;
sigRecord(:,1)=sigState;
posRecordX(:,1)=curPos(:,1);
posRecordY(:,2)=curPos(:,2);
posRecordZ(:,3)=curPos(:,3);

tic
for ii=2:numel(tspan)
    [curTF,~,~,~,~,curQuat]=numKinematics_SwingBar_mex(tspan(ii),odeState,disc,0,sigState);
    curRotMat=curTF(1:3,1:3,8);
    quat2Mat(curQuat(:,8))-curRotMat;
    
    
    [odeState,sigState,l1]=rk4Hybrid(@Flow_SwingBar_mex,h,1,tspan(ii),odeState,disc,0,sigState,0);
    curPos=FrameFunc(odeState,sigState);
    if(rem(ii,30)==0)
        handle.set('XData',[0;curPos(:,1)],'YData',[0;curPos(:,2)],'ZData',[0;curPos(:,3)]);
        drawnow
    end
    stateRecord(:,ii)=odeState;
    sigRecord(:,ii)=sigState;
    posRecordX(:,ii)=curPos(:,1);
    posRecordY(:,ii)=curPos(:,2);
    posRecordZ(:,ii)=curPos(:,3);
end
toc

fig=figure(5);
sub1=subplot(4,1,1);
hold on
axis([-3,3,-3,3,-2,2],'equal');
sub1.View=[45,5];
plot3([0;curPos(:,1)],[0;curPos(:,2)],[0;curPos(:,3)],'g','LineWidth',5);
plot3(posRecordX(1,1:100:end),posRecordY(1,1:100:end),posRecordZ(1,1:100:end),'-.k','LineWidth',2)
plot3(posRecordX(2,1:100:end),posRecordY(2,1:100:end),posRecordZ(2,1:100:end),'r','LineWidth',1)
plot3(posRecordX(3,1:100:end),posRecordY(3,1:100:end),posRecordZ(3,1:100:end),':b','LineWidth',1)
title('3D Trajectory in World Frame')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Bodies','H.Bar End Traj','BS.Bar End Traj','ES.Bar End Traj','location','southoutside')
axis('image')
hold off
sub2=subplot(4,1,2);
hold on
plot(tspan(1:500:end),sigRecord(1,1:500:end),'-or','LineWidth',1);
plot(tspan(1:500:end),sigRecord(2,1:500:end),'-*g','LineWidth',1);
plot(tspan(1:500:end),sigRecord(3,1:500:end),'-b','LineWidth',1);
plot(tspan(1:500:end),sigRecord(4,1:500:end),'--k','LineWidth',1);
title('Quaternion of Ball Joint Rotation')
xlabel('Time (s)')
ylabel('Value')
legend('Qw','Qx','Qy','Qz','location','northoutside','Orientation','horizontal')
hold off
sub3=subplot(4,1,3);
hold on
plot(tspan(1:500:end),posRecordX(1,1:500:end),'-og','LineWidth',1);
plot(tspan(1:500:end),posRecordX(2,1:500:end),'-*b','LineWidth',1);
plot(tspan(1:500:end),posRecordX(3,1:500:end),':r','LineWidth',3);
title('X Coordinate Trajectory of Bar End Points')
xlabel('Time (s)')
ylabel('X (m)')
legend('H.Bar','BS.Bar','ES.Bar','location','northoutside','Orientation','horizontal')
hold off
sub4=subplot(4,1,4);
hold on
plot(tspan(1:500:end),posRecordZ(1,1:500:end),'-*g','LineWidth',1);
plot(tspan(1:500:end),posRecordZ(2,1:500:end),'-ob','LineWidth',1);
plot(tspan(1:500:end),posRecordZ(3,1:500:end),':r','LineWidth',3);
title('Z Coordinate Trajectory of Bar End Points')
xlabel('Time (s)')
ylabel('Z (m)')
legend('H.Bar','BS.Bar','ES.Bar','location','northoutside','Orientation','horizontal')
hold off

set(sub1,'FontName','Times New Roman','FontWeight','bold')
set(sub2,'FontName','Times New Roman','FontWeight','bold')
set(sub3,'FontName','Times New Roman','FontWeight','bold')
set(sub4,'FontName','Times New Roman','FontWeight','bold')