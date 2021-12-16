PathSetup;
% close all;
Sim=sAxes('Quadruped With Tail Simulator',3,'Path_QuadrupedWithTail.mat',@numTF_QuadrupedWithTail);
Sim.setAxesProp('TorsoFrame',[-0.75 -1.25 -0.75; 0.75 0.75 0.75],[210 15]).setPresetTF('WORLD');

[LFLegPose,RFLegPose,LRLegPose,RRLegPose,TorsoPose,TailPose]...
    =Sim.genPlot({'LFLegPose';'RFLegPose';'LRLegPose';'RRLegPose';'TorsoPose';'TailPose'});
LFLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegLFThighJoint';'LegLFShankJoint';'LegLFFoot'});
RFLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegRFThighJoint';'LegRFShankJoint';'LegRFFoot'});
LRLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegLRThighJoint';'LegLRShankJoint';'LegLRFoot'});
RRLegPose.setLineSpec('-','none','r',3).setPlotPoint({'LegRRThighJoint';'LegRRShankJoint';'LegRRFoot'});
TorsoPose.setLineSpec('-','none','b',3).setPlotPoint({'LegLFHipJoint';'LegRFHipJoint';'LegRRHipJoint';'LegLRHipJoint';'LegLFHipJoint'});
TailPose.setLineSpec('-','none','g',3).setPlotPoint({'TailYaw';'TailCOM'});

[Floor,Torso,Tail,LFThigh,LFShank,RFThigh,RFShank,LRThigh,LRShank,RRThigh,RRShank]...
=Sim.genPatch({'Floor'  'WORLD';...
               'Torso' 'TorsoFrame';...
               'Tail' 'TailCOM';...
               'LFThigh' 'LegLFThighCOM';...
               'LFShank' 'LegLFShankCOM';...
               'RFThigh' 'LegRFThighCOM';...
               'RFShank' 'LegRFShankCOM';...
               'LRThigh' 'LegLRThighCOM';...
               'LRShank' 'LegLRShankCOM';...
               'RRThigh' 'LegRRThighCOM';...
               'RRShank' 'LegRRShankCOM';...
               });
Floor.setFaceProp([0.8 0.8 1],0.75).setModel('Floor.STL',1,[],0.001);
Torso.setFaceProp('b',0.75).setModel('Torso.STL',1,[],0.001);
Tail.setFaceProp('g',0.75).setModel('Tail.STL',1,[],0.001);
LFThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
LFShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);
RFThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
RFShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);
LRThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
LRShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);
RRThigh.setFaceProp('r',0.75).setModel('Thigh.STL',1,[],0.001);
RRShank.setFaceProp('y',0.75).setModel('Shank.STL',1,[],0.001);

%%


tic
freq=2500;
h=1/freq;
flow=2;
tspan=0:h:0.25;
initTorsoPos=[0.2;0.3;0.4;0;0;0];
initLegPos2=[0.0749;-0.8386-0.1391;1.6570];
initLegPos1=[0.0749;-0.8386;1.6771];
initLegPos3=[0.0749;-0.8386;1.6771];
initLegPos4=[0.0749;-0.8386+0.1391;1.6570];
initTailPos=[0;0];
initPos=[initTorsoPos;initLegPos1;initLegPos2;initLegPos3;initLegPos4;initTailPos];
initVel=zeros(20,1);
contState=[initPos;initVel];
discState=[-0.01;0.57;0.41;0.63;-0.01;0.03;0.41;-0.03;0;100];
consForce=zeros(12,1);
legForce=[0;0;-15];
input=zeros(14,1);
input=[legForce;legForce;legForce/15*20.5;legForce/15*20.5;-7.7;-10*sin(31.4*0)];
nhSignal=[1;0;0;0];
% Sim.drawNow(0,[initPos;initVel],zeros(10,1),zeros(14,1),nhSignal);

XYZ=diag([0.2;0.3;0.4])*ones(3,numel(tspan));
for ii=2:numel(tspan)
    [contState,nhSignal,consForce]=rk4Hybrid(@Flow_QuadrupedWithTail,h,flow,tspan(ii),contState,discState,input,nhSignal,consForce);
    
        [~,~,~,FrameJac,~,Quat,MM,GFI,GFF,GFU,JacU,JacCons1,~,~,CenMat,InertLeftComponent,InertRightComponent1]...
            =System_QuadrupedWithTail(0,contState,discState,input,nhSignal);
        
        sum(GFF,2).'
        
    input=[legForce;legForce;legForce/15*20.5;legForce/15*20.5;-7.7;-10*sin(31.4*tspan(ii)/2)];

%     if(rem(ii,10)==0)
%         Sim.drawNow(tspan(ii),contState,discState,input,nhSignal);
%     end
end
toc
