PathSetup;
close all;

Sim=sAxes('PassiveWalker Simulator',3,'Path_PassiveWalker.mat',@numTF_PassiveWalker_mex);
Sim.setAxesProp('BodyFrame',[-0.75 -0.25 -1.25;0.75 0.25 0.25],[30 15]).setPresetTF('WallFrame');

[LiFootTraj,LoFootTraj,InnerPose,OuterPose]=Sim.genPlot({'LiFootTraj';'LoFootTraj';'InnerPose';'OuterPose'});
% LiFootTraj.setLineSpec('--','none','y',2).setTimedPlotPoint('LiFoot');
% LoFootTraj.setLineSpec('--','none','c',2).setTimedPlotPoint('LoFoot');
InnerPose.setLineSpec('-','none','r',3).setPlotPoint({'BodyFrame';'LiKnee';'LiFoot'});
OuterPose.setLineSpec('-','none','b',3).setPlotPoint({'BodyFrame';'LoKnee';'LoFoot'});
[LiFootMarker,LiKneeMarker,BodyMarker,LoKneeMarker,LoFootMarker]=Sim.genPatch({'LiFootMarker' 'LiFoot';...
                                                                              'LiKneeMarker' 'LiKnee';...
                                                                              'BodyMarker' 'BodyFrame';...
                                                                              'LoKneeMarker' 'LoKnee';...
                                                                              'LoFootMarker' 'LoFoot'});
[Floor,LiFoot3D_L,LiFoot3D_R,LiKnee3D_L,LiKnee3D_R,Body3D,LoFoot3D_L,LoFoot3D_R,LoKnee3D_L,LoKnee3D_R]...
=Sim.genPatch({'Floor'  'WORLD';...
               'LiFoot3D_L' 'LiFoot';...
               'LiFoot3D_R' 'LiFoot';...
               'LiKnee3D_L' 'LiKnee';...
               'LiKnee3D_R' 'LiKnee';...
               'Body3D' 'BodyFrame';...
               'LoFoot3D_L' 'LoFoot';...
               'LoFoot3D_R' 'LoFoot';...
               'LoKnee3D_L' 'LoKnee';...
               'LoKnee3D_R' 'LoKnee'});
Floor.setFaceProp([0.8 0.8 1],0.75).setModel('Floor.STL',1,[],[]);
Body3D.setFaceProp('k',1).setModel('Body.STL',1,[],[]);
LiFoot3D_L.setFaceProp('r',1).setModel('Shank.STL',1,[eye(3,3),[0;0;0.04];zeros(1,3),1],[]);
LiFoot3D_R.setFaceProp('r',1).setModel('Shank.STL',1,[eye(3,3),[0;0;-0.04];zeros(1,3),1],[]);
LiKnee3D_L.setFaceProp('r',1).setModel('Thigh.STL',1,[eye(3,3),[0;0;0.04];zeros(1,3),1],[]);
LiKnee3D_R.setFaceProp('r',1).setModel('Thigh.STL',1,[eye(3,3),[0;0;-0.04];zeros(1,3),1],[]);
LoFoot3D_L.setFaceProp('g',1).setModel('Shank.STL',1,[eye(3,3),[0;0;0.12];zeros(1,3),1],[]);
LoFoot3D_R.setFaceProp('g',1).setModel('Shank.STL',1,[eye(3,3),[0;0;-0.12];zeros(1,3),1],[]);
LoKnee3D_L.setFaceProp('g',1).setModel('Thigh.STL',1,[eye(3,3),[0;0;0.12];zeros(1,3),1],[]);
LoKnee3D_R.setFaceProp('g',1).setModel('Thigh.STL',1,[eye(3,3),[0;0;-0.12];zeros(1,3),1],[]);

freq=2000;
h=1/freq;
flow=1;
tspan=0:h:120;
contState=[-0.4177,0.9086,0.4309,0.4309,0.5608,-0.2439,0.3958,0.1820,-0.4357,-0.4357,-0.0435,1.8896].';
discState=[[0.35;0;100;0.063];[0;0]];
input=0;
consForce=zeros(6,1);
tic

Sim.drawNow(0,contState,discState,input,0);

for ii=2:numel(tspan)
    flow0=flow;
    [flow,~,contState,discState,~]=Jump_PassiveWalker_mex(flow,tspan(ii),contState,discState,input,0,consForce);
    
    if(flow<0)
        break;
    end
    
    
%     if((ii>3000)&&(flow-flow0<0))
%         break;
%     end
    
    [contState,~,consForce]=rk4Hybrid(@Flow_PassiveWalker_mex,h,flow,tspan(ii),contState,discState,input,0,consForce);
    
    if(rem(ii,66)==0)
        Sim.drawNow(tspan(ii),contState,discState,input,0);
        pause(0.02);
    end
end
toc