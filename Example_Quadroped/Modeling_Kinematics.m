%% Declare Coordinate Frames
[TorsoFrame]...
=System.Space.genNode({
                     'TorsoFrame';
                     });
                 
[TailRoll,TailYaw,TailCOM]...
=System.Space.genNode({
                     'TailRoll';
                     'TailYaw';
                     'TailCOM';
                     });
                 
[LegLFHipJoint,LegLFThighJoint,LegLFShankJoint,LegLFFoot,LegLFThighCOM,LegLFShankCOM]...
=System.Space.genNode({
                     'LegLFHipJoint';
                     'LegLFThighJoint';
                     'LegLFShankJoint';
                     'LegLFFoot';
                     'LegLFThighCOM';
                     'LegLFShankCOM';
                     });
                 
[LegRFHipJoint,LegRFThighJoint,LegRFShankJoint,LegRFFoot,LegRFThighCOM,LegRFShankCOM]...
=System.Space.genNode({
                     'LegRFHipJoint';
                     'LegRFThighJoint';
                     'LegRFShankJoint';
                     'LegRFFoot';
                     'LegRFThighCOM';
                     'LegRFShankCOM';
                     });
                 
[LegLRHipJoint,LegLRThighJoint,LegLRShankJoint,LegLRFoot,LegLRThighCOM,LegLRShankCOM]...
=System.Space.genNode({
                     'LegLRHipJoint';
                     'LegLRThighJoint';
                     'LegLRShankJoint';
                     'LegLRFoot';
                     'LegLRThighCOM';
                     'LegLRShankCOM';
                     });
                 
[LegRRHipJoint,LegRRThighJoint,LegRRShankJoint,LegRRFoot,LegRRThighCOM,LegRRShankCOM]...
=System.Space.genNode({
                     'LegRRHipJoint';
                     'LegRRThighJoint';
                     'LegRRShankJoint';
                     'LegRRFoot';
                     'LegRRThighCOM';
                     'LegRRShankCOM';
                     });

%% Declare Links and Form Kinematics
World2Torso=System.Space.genEdge({'World2Torso' 'WORLD' 'TorsoFrame'});
Torso2TailRoll=System.Space.genEdge({'Torso2TailRoll' 'TorsoFrame' 'TailRoll'});
TailRoll2TailYaw=System.Space.genEdge({'TailRoll2TailYaw' 'TailRoll' 'TailYaw'});
TailYaw2TailCOM=System.Space.genEdge({'TailYaw2TailCOM' 'TailYaw' 'TailCOM'});
World2Torso.setAngVel([qrx.dot(1);qry.dot(1);qrz.dot(1)],'s_tr').setTransDis([qtx.dot(0);qty.dot(0);qtz.dot(0)]).genProp;
Torso2TailRoll.setDH([0;-pi/2;C.d_t;-pi/2]).genProp;
TailRoll2TailYaw.setDH([0;q_TR.dot(0);0;pi/2]).genProp;
TailYaw2TailCOM.setDH([0;q_TY.dot(0);C.l_t;0]).genProp;

Torso2LFH=System.Space.genEdge({'Torso2LFH' 'TorsoFrame' 'LegLFHipJoint'});
LFH2LFT=System.Space.genEdge({'LFH2LFT' 'LegLFHipJoint' 'LegLFThighJoint'});
LFT2LFS=System.Space.genEdge({'LFT2LFS' 'LegLFThighJoint' 'LegLFShankJoint'});
LFS2LFF=System.Space.genEdge({'LFS2LFF' 'LegLFShankJoint' 'LegLFFoot'});
LFT2LFTCOM=System.Space.genEdge({'LFT2LFTCOM' 'LegLFThighJoint' 'LegLFThighCOM'});
LFS2LFSCOM=System.Space.genEdge({'LFS2LFSCOM' 'LegLFShankJoint' 'LegLFShankCOM'});
Torso2LFH.setAxang(sym([-pi/2;1;0;0])).setTransDis(0.5*[-C.b_w;C.b_l;0]).genProp;
LFH2LFT.setDH([0;pi/2+q_LF_H.dot(0);0;-pi/2]).genProp;
LFT2LFS.setDH([0;q_LF_T.dot(0);C.l_th;0]).genProp;
LFS2LFF.setDH([0;q_LF_S.dot(0);C.l_sh;0]).genProp;
LFT2LFTCOM.setDH([0;q_LF_T.dot(0);0.5*C.l_th;0]).genProp;
LFS2LFSCOM.setDH([0;q_LF_S.dot(0);0.5*C.l_sh;0]).genProp;

Torso2RFH=System.Space.genEdge({'Torso2RFH' 'TorsoFrame' 'LegRFHipJoint'});
RFH2RFT=System.Space.genEdge({'RFH2RFT' 'LegRFHipJoint' 'LegRFThighJoint'});
RFT2RFS=System.Space.genEdge({'RFT2RFS' 'LegRFThighJoint' 'LegRFShankJoint'});
RFS2RFF=System.Space.genEdge({'RFS2RFF' 'LegRFShankJoint' 'LegRFFoot'});
RFT2RFTCOM=System.Space.genEdge({'RFT2RFTCOM' 'LegRFThighJoint' 'LegRFThighCOM'});
RFS2RFSCOM=System.Space.genEdge({'RFS2RFSCOM' 'LegRFShankJoint' 'LegRFShankCOM'});
Torso2RFH.setAxang(sym([-pi/2;1;0;0])).setTransDis(0.5*[C.b_w;C.b_l;0]).genProp;
RFH2RFT.setDH([0;pi/2-q_RF_H.dot(0);0;-pi/2]).genProp;
RFT2RFS.setDH([0;q_RF_T.dot(0);C.l_th;0]).genProp;
RFS2RFF.setDH([0;q_RF_S.dot(0);C.l_sh;0]).genProp;
RFT2RFTCOM.setDH([0;q_RF_T.dot(0);0.5*C.l_th;0]).genProp;
RFS2RFSCOM.setDH([0;q_RF_S.dot(0);0.5*C.l_sh;0]).genProp;

Torso2LRH=System.Space.genEdge({'Torso2LRH' 'TorsoFrame' 'LegLRHipJoint'});
LRH2LRT=System.Space.genEdge({'LRH2LRT' 'LegLRHipJoint' 'LegLRThighJoint'});
LRT2LRS=System.Space.genEdge({'LRT2LRS' 'LegLRThighJoint' 'LegLRShankJoint'});
LRS2LRF=System.Space.genEdge({'LRS2LRF' 'LegLRShankJoint' 'LegLRFoot'});
LRT2LRTCOM=System.Space.genEdge({'LRT2LRTCOM' 'LegLRThighJoint' 'LegLRThighCOM'});
LRS2LRSCOM=System.Space.genEdge({'LRS2LRSCOM' 'LegLRShankJoint' 'LegLRShankCOM'});
Torso2LRH.setAxang(sym([-pi/2;1;0;0])).setTransDis(0.5*[-C.b_w;-C.b_l;0]).genProp;
LRH2LRT.setDH([0;pi/2+q_LR_H.dot(0);0;-pi/2]).genProp;
LRT2LRS.setDH([0;q_LR_T.dot(0);C.l_th;0]).genProp;
LRS2LRF.setDH([0;q_LR_S.dot(0);C.l_sh;0]).genProp;
LRT2LRTCOM.setDH([0;q_LR_T.dot(0);0.5*C.l_th;0]).genProp;
LRS2LRSCOM.setDH([0;q_LR_S.dot(0);0.5*C.l_sh;0]).genProp;

Torso2RRH=System.Space.genEdge({'Torso2RRH' 'TorsoFrame' 'LegRRHipJoint'});
RRH2RRT=System.Space.genEdge({'RRH2RRT' 'LegRRHipJoint' 'LegRRThighJoint'});
RRT2RRS=System.Space.genEdge({'RRT2RRS' 'LegRRThighJoint' 'LegRRShankJoint'});
RRS2RRF=System.Space.genEdge({'RRS2RRF' 'LegRRShankJoint' 'LegRRFoot'});
RRT2RRTCOM=System.Space.genEdge({'RRT2RRTCOM' 'LegRRThighJoint' 'LegRRThighCOM'});
RRS2RRSCOM=System.Space.genEdge({'RRS2RRSCOM' 'LegRRShankJoint' 'LegRRShankCOM'});
Torso2RRH.setAxang(sym([-pi/2;1;0;0])).setTransDis(0.5*[C.b_w;-C.b_l;0]).genProp;
RRH2RRT.setDH([0;pi/2-q_RR_H.dot(0);0;-pi/2]).genProp;
RRT2RRS.setDH([0;q_RR_T.dot(0);C.l_th;0]).genProp;
RRS2RRF.setDH([0;q_RR_S.dot(0);C.l_sh;0]).genProp;
RRT2RRTCOM.setDH([0;q_RR_T.dot(0);0.5*C.l_th;0]).genProp;
RRS2RRSCOM.setDH([0;q_RR_S.dot(0);0.5*C.l_sh;0]).genProp;

%Generate Kinematic System
System.Space.plotGraph(1);
System.Space.makeNumKinematics();