%% Declare Body
[Torso,Tail,LFThigh,LFShank,RFThigh,RFShank,LRThigh,LRShank,RRThigh,RRShank]...
=System.genBody({
            'Torso' 'TorsoFrame';
            'Tail' 'TailCOM';
            'LFThigh' 'LegLFThighCOM';
            'LFShank' 'LegLFShankCOM';
            'RFThigh' 'LegRFThighCOM';
            'RFShank' 'LegRFShankCOM';
            'LRThigh' 'LegLRThighCOM';
            'LRShank' 'LegLRShankCOM';
            'RRThigh' 'LegRRThighCOM';
            'RRShank' 'LegRRShankCOM';
            });
Torso.setProp(C.m_b,diag([C.I_bx;C.I_by;C.I_bz])).genForce({'GTorso' TorsoFrame WORLD}).setProp([0;0;-C.m_b*C.g]);
Tail.setProp(C.m_t).genForce({'GTail' TailCOM WORLD}).setProp([0;0;-C.m_t*C.g]);
LFThigh.setProp(C.m_th,diag((1/12*(C.m_th*C.l_th))*[0;1;1])).genForce({'GLFThigh' LegLFThighCOM WORLD}).setProp([0;0;-C.m_th*C.g]);
LFShank.setProp(C.m_sh,diag((1/12*(C.m_sh*C.l_sh))*[0;1;1])).genForce({'GLFShank' LegLFShankCOM WORLD}).setProp([0;0;-C.m_sh*C.g]);
RFThigh.setProp(C.m_th,diag((1/12*(C.m_th*C.l_th))*[0;1;1])).genForce({'GRFThigh' LegRFThighCOM WORLD}).setProp([0;0;-C.m_th*C.g]);
RFShank.setProp(C.m_sh,diag((1/12*(C.m_sh*C.l_sh))*[0;1;1])).genForce({'GRFShank' LegRFShankCOM WORLD}).setProp([0;0;-C.m_sh*C.g]);
LRThigh.setProp(C.m_th,diag((1/12*(C.m_th*C.l_th))*[0;1;1])).genForce({'GLRThigh' LegLRThighCOM WORLD}).setProp([0;0;-C.m_th*C.g]);
LRShank.setProp(C.m_sh,diag((1/12*(C.m_sh*C.l_sh))*[0;1;1])).genForce({'GLRShank' LegLRShankCOM WORLD}).setProp([0;0;-C.m_sh*C.g]);
RRThigh.setProp(C.m_th,diag((1/12*(C.m_th*C.l_th))*[0;1;1])).genForce({'GRRThigh' LegRRThighCOM WORLD}).setProp([0;0;-C.m_th*C.g]);
RRShank.setProp(C.m_sh,diag((1/12*(C.m_sh*C.l_sh))*[0;1;1])).genForce({'GRRShank' LegRRShankCOM WORLD}).setProp([0;0;-C.m_sh*C.g]);

%% Declare Torques
LFThigh.genTorque({'TLFHip' LegLFThighCOM LegLFHipJoint}).setProp(u_LF.u_LF_H*[0;0;1]);
Torso.genTorque({'CounterTLFHip' TorsoFrame LegLFHipJoint}).setProp(u_LF.u_LF_H*[0;0;-1]);
LFThigh.genTorque({'TLFThigh' LegLFThighCOM LegLFThighJoint}).setProp(u_LF.u_LF_T*[0;0;1]);
Torso.genTorque({'CounterTLFThigh' TorsoFrame LegLFThighJoint}).setProp(u_LF.u_LF_T*[0;0;-1]);
LFShank.genTorque({'TLFShank' LegLFShankCOM LegLFShankJoint}).setProp(u_LF.u_LF_S*[0;0;1]);
LFThigh.genTorque({'CounterTLFShank' LegLFThighCOM LegLFShankJoint}).setProp(u_LF.u_LF_S*[0;0;-1]);

RFThigh.genTorque({'TRFHip' LegRFThighCOM LegRFHipJoint}).setProp(u_RF.u_RF_H*[0;0;-1]);%Note the Reverse Sign
Torso.genTorque({'CounterTRFHip' TorsoFrame LegRFHipJoint}).setProp(u_RF.u_RF_H*[0;0;1]);
RFThigh.genTorque({'TRFThigh' LegRFThighCOM LegRFThighJoint}).setProp(u_RF.u_RF_T*[0;0;1]);
Torso.genTorque({'CounterTRFThigh' TorsoFrame LegRFThighJoint}).setProp(u_RF.u_RF_T*[0;0;-1]);
RFShank.genTorque({'TRFShank' LegRFShankCOM LegRFShankJoint}).setProp(u_RF.u_RF_S*[0;0;1]);
RFThigh.genTorque({'CounterTRFShank' LegRFThighCOM LegRFShankJoint}).setProp(u_RF.u_RF_S*[0;0;-1]);

LRThigh.genTorque({'TLRHip' LegLRThighCOM LegLRHipJoint}).setProp(u_LR.u_LR_H*[0;0;1]);
Torso.genTorque({'CounterTLRHip' TorsoFrame LegLRHipJoint}).setProp(u_LR.u_LR_H*[0;0;-1]);
LRThigh.genTorque({'TLRThigh' LegLRThighCOM LegLRThighJoint}).setProp(u_LR.u_LR_T*[0;0;1]);
Torso.genTorque({'CounterTLRThigh' TorsoFrame LegLRThighJoint}).setProp(u_LR.u_LR_T*[0;0;-1]);
LRShank.genTorque({'TLRShank' LegLRShankCOM LegLRShankJoint}).setProp(u_LR.u_LR_S*[0;0;1]);
LRThigh.genTorque({'CounterTLRShank' LegLRThighCOM LegLRShankJoint}).setProp(u_LR.u_LR_S*[0;0;-1]);

RRThigh.genTorque({'TRRHip' LegRRThighCOM LegRRHipJoint}).setProp(u_RR.u_RR_H*[0;0;-1]);%Note the Reverse Sign
Torso.genTorque({'CounterTRRHip' TorsoFrame LegRRHipJoint}).setProp(u_RR.u_RR_H*[0;0;1]);
RRThigh.genTorque({'TRRThigh' LegRRThighCOM LegRRThighJoint}).setProp(u_RR.u_RR_T*[0;0;1]);
Torso.genTorque({'CounterTRRThigh' TorsoFrame LegRRThighJoint}).setProp(u_RR.u_RR_T*[0;0;-1]);
RRShank.genTorque({'TRRShank' LegRRShankCOM LegRRShankJoint}).setProp(u_RR.u_RR_S*[0;0;1]);
RRThigh.genTorque({'CounterTRRShank' LegRRThighCOM LegRRShankJoint}).setProp(u_RR.u_RR_S*[0;0;-1]);

Tail.genTorque({'TTailRoll' TailCOM TailRoll}).setProp(u_T.u_TR*[0;0;1]);
Torso.genTorque({'CounterTTailRoll' TorsoFrame TailRoll}).setProp(u_T.u_TR*[0;0;-1]);
Tail.genTorque({'TTailYaw' TailCOM TailYaw}).setProp(u_T.u_TY*[0;0;1]);
Torso.genTorque({'CounterTTailYaw' TorsoFrame TailYaw}).setProp(u_T.u_TY*[0;0;-1]);

%% Declare Damper
DOF=numel(System.getContVector(1));
System.genDamper({'SystemDamper' DOF 1})...
.setProp(sym(u_SYS.u_b*eye(DOF,DOF))...
        ,System.getContVector(1));
    
%% Declare Constraint
[LFFootLockX,LFFootLockY,LFFootLockZ]...
=System.genCons({'LFFootLockX' 'l_LFx';
                 'LFFootLockY' 'l_LFy';
                 'LFFootLockZ' 'l_LFz';
               });
LFFixFootPointExpr=jSimplify(LegLFFoot.getTransSym(0)-[p_LF.p_LF_X;p_LF.p_LF_Y;0]);
LFFootLockX.setHCons([1 0 0]*LFFixFootPointExpr,u_SYS.u_c);
LFFootLockY.setHCons([0 1 0]*LFFixFootPointExpr,u_SYS.u_c);
LFFootLockZ.setHCons([0 0 1]*LFFixFootPointExpr,u_SYS.u_c);

[RFFootLockX,RFFootLockY,RFFootLockZ]...
=System.genCons({'RFFootLockX' 'l_RFx';
                 'RFFootLockY' 'l_RFy';
                 'RFFootLockZ' 'l_RFz';
               });
RFFixFootPointExpr=jSimplify(LegRFFoot.getTransSym(0)-[p_RF.p_RF_X;p_RF.p_RF_Y;0]);
RFFootLockX.setHCons([1 0 0]*RFFixFootPointExpr,u_SYS.u_c);
RFFootLockY.setHCons([0 1 0]*RFFixFootPointExpr,u_SYS.u_c);
RFFootLockZ.setHCons([0 0 1]*RFFixFootPointExpr,u_SYS.u_c);

[LRFootLockX,LRFootLockY,LRFootLockZ]...
=System.genCons({'LRFootLockX' 'l_LRx';
                 'LRFootLockY' 'l_LRy';
                 'LRFootLockZ' 'l_LRz';
               });
LRFixFootPointExpr=jSimplify(LegLRFoot.getTransSym(0)-[p_LR.p_LR_X;p_LR.p_LR_Y;0]);
LRFootLockX.setHCons([1 0 0]*LRFixFootPointExpr,u_SYS.u_c);
LRFootLockY.setHCons([0 1 0]*LRFixFootPointExpr,u_SYS.u_c);
LRFootLockZ.setHCons([0 0 1]*LRFixFootPointExpr,u_SYS.u_c);

[RRFootLockX,RRFootLockY,RRFootLockZ]...
=System.genCons({'RRFootLockX' 'l_RRx';
                 'RRFootLockY' 'l_RRy';
                 'RRFootLockZ' 'l_RRz';
               });
RRFixFootPointExpr=jSimplify(LegRRFoot.getTransSym(0)-[p_RR.p_RR_X;p_RR.p_RR_Y;0]);
RRFootLockX.setHCons([1 0 0]*RRFixFootPointExpr,u_SYS.u_c);
RRFootLockY.setHCons([0 1 0]*RRFixFootPointExpr,u_SYS.u_c);
RRFootLockZ.setHCons([0 0 1]*RRFixFootPointExpr,u_SYS.u_c);
