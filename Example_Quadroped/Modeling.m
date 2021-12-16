PathSetup;
PathSetup;

%% Declare Time Variable and System
t=sym('t','real');
System=kSystem('QuadrupedWithTail',t);
WORLD=System.Space.RootFrame;
System.Space.setPrecompile(true);
System.Model.setPrecompile(true);

%% Declare Constant Parameters
C=System.genParam({
             'l_sh' 'Leg Shank Length' 0.3;
             'l_th' 'Leg Thigh Length' 0.3;
             'l_t' 'Tail Length' 0.6;
             'b_w' 'Torso Joint Width' 0.48;
             'b_l' 'Torso Joint Length' 0.54;
             'd_t' 'Tail Distance for Torso Center' 0.34;
             'm_sh' 'Shank Mass' 1.2289;
             'm_th' 'Thigh Mass' 0.9453;
             'm_b' 'Torso Mass' 26.9078;
             'm_t' 'Tail Mass' 1.4347;
             'I_bx' 'Torso Moment X' 0.8404;
             'I_by' 'Torso Moment Y' 0.3793;
             'I_bz' 'Torso Moment Z' 1.2125;
             'g' 'Gravity Acceleration' 9.8067;
             });
         
%% Declare Continuous States (Generalized Coordinates) 
[qtx,qty,qtz,qrx,qry,qrz]...
=System.genCont({
            'qtx' 'Body X Translational Displacement';
            'qty' 'Body Y Translational Displacement';
            'qtz' 'Body Z Translational Displacement';
            'qrx' 'Body X Rotary Displacement';
            'qry' 'Body Y Rotary Displacement';
            'qrz' 'Body Z Rotary Displacement';
            });
[q_LF_H,q_LF_T,q_LF_S]...
=System.genCont({
            'q_LF_H' 'Left Fore Leg Hip Joint Angle';
            'q_LF_T' 'Left Fore Leg Thigh Joint Angle';
            'q_LF_S' 'Left Fore Leg Shank Joint Angle';
            });
[q_RF_H,q_RF_T,q_RF_S]...
=System.genCont({
            'q_RF_H' 'Right Fore Leg Hip Joint Angle';
            'q_RF_T' 'Right Fore Leg Thigh Joint Angle';
            'q_RF_S' 'Right Fore Leg Shank Joint Angle';
            });
[q_LR_H,q_LR_T,q_LR_S]...
=System.genCont({
            'q_LR_H' 'Left Rear Leg Hip Joint Angle';
            'q_LR_T' 'Left Rear Leg Thigh Joint Angle';
            'q_LR_S' 'Left Rear Leg Shank Joint Angle';
            });
[q_RR_H,q_RR_T,q_RR_S]...
=System.genCont({
            'q_RR_H' 'Right Rear Leg Hip Joint Angle';
            'q_RR_T' 'Right Rear Leg Thigh Joint Angle';
            'q_RR_S' 'Right Rear Leg Shank Joint Angle';
            });
[q_TR,q_TY]...
=System.genCont({
            'q_TR' 'Tail Pitch Angle';
            'q_TY' 'Tail Yaw Angle';
            });
        
%% Declare Discrete States
[p_LF]...
=System.genDisc({
        'p_LF_X' 'Left Fore Foot Contact Point X';
        'p_LF_Y' 'Left Fore Foot Contact Point Y';
        });
    
[p_RF]...
=System.genDisc({
        'p_RF_X' 'Right Fore Foot Contact Point X';
        'p_RF_Y' 'Right Fore Foot Contact Point Y';
        });
    
[p_LR]...
=System.genDisc({
        'p_LR_X' 'Left Rear Foot Contact Point X';
        'p_LR_Y' 'Left Rear Foot Contact Point Y';
        });
    
[p_RR]...
=System.genDisc({
        'p_RR_X' 'Right Rear Foot Contact Point X';
        'p_RR_Y' 'Right Rear Foot Contact Point Y';
        });
    
u_SYS=System.genDisc({
        'u_b' 'Universal Damper for the Whole System';
        'u_c' 'Universal Soft Constraint Factor for the Whole System';
        });
    
%% Declare Inputs
[u_LF]...
=System.genInput({
            'u_LF_H' 'Left Fore Leg Hip Joint Torque';
            'u_LF_T' 'Left Fore Leg Thigh Joint Torque';
            'u_LF_S' 'Left Fore Leg Shank Joint Torque';
            });
[u_RF]...
=System.genInput({
            'u_RF_H' 'Right Fore Leg Hip Joint Torque';
            'u_RF_T' 'Right Fore Leg Thigh Joint Torque';
            'u_RF_S' 'Right Fore Leg Shank Joint Torque';
            });
[u_LR]...
=System.genInput({
            'u_LR_H' 'Left Rear Leg Hip Joint Torque';
            'u_LR_T' 'Left Rear Leg Thigh Joint Torque';
            'u_LR_S' 'Left Rear Leg Shank Joint Torque';
            });
[u_RR]...
=System.genInput({
            'u_RR_H' 'Right Rear Leg Hip Joint Torque';
            'u_RR_T' 'Right Rear Leg Thigh Joint Torque';
            'u_RR_S' 'Right Rear Leg Shank Joint Torque';
            });
[u_T]...
=System.genInput({
            'u_TR' 'Tail Pitch Torque';
            'u_TY' 'Tail Yaw Torque';
            });
        
Modeling_Kinematics;%Kinematic Elements
Modeling_Dynamics;%Dynamic Elements

%Modeling Generation Initialization
System.Model.Init();
[FloatInAir,FixAllFoot]=System.Model.genNode({'FloatInAir';'FixAllFoot'});
System.Model.plotGraph(2);
FloatInAir.genFlowEOM({});
FixAllFoot.genFlowEOM({'LFFootLockX';'LFFootLockY';'LFFootLockZ';...
                        'RFFootLockX';'RFFootLockY';'RFFootLockZ';...
                        'LRFootLockX';'LRFootLockY';'LRFootLockZ';...
                        'RRFootLockX';'RRFootLockY';'RRFootLockZ';...
                       });
System.Model.makeDynamics('num');
System.Model.makeSystemDynamicsForControl(2,'num');
