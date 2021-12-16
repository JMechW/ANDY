classdef kBody < kSysObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    % 
    properties(SetAccess=protected)
        Mass=0; %
        Moment=zeros(3,3);
        
        Force;
        Torque;
        
        GFMatrix=0;
        CorMat=0;
        CenMat=0;
        InertMatrix=0;
    end
    
    methods(Access=public)
        function obj=kBody(inName,inSys,inFrame)
            obj@kSysObj(inName,inSys,inFrame,inFrame);
            obj.System.Body.reg(obj);
            
            obj.Force=jDic(obj.tagStr('Force'),'kForce');
            obj.Torque=jDic(obj.tagStr('Torque'),'kTorque');
        end
        
        
        function varargout=genForce(obj,inForceList)
            if ~(isa(inForceList,'cell'))
                error(obj.msgStr('Error','Input list of body should be cell array!'));
            end
            
            ListSize=size(inForceList);
            if ListSize(2)~=3
                error(obj.msgStr('Error','Input list of body should follow format of {name ActionFrame DirectionRefFrame;...}!'));
            end
            
            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)                
                varargout{ii}=kForce(inForceList{ii,1},obj,inForceList{ii,2},inForceList{ii,3});
            end
        end
        
        function varargout=genTorque(obj,inTorqueList)
            if ~(isa(inTorqueList,'cell'))
                error(obj.msgStr('Error','Input list of body should be cell array!'));
            end

            ListSize=size(inTorqueList);
            if ListSize(2)~=3
                error(obj.msgStr('Error','Input list of body should follow format of {name ActionFrame DirectionRefFrame;...}!'));
            end

            varargout=cell(ListSize(1),1);
            for ii=1:ListSize(1)                
                varargout{ii}=kTorque(inTorqueList{ii,1},obj,inTorqueList{ii,2},inTorqueList{ii,3});
            end
        end
        
        function obj=setProp(obj,varargin)
            if nargin==2
                inMass=varargin{1};
            elseif nargin==3
                inMass=varargin{1};
                inMoment=varargin{2};
            else
                error(obj.msgStr('Error','Input should be Mass and Moment(optional)!'));
            end
                
            if (isa(inMass,'sym')&&(numel(inMass)==1))
                if (isequal(obj.Mass,sym(0)))
                    error(obj.msgStr('Error','Mass cannot be zero!'));
                else
                    obj.Mass=inMass;
                end
            else
                error(obj.msgStr('Error','The Mass need to be a 1*1 sym!'));
            end
            
            if nargin==3
                if (isa(inMoment,'sym')&&(isequal(size(inMoment),[3,3])))
                    obj.Moment=inMoment;
                else
                    error(obj.msgStr('Error','The Moment need to be a 3*3 sym!'));
                end
            end
        end
        
        function obj=setInertia(obj,inM,inGF)
            obj.InertMatrix=inM;
            obj.GFMatrix=inGF;
        end
        
        function obj=compile(obj)
            obj.CompileInfo.BaseFrame=obj.BaseFrame.NodeNumber;
            obj.CompileInfo.Mass=obj.Mass;
            obj.CompileInfo.Moment=obj.Moment;
        end
        
        function [MMat,FMat,CorMat,CenMat]=genInertia(obj,varargin)
            disp(obj.msgStr('Message',['Generating Inertia Matrix and Coriolis Force...']));
            
            
            dtVec=obj.System.getContVector(1);
            
            MMat=eye(numel(dtVec));
            FMat=zeros(numel(dtVec),1);
            CorMat=zeros(numel(dtVec),numel(dtVec));
            CenMat=zeros(numel(dtVec),numel(dtVec));
            
            if strcmp(varargin,'Full')
                rotor=obj.BaseFrame.rootRotMat();
                m=obj.Mass;
                I=rotor*obj.Moment*rotor.';
                w=obj.BaseFrame.rootAngVel;
                a=obj.BaseFrame.rootCorTransAcc;
                alpha=obj.BaseFrame.rootCorAngAcc;

                vJacobian=obj.BaseFrame.rootTransJacobian;
                wJacobian=obj.BaseFrame.rootAngJacobian;

                MMat=vJacobian.'*m*vJacobian+wJacobian.'*I*wJacobian;
                FMat=-vJacobian.'*m*a-wJacobian.'*(I*alpha+cross(w,I*w));
                CorMat= vJacobian.'*m*pDiff(vJacobian,obj.System.TimeVar) + wJacobian.'*I*pDiff(wJacobian,obj.System.TimeVar);
                CenMat= - wJacobian.'* skew3(I*w) * wJacobian;

                obj.InertMatrix=MMat;
                obj.GFMatrix=FMat;
                obj.CorMat=CorMat;
                obj.CenMat=CenMat;
                
            end
            
            obj.CompileInfo.Mass=obj.Mass;
            obj.CompileInfo.Moment=obj.Moment;
            obj.CompileInfo.BaseFrame=obj.BaseFrame.NodeNumber;
        end
        
        function output=genKineticEnergy(obj)
            m=obj.Mass;
            I=obj.Moment;
            v=obj.BaseFrame.rootTransVel;
            w=obj.BaseFrame.rootAngVel;
            
            output=0.5*((v.')*m*v+(w.')*I*w);
        end
    end
end
