classdef kCons < kSysObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        Sym;
        Expression;
        DtExpression;
        GFExpression;
        SoftFactor=0;
        
        Jacobian;
        Coriolis;
        GFMatrix=0;
    end
    
    methods(Access=public)
        function obj=kCons(inName,inSys,inMultSym)
            obj@kSysObj(inName,inSys,inSys.Space.RootFrame,inSys.Space.RootFrame);
            if(isa(inMultSym,'char'))
                obj.Sym=sym(inMultSym,'real');
            else
                error(obj.msgStr('Error','Multiplier need to be symbolic variable name'));
            end
            obj.System.Constraint.reg(obj);
        end
        
        function obj=setConsProp(obj,injac,incor,ingf)
            obj.Jacobian=injac;
            obj.Coriolis=incor;
            obj.GFMatrix=ingf;
        end
        
        function [jac,cor,gf]=genConsProp(obj,varargin)
            disp(obj.msgStr('Message',['Generating Constraint Force...']));
            Expr=obj.Expression;
            dtExpr=obj.DtExpression;
            gfExpr=obj.GFExpression;
            dtVec=obj.System.getContVector(1);
            sFactor=obj.SoftFactor;
            
            jac=(jacobian(Expr,dtVec));
            jacBase=jac;
            cor=jacobian(dtExpr,dtVec)*dtVec;
            corBase=cor;
            if(~isempty(obj.InvolvedFrame))
                FrameList=obj.System.Space.getNode(obj.InvolvedFrame);
                FrameVec=sym(zeros(3,numel(obj.InvolvedFrame)));
                FrameQuat=sym(zeros(4,numel(obj.InvolvedFrame)));
                dtFrameVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                
                if strcmp(varargin,'Full')
                    SubVec=sym(zeros(3,numel(obj.InvolvedFrame)));
                    SubQuat=sym(zeros(4,numel(obj.InvolvedFrame)));
                    dtSubVec=sym(zeros(6,numel(obj.InvolvedFrame)));

                    CorVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                    SubCorVec=sym(zeros(6,numel(obj.InvolvedFrame)));
                    FrameVecJac=sym(zeros(numel(obj.InvolvedFrame),6));
                    FrameJacobian=sym(zeros(6,numel(dtVec),numel(obj.InvolvedFrame)));
                end
                
                for ii=1:numel(FrameList)
                    curFrame=FrameList{ii};
                    
                    FrameVec(:,ii)=curFrame.getTransSym(0);
                    dtFrameVec(1:3,ii)=curFrame.getTransSym(1);
                    dtFrameVec(4:6,ii)=curFrame.getAngSym(1);
                    FrameQuat(:,ii)=curFrame.getAngSym(0);
                    CorVec(:,ii)=curFrame.getCorSym();
                    
                    FrameVecJac(ii,:)=jacobian(Expr,dtFrameVec(:,ii));
                    corBase=corBase+jacobian(dtExpr,dtFrameVec(:,ii))*dtFrameVec(:,ii);
                    
                    if strcmp(varargin,'Full')
                        SubVec(:,ii)=curFrame.rootTransDis();
                        dtSubVec(1:3,ii)=curFrame.rootTransVel();
                        dtSubVec(4:6,ii)=curFrame.rootAngVel();
                        SubQuat(:,ii)=curFrame.rootRotQuat();
                        SubCorVec(1:3,ii)=curFrame.rootCorTransAcc();
                        SubCorVec(4:6,ii)=curFrame.rootCorAngAcc();

                        FrameJacobian(1:3,:,ii)=curFrame.rootTransJacobian();
                        FrameJacobian(4:6,:,ii)=curFrame.rootAngJacobian();
                        jac=jac+jacobian(Expr,dtFrameVec(:,ii))*FrameJacobian(:,:,ii);
                        cor=cor+jacobian(dtExpr,dtFrameVec(:,ii))*dtFrameVec(:,ii)+jacobian(Expr,dtFrameVec(:,ii))*SubCorVec(:,ii);
                    end
                end
            end
            
            gf=sym(0);
            if(numel(sFactor)==1)
                for ii=1:numel(gfExpr)
                    gf=gf+sFactor*(gfExpr(ii));
                end
            elseif(numel(sFactor)==numel(gfExpr))
                for ii=1:numel(gfExpr)
                    gf=gf+sFactor(ii)*(gfExpr(ii));
                end
            else
                error(obj.msgStr('Error','Holonomic Soft Constraint Parameter should be Uniform or Match in Number!'));
            end
            
%             obj.CompileInfo.Jacobian=jac;
%             obj.CompileInfo.Coriolis=cor;
%             obj.CompileInfo.GFFactor=gf;
            gfBase=gf;
            obj.CompileInfo.Jacobian=jacBase;
            obj.CompileInfo.Coriolis=corBase;
            obj.CompileInfo.GFFactor=gfBase;
            
            gf=-jac.'*gf;
            
            TempVec=[];
            if(~isempty(obj.InvolvedFrame))            
                for ii=1:numel(obj.InvolvedFrame)
                    TempVec=[TempVec [FrameVec(:,ii);dtFrameVec(:,ii);FrameQuat(:,ii)]];
                    if strcmp(varargin,'Full')
                        jac=subs(jac,[FrameVec(:,ii);dtFrameVec(:,ii);FrameQuat(:,ii)],[SubVec(:,ii);dtSubVec(:,ii);SubQuat(:,ii)]);
                        cor=subs(cor,[FrameVec(:,ii);dtFrameVec(:,ii);FrameQuat(:,ii)],[SubVec(:,ii);dtSubVec(:,ii);SubQuat(:,ii)]);
                        gf=subs(gf,[FrameVec(:,ii);dtFrameVec(:,ii);FrameQuat(:,ii)],[SubVec(:,ii);dtSubVec(:,ii);SubQuat(:,ii)]);
                    end
                end
            end
            
            obj.CompileInfo.InvolvedFrame=0;
            obj.CompileInfo.SubsVec=sym('SUBSVECTOR__');
            obj.CompileInfo.FrameVecJacobian=sym(0);
            if ~isempty(obj.InvolvedFrame)
                obj.CompileInfo.InvolvedFrame=obj.InvolvedFrame;
                obj.CompileInfo.SubsVec=TempVec;
                obj.CompileInfo.FrameVecJacobian=FrameVecJac;
            end
            
            obj.Jacobian=jac;
            obj.Coriolis=cor;
            obj.GFMatrix=gf;
        end
        
        function obj=setNHCons(obj,inExpr,varargin)
            if isa(inExpr,'sym')&&(numel(inExpr)==1)
                obj.Expression=inExpr;
            else
                error(obj.msgStr('Error','NonHolonomic Constraint Need to be scalar <sym>!'));
            end
            
            if nargin==3
                obj.SoftFactor=varargin{1};
            end
            
            if nargin==4
                obj.GFExpression=[obj.GFExpression;varargin{1}];
                obj.SoftFactor=varargin{2};
            end
            
            obj.setInvolvedFrame(unique([symvar(obj.GFExpression)]));
        end
        
        function obj=setHCons(obj,inExpr,varargin)
            if isa(inExpr,'sym')&&(numel(inExpr)==1)
                obj.Expression=inExpr;
                obj.GFExpression=inExpr;
            else
                error(obj.msgStr('Error','NonHolonomic Constraint Need to be scalar <sym>!'));
            end
            
            obj.setInvolvedFrame(unique(symvar(inExpr)));
            if(~isempty(obj.InvolvedFrame))
                FrameList=obj.System.Space.getNode(obj.InvolvedFrame);
                for ii=1:numel(FrameList)
                    curFrame=FrameList{ii};
                    curFrameQuat=curFrame.getAngSym(0);
                    curFrameAngVel=curFrame.getAngSym(1);
                    
                    obj.GFExpression=subs(obj.GFExpression,pDiff(curFrameQuat,obj.System.TimeVar),0.5*quatMultiply([0;curFrameAngVel],curFrameQuat));
                end
            end
            %This therefore assumes that quaternion 2nd derivative or higher are not supported
            
            [NHSym,NHExpr]=obj.System.getNHSignalVector();
            NHdt=[];
            if ~isempty(NHSym)
                NHdt=pDiff(NHSym,obj.System.TimeVar);
            end
            
            obj.Expression=(subs(pDiff(obj.GFExpression,obj.System.TimeVar),NHdt,NHExpr));
            if(~isempty(obj.InvolvedFrame))
                FrameList=obj.System.Space.getNode(obj.InvolvedFrame);
                for ii=1:numel(FrameList)
                    curFrame=FrameList{ii};
                    curFrameQuat=curFrame.getAngSym(0);
                    curFrameAngVel=curFrame.getAngSym(1);
                    
                    obj.Expression=subs(obj.Expression,pDiff(curFrameQuat,obj.System.TimeVar),0.5*quatMultiply([0;curFrameAngVel],curFrameQuat));
                end
            end
            obj.GFExpression=[obj.GFExpression;obj.Expression]; % Double the Damping Effect
            
            if nargin==3
                obj.SoftFactor=varargin{1};
            end
            
            if nargin==4
                obj.GFExpression=[obj.GFExpression;varargin{1}];
                obj.SoftFactor=varargin{2};
            end
            
            obj.DtExpression=(subs(pDiff(obj.Expression,obj.System.TimeVar),NHdt,NHExpr));
            if(~isempty(obj.InvolvedFrame))
                FrameList=obj.System.Space.getNode(obj.InvolvedFrame);
                for ii=1:numel(FrameList)
                    curFrame=FrameList{ii};
                    curFrameQuat=curFrame.getAngSym(0);
                    curFrameAngVel=curFrame.getAngSym(1);
                    
                    obj.DtExpression=subs(obj.DtExpression,pDiff(curFrameQuat,obj.System.TimeVar),0.5*quatMultiply([0;curFrameAngVel],curFrameQuat));
                end
            end
        end
    end
end
