classdef sAxes<jObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        FigHandle;
        AxesHandle;
        
        PresetTFChain;
        OriginTFChain;
        
        AxesShift=[0,0,0];
        AxesRange=[-0.25,-0.25,-0.25;0.25,0.25,0.25];
        AxesView=[135,45];
        
        Frame;
        TFMap;
        TFFunc;
        
        Plot=[];
        Patch=[];
    end
    
    methods(Access=public)
        function obj=sAxes(InName,FigNumber,FrameList,TFMapFunc)
            obj@jObj(InName);
            obj.FigHandle=figure(FigNumber);
            obj.AxesHandle=axes;
            title(obj.AxesHandle,InName);
            camlight('headlight');
            material('shiny');
            tf=load(FrameList);
            obj.Frame=jDic(obj.tagStr('Parameter'),'double');
            obj.Frame=obj.Frame.reg(tf.TFPath(:,1),tf.TFPath(:,2));
            obj.TFFunc=TFMapFunc;
            obj.genPatch({'WORLD_FRAME','WORLD'});
            obj.OriginTFChain=obj.Frame.get('WORLD');
            obj.PresetTFChain=obj.Frame.get('WORLD');
            obj.setPOV();
        end
        
        function obj=setPresetTF(obj,inFrame)
            obj.PresetTFChain=obj.Frame.get(inFrame);
        end
        
        function varargout=genPatch(obj,inList)
            if(numel(inList(1,:))==2)
                varargout=cell(1,numel(inList(:,1)));
                for ii=1:numel(inList(:,1))
                    newPatch=sPatch(inList{ii,1},obj,inList{ii,2});
                    obj.Patch=[obj.Patch;newPatch];
                    varargout{ii}=newPatch;
                end
            else
                error(obj.msgStr('Error','Input should be {patchName,patchFrame;...}!'));
            end
        end
        
        function varargout=genPlot(obj,inList)
            if(numel(inList(1,:))==1)
                varargout=cell(1,numel(inList(:,1)));
                for ii=1:numel(inList(:,1))
                    newPlot=sPlot(inList{ii,1},obj);
                    obj.Plot=[obj.Plot;newPlot];
                    varargout{ii}=newPlot;
                end
            else
                error(obj.msgStr('Error','Input should be {plotName;...}!'));
            end
        end
        
        function obj=setAxesProp(obj,inOrigin,inRange,inView)
            if(~isempty(inOrigin))
                obj.OriginTFChain=obj.Frame.get(inOrigin);
            end
            
            if(~isempty(inRange))
                if isequal(size(inRange),[2 3])
                    obj.AxesRange=inRange;
                else
                    error(obj.msgStr('Error','Axis Range should be 2X3 Matrix'));
                end
            end
            
            if(~isempty(inView))
                if isequal(size(inView),[1 2])
                    obj.AxesView=inView;
                else
                    error(obj.msgStr('Error','View Angle should be 1X2 Vector'));
                end
            end
            obj.AxesHandle.set('View',obj.AxesView);
        end
        
        function obj=setPOV(obj)
            AxesLimit=obj.AxesRange+[obj.AxesShift;obj.AxesShift];
            obj.AxesHandle.set('XLim',AxesLimit(:,1).');
            obj.AxesHandle.set('YLim',AxesLimit(:,2).');
            obj.AxesHandle.set('ZLim',AxesLimit(:,3).');
            obj.AxesHandle.set('DataAspectRatio',[1 1 1]);
        end
        
        function obj=drawNow(obj,t,q,p,u,s)
            obj.TFMap=obj.TFFunc(t,q,p,u,s);
            
            for ii=1:numel(obj.Patch)
                obj.Patch(ii).drawPatch(t,q,p,u,s);
            end
            
            for ii=1:numel(obj.Plot)
                obj.Plot(ii).drawPlot(t,q,p,u,s);
            end
            
            camTF=obj.getTF(obj.OriginTFChain,t,q,p,u,s);
            obj.AxesShift=camTF(1:3,4).';
            obj.setPOV();
            
            drawnow('limitrate');
        end
        
        function TF=getTF(obj,TFChain,t,q,p,u,s)
%             TF=eye(4,4);
%             for ii=numel(TFChain):-1:2
%                 TF=(obj.TFMap(TFChain(ii),t,q,p,u,s)*TF);
%             end
            TF=obj.TFMap(:,:,TFChain(end));
            
%             PreTF=eye(4,4);
%             for ii=numel(obj.PresetTFChain):-1:2
%                 PreTF=(obj.TFMap(obj.PresetTFChain(ii),t,q,p,u,s)*PreTF);
%             end
            PreTF=obj.TFMap(:,:,obj.PresetTFChain(end));
            TF=PreTF\TF;
        end
    end
end
