classdef sPlot<jObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        Axes;
        Handle;
        
        PlotPoint={};
        PlotData=zeros(3,1);
        
        IsTimed=false;
        TimeCounter=0;
    end
    
    methods(Access=public)
        function obj=sPlot(inName,inAxes)
            obj@jObj(inName);
            obj.Axes=inAxes;
            hold(obj.Axes.AxesHandle,'on');
            obj.Handle=plot3(obj.Axes.AxesHandle,obj.PlotData(1,:),obj.PlotData(2,:),obj.PlotData(3,:));
            hold(obj.Axes.AxesHandle,'off');
            obj.PlotPoint{1}=obj.Axes.Frame.get('WORLD');
        end
        
        function obj=setLineSpec(obj,inLineStyle,inMarkerStyle,inLineColor,inLineWidth)
            obj.Handle.set('LineStyle',inLineStyle);
            obj.Handle.set('Marker',inMarkerStyle);
            obj.Handle.set('Color',inLineColor);
            obj.Handle.set('LineWidth',inLineWidth);
            obj.Handle.set('MarkerSize',inLineWidth*2);
        end
        
        function obj=setPlotPoint(obj,inList)
            obj.PlotPoint=obj.Axes.Frame.get(inList);
            obj.IsTimed=false;
            obj.PlotData=zeros(3,numel(obj.PlotPoint));
        end
        
        function obj=setTimedPlotPoint(obj,inPoint)
            obj.PlotPoint=obj.Axes.Frame.get(inPoint);
            obj.IsTimed=true;
            obj.TimeCounter=0;
        end
        
        function obj=clearTimedPlot(obj)
            if obj.IsTimed
                obj.PlotData=zeros(3,1);
                obj.TimeCounter=0;
            end
        end
        
        function obj=drawPlot(obj,t,q,p,u,s)
            if obj.IsTimed
                obj.TimeCounter=obj.TimeCounter+1;
                TF=obj.Axes.getTF(obj.PlotPoint,t,q,p,u,s);
                obj.PlotData(:,obj.TimeCounter)=TF(1:3,4);
            else
                for ii=1:numel(obj.PlotPoint)
                    CurTF=obj.Axes.getTF(obj.PlotPoint{ii},t,q,p,u,s);
                    obj.PlotData(:,ii)=CurTF(1:3,4);
                end
            end
            obj.Handle.set('XData',obj.PlotData(1,:));
            obj.Handle.set('YData',obj.PlotData(2,:));
            obj.Handle.set('ZData',obj.PlotData(3,:));
        end
    end
end
