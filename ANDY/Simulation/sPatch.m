classdef sPatch < jObj
% Author: Jiamin Wang; Updated: 2021-12-15;

    properties(SetAccess=protected)
        Axes;
        Handle;
        
        Model;
        TFChain;
        Offset=eye(4,4);
    end
    
    methods(Access=public)
        function obj=sPatch(InName,InAxes,InFrame)
            obj@jObj(InName);
            obj.Axes=InAxes;
            obj.TFChain=obj.Axes.Frame.get(InFrame);
            
            framePatch=load('XYZFramePatch.mat');
            obj.Model=framePatch.XYZFrame;
            hold(obj.Axes.AxesHandle,'on');
            obj.Handle=patch(obj.Axes.AxesHandle,obj.Model,...
                             'FaceLighting','gouraud',...
                             'AmbientStrength', 0.5,...
                            'FaceColor',[0.8 0.8 1],...
                            'FaceAlpha',1,...
                            'EdgeColor','none');
                            
            hold(obj.Axes.AxesHandle,'off');
        end
        
        function obj=setFaceProp(obj,inColor,inAlpha)
            if ~isempty(inColor)
                obj.Handle.set('FaceColor',inColor);
            end
            if ~isempty(inAlpha)
                obj.Handle.set('FaceAlpha',inAlpha);
            end
        end
        
        function obj=setModel(obj,inStl,inSuppress,inOffset,inScale)
            if isempty(inSuppress)
                inSuppress=1;
            end
            if isempty(inStl)
                framePatch=load('XYZFramePatch.mat');
                obj.Model=framePatch.XYZFrame;
            else
                stlModel=reducepatch(stlread(inStl),abs(inSuppress));
                obj.Model.Vertices=stlModel.vertices;
                obj.Model.Faces=stlModel.faces;
            end
            if isempty(inOffset)
                inOffset=eye(4,4);
            end
            if isempty(inScale)
                inScale=1;
            end
            
            obj.Model.Vertices=(obj.Model.Vertices)*inScale;
            obj.Offset=inOffset;
            obj.Handle.set('Vertices',obj.Model.Vertices);
            obj.Handle.set('Faces',obj.Model.Faces);
        end
        
        function obj=drawPatch(obj,t,q,p,u,s)
            TF=obj.Axes.getTF(obj.TFChain,t,q,p,u,s)*obj.Offset;
            CurVertices=obj.Model.Vertices*TF(1:3,1:3).'+TF(1:3,4).';
            obj.Handle.set('Vertices',CurVertices);
        end
    end
end
