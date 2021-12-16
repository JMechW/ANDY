classdef jEdge < jObj
% jEdge is the object that contains the properties of an edge in a jNet.

% Author: Jiamin Wang; Updated: 2021-12-15;


    properties(SetAccess=protected)
        EdgeNumber; %The edge number in the network (constructed of the two node numbers)
        Net; % The network of the edge
        
        StartNode; % The node that the edge start from
        EndNode; % The node that the edge end into
    end
    
    methods(Access=public)
        function obj=jEdge(inName,startNode,endNode)
            %Constructor of jEdge. [Original to jEdge]
            %obj=jEdge(inName,startNode,endNode)
            %<startNode>: The jNode that the edge start from
            %<endNode>: The jNode that the edge end into
            obj@jObj(inName)
            if(isa(startNode,'jNode')&&isa(endNode,'jNode'))
                if startNode.netCheck(endNode)
                    obj.StartNode=startNode;
                    obj.EndNode=endNode;
                    startNode.StartEdge.reg(endNode.NameTag,obj);
                    endNode.EndEdge.reg(startNode.NameTag,obj);
                    obj.Net=startNode.Net;
                    obj.EdgeNumber=[startNode.NodeNumber,endNode.NodeNumber];
                end
            end
        end
    end
end
