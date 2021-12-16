classdef jNode < jObj
% jNode is the object that contains the properties of a node in a jNet.

% Author: Jiamin Wang; Updated: 2021-12-15;


    properties(SetAccess=protected)
        NodeNumber; % The Number of the Node in the jNet
        Net=[]; % The network of the node
        
        StartEdge=[]; % The Edges that start from the node
        EndEdge=[]; % The Edges that end in the node
    end
    
    methods(Access=public)
        function obj=jNode(inName,inNet)
            % Constructor of jNode. [Original to jNode]
            % obj=jNode(inName,inNet)
            % <inNet>: The jNet in which the jNode is established.
            obj@jObj(inName);
            if isa(inNet,'jNet')
                obj.Net=inNet;
                inNet.Node.reg(obj);
            end
            obj.StartEdge=jDic(obj.tagStr('StartEdge'),inNet.EdgeType);
            obj.EndEdge=jDic(obj.tagStr('EndEdge'),inNet.EdgeType);
            obj.NodeNumber=numel(obj.Net.Node.Content());
        end
        
        function output=netCheck(obj,node2)
            % Check if the two jNode are in the same jNet. [Original to jNode]
            % output=netCheck(obj,node2)
            output=0;
            if(isequal(obj.Net,node2.Net))
                output=1;
            end
        end
    end
end
