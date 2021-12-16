classdef jObj < matlab.mixin.Copyable
% jObj is the extended handle (Copyable) class, which provides the object
% basis for the ANDY library.

% Author: Jiamin Wang; Updated: 2021-12-15;


    properties(SetAccess=protected)      
        NameTag='obj';%Name Tag - The String ID used for object identification
    end
    
    methods(Access=public)
        function obj=jObj(inName)
            % Constructor of jObj. [Original to jObj]
            % obj=jObj(inName)
            % <inName>: NameTag (Char)
            if(ischar(inName))
                obj.NameTag=inName;
            else
                error(obj.msgStr('Error','Name Tag need to be char!'));
            end
        end
        
        function obj=reName(obj,inName)
            % Assign New NameTag to jObj. [Original to jObj]
            % obj=reName(obj,inName)
            % <inName>: New NameTag (Char)
            if(ischar(inName))
                obj.NameTag=inName;
            else
                error(obj.msgStr('Error','Name Tag need to be char!'));
            end
        end
        function output=tagStr(obj,inStr)
            % Add the jObj NameTag to the String. [Original to jObj]
            % output=tagStr(obj,inStr)
            % <inStr>: Original (Char)
            output=strcat(inStr,{'_'},obj.NameTag);
            output=output{1};
        end
        
        function output=msgStr(obj,inType,inStr)
            % Generate a message from the jObj NameTag [Original to jObj]
            % output=msgStr(obj,inType,inStr)
            % <inType>: Message Type (Char)
            % <inStr>: Message Content (Char)
            output=strcat(inType,{' from '},obj.NameTag,{': '},inStr);
            output=output{1};
        end
    end
end
