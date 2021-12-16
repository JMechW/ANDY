classdef jSys < jObj
% jSys is the basis for all the system objects, such as kSystem and
% nContSys
% Author: Jiamin Wang; Updated: 2021-12-15;

    methods(Access=public)
        function obj=jSys(inName)
            obj@jObj(inName);
        end
    end
end
