function output=jSimplify(input,varargin)
% output=jSimplify(input)
% A Wrapper for 'simplify' for ANDY to provide alternative simplify options
% to the <input>. This file can be motified based on user preference.

% Author: Jiamin Wang; Updated: 2021-12-15;

    type=true;
    if(nargin>1)
        type=varargin{1};
    end
    if type
    output=simplify(collect(simplify(expand(simplify(input,'Criterion','preferReal')),'Criterion','preferReal')),'Criterion','preferReal');
    end
end
