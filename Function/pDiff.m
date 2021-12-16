function [de_EqSum,de_Eq]=pDiff(f,xVar)
% [de_EqSum,de_Eq]=pDiff(f,xVar)
% This function is used to calculate the partial derivative of a set of
% expressions <f> with respect to a dependent variable <xVar> To use this function
% requires the correct naming protocol:
%   {x}__d{@}_{$}_ 
% Here {x} is the actual variable name {@} is the derivative variable
% xVar, {$} is the rank of derivative.
% [de_EqSum] is the default output, [de_Eq] is a secondary output that gives
% derivative of each <xVar> dependent variable.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    % Get all the variables for differentiation
    x0=symvar(f);
    x0(end+1)=xVar;
    x0=unique(x0);
        
    % Differentiation
    if(max(size(xVar))>1)
        error('Currently Only Support Partial Differentiation According to One Variable')
    else
        diffSign=strcat('d',char(xVar));
        fSize=size(f);
        x0Size=numel(x0);
        de_EqSum=sym(zeros(fSize(1),fSize(2)));
        de_Eq=sym(zeros(fSize(1),fSize(2),x0Size));
        for ii=1:x0Size
            diff_Eq=diff(f,x0(ii));    
            symContent=getSymTag(x0(ii),diffSign);
            
            if(isempty(symContent))
                if(x0(ii)==xVar)
                    de_Eq(:,:,ii)=diff_Eq;%If the current variable is xVar
                else
                    de_Eq(:,:,ii)=zeros(fSize);%If the current variable is xVar-independent
                end
            else%If the current variable is xVar-dependent
                
                dx_Str=setSymTag(x0(ii),diffSign,str2double(symContent)+1);
                de_Eq(:,:,ii)=diff_Eq*sym(dx_Str);
            end
            
            de_EqSum=de_EqSum+de_Eq(:,:,ii);
        end
    end
end
