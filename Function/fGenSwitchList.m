function string=fGenSwitchList(Flag,Case,Process)
% string=fGenSwitchList(Flag,Case,Process)
% A function used to generate switch-case-otherwise formats. <Flag> for
% switched variable, <Case> for switched value, <Process> for case functions.
% If <Case> is one number less than the <Process>, assign the last of the
% <Process> in otherwise case.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

    % Input Initialization
    string={};
    if numel(Process)-numel(Case)==0
        otherwiseflag=0;%Otherwise Statement Dectivated
    elseif numel(Process)-numel(Case)==1
        otherwiseflag=1;%Otherwise Statement Activated
    else
        error('Switch process group quantity should be equal or one more than case group quantity!')
    end
    
    line=strcat('switch',{' '},Flag);
    string{1,1}=line{:};
    
    % Generate Case
    for ii=1:numel(Case)
        if(ischar(Case{ii}))
            caseflag=strcat('''',Case{ii},'''');
        else
            caseflag=num2str(Case{ii});
        end
        line=strcat('    case',{' '},caseflag);
        %string{end+1,1}=line{:};
        string=cat(1,string,line);
        processStr=Process{ii};
        for jj=1:numel(processStr)
            line=strcat({'        '},processStr{jj});%Extract content by line and add indent
            string=cat(1,string,line);
        end
    end
    
    % Generate Otherwise Case
    if(otherwiseflag)
        string{end+1,1}='    otherwise';
        processStr=Process{end};
        for jj=1:numel(processStr)
            line=strcat({'        '},processStr{jj});%Extract content by line and add indent
            string=cat(1,string,line);
        end
    end
    
    string{end+1,1}='end';
end
