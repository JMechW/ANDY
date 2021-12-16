function output=skew3(w)
% output=skew3(w)
% Convert 3x1 vector <w> to a skew matrix <output>.

% Author: Jiamin Wang; Updated: 2021-12-15;


    if(numel(w)==3)
        output=[0 -w(3) w(2) ; w(3) 0 -w(1) ; -w(2) w(1) 0 ];
    else
        error('Cannot convert to Skew Symmetric Matrix unless is a 3x1 vector');
    end
end
