function p = quatMultiply(q, r)
% p = quatMultiply(q, r) 
% Left Multiplies quaternions <q> and <r>, returning their product <p>.
% For rotation, <r> happens first
% quat2Mat(quatMultiply(q, r))-quat2Mat(q)*quat2Mat(r)

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

if size(q, 1) ~= 4 || size(r, 1) ~= 4
  error('Expecting quaternions as Column.');
elseif size(q, 1) ~= size(r, 1)
  error('Number of quaternions don''t match.');
end

p = [q(1).*r(1) - q(2).*r(2) - q(3).*r(3) - q(4).*r(4);
     q(1).*r(2) + r(1).*q(2) + q(3).*r(4) - q(4).*r(3);
     q(1).*r(3) + r(1).*q(3) + q(4).*r(2) - q(2).*r(4);
     q(1).*r(4) + r(1).*q(4) + q(2).*r(3) - q(3).*r(2)];
end