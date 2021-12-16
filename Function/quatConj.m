function p = quatConj(q)
% p = quatConj(q)
% Quaternion conjugation from <q> to <p>.

% Robotics & Mechatronics Lab, Virginia Tech. (No Copyright Claimed)
% Author: Jiamin Wang; Revised: 20-Jan-2019

if size(q, 1) ~= 4
  error('Expecting quaternions as Column.');
end

p = [q(1);-q(2:4)];
end