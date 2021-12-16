function p = quatConj(q)
% p = quatConj(q)
% Quaternion conjugation from <q> to <p>.

% Author: Jiamin Wang; Updated: 2021-12-15;

if size(q, 1) ~= 4
  error('Expecting quaternions as Column.');
end

p = [q(1);-q(2:4)];
end
