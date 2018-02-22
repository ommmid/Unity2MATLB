function [b, v, theta] = conM2U_Q(T)
% MATLAB to Unity Conversion in Quaternion form

% q = cos (theta/2) < v sin(theta/2)>
Uq = UnitQuaternion(T);
theta = -2*atan2(norm(Uq.v), Uq.s)*180/pi;
v = Uq.v/norm(Uq.v);

Sz = [1 0 0
      0 0 1
      0 1 0];

b = Sz * T(1:3,4);


