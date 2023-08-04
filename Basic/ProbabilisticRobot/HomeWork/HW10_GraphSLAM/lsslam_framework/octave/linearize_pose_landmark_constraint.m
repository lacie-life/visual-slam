% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  X = v2t(x);
    
  Ri = X(1:2,1:2);

  e = Ri'*(l-x(1:2))-z;

  thi = atan2(Ri(2,1),Ri(1,1));

  xl = l(1); yl = l(2);
  xi = x(1); yi = x(2);

  A = [-cos(thi) -sin(thi) -sin(thi)*(xl-xi) + cos(thi)*(yl-yi); sin(thi) -cos(thi) -cos(thi)*(xl-xi)-sin(thi)*(yl-yi)];
  B = [cos(thi) sin(thi); -sin(thi) cos(thi)];


end;
