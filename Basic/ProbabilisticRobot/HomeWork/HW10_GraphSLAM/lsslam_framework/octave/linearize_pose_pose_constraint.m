% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);
  e = t2v(Z\(X1\X2));
  
  Rij = Z(1:3, 1:3);
  Ri = X1(1:3, 1:3);
  
  thi = atan2(Ri(2,1),Ri(1,1));
  thij = atan2(Rij(2,1),Rij(1,1));
  
  xi = x1(1);
  yi = x1(2);
  xj = x2(1);
  yj = x2(2);
  
  A = [-cos(thi)*cos(thij)+sin(thi)*sin(thij) -sin(thi)*cos(thij)-cos(thi)*sin(thij) 0; 
       cos(thi)*sin(thij)+sin(thi)*cos(thij) sin(thi)*sin(thij)-cos(thi)*cos(thij) 0; 
       0 0 -1];
       
  A(1:2,3) = [cos(thij)*(-sin(thi)*(xj-xi)+cos(thi)*(yj-yi))+sin(thij)*(-cos(thi)*(xj-xi)-sin(thi)*(yj-yi)); 
              -sin(thij)*(-sin(thi)*(xj-xi)+cos(thi)*(yj-yi))+cos(thij)*(-cos(thi)*(xj-xi)-sin(thi)*(yj-yi))];
  
  B = [cos(thi)*cos(thij)-sin(thi)*sin(thij) sin(thi)*cos(thij)+cos(thi)*sin(thij) 0; 
      -cos(thi)*sin(thij)-sin(thi)*cos(thij) -sin(thi)*sin(thij)+cos(thi)*cos(thij) 0; 
      0 0 1];

end;
