function [x] = motion_command(x, u)
% Updates the robot pose according to the motion model
% x: 3x1 vector representing the robot pose [x; y; theta]
% u: struct containing odometry reading (r1, t, r2).
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

%TODO: update x according to the motion represented by u

u_norm = normalize_all_bearings(u);

x(1) = x(1) + u_norm.t*cos(x(3)+u_norm.r1);
x(2) = x(2) + u_norm.t*sin(x(3)+u_norm.r1);
x(3) = x(3) + u_norm.r1 + u_norm.r2;

%TODO: remember to normalize theta by calling normalize_angle for x(3)

end
