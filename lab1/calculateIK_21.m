function [q isPos] = calculateIK_pennkey(T0e)
% CALCULATEIK_PENNKEY - Please rename this function using your pennkey in
%   both the function header and the file name. If you are working in
%   pairs, choose ONE pennkey to use - otherwise the grading scripts will
%   get confused.
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)
%
% OUTPUT:
%   q          - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
%                which are required for the Lynx robot to reach the given 
%                transformation matrix T. Each row represents a single
%                solution to the IK problem. If the transform is
%                infeasible, q should be all zeros.
%   isPos      - a boolean set to true if the provided
%                transformation T is achievable by the Lynx robot, ignoring
%                joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    d1 = 76.20;
    d2 = 146.05;
    d3 = 187.325;
    d5 = 85.725-9.525;
    d6 = 104.775;

    X = T0e(1,4) - d6*T0e(1,3);
    Y = T0e(2,4) - d6*T0e(2,3);
    Z = T0e(3,4) - d6*T0e(3,3);
    r1 = sqrt(X^2+Y^2);
    r2 = d1-Z;
    r3 = sqrt(r1^2+r2^2);
    phi1 = acos((d3^2-d2^2-r3^2)/(-2*d2*r3));
    phi2 = atan2(r2, r1);
    phi3 = acos((r3^2-d2^2-d3^2)/(-2*d2*d3));
    
    theta1 = atan2(Y,X);
    theta2 = pi/2 + (phi2 - phi1);
    theta3 = pi - phi3 - pi/2;


    a1 = [cos(theta1) 0 -sin(theta1) 0; sin(theta1) 0 cos(theta1) 0; 0 -1 0 d1; 0 0 0 1];
    a2 = [sin(theta2) cos(theta2) 0 d2*sin(theta2); -cos(theta2) sin(theta2) 0 -d2*cos(theta2); 0 0 1 0; 0 0 0 1];
    a3 = [-sin(theta3) -cos(theta3) 0 -d3*sin(theta3); cos(theta3) -sin(theta3) 0 d3*cos(theta3); 0 0 1 0; 0 0 0 1];

    T03 = a1*a2*a3;
    R03 = T03(1:3,1:3);
    R46 = R03'*T0e(1:3,1:3);

    theta5 = -acos(R46(3,2));
    theta4 = asin(R46(2,3));
%     q = [0, 0, 0, 0, 0];
    q = [theta1, theta2, theta3, theta4, theta5];
    isPos = 1;

end