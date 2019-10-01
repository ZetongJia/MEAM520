function [jointPositions,T0e] = calculateFK_21(q)
% CALCULATEFK_PENNKEY - Please rename this function using your pennkey in
%   both the function header and the file name. If you are working in
%   pairs, choose ONE pennkey to use - otherwise the grading scripts will
%   get confused.
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta1 = q(1);
    theta2 = q(2);
    theta3 = q(3);
    theta4 = q(4);
    theta5 = q(5);
    d1 = 76.20;
    d2 = 146.05;
    d3 = 187.325;
    d5 = 85.725-9.525;
    
    a1 = [cos(theta1) 0 -sin(theta1) 0; sin(theta1) 0 cos(theta1) 0; 0 -1 0 d1; 0 0 0 1];
    a2 = [sin(theta2) cos(theta2) 0 d2*sin(theta2); -cos(theta2) sin(theta2) 0 -d2*cos(theta2); 0 0 1 0; 0 0 0 1];
    a3 = [-sin(theta3) -cos(theta3) 0 -d3*sin(theta3); cos(theta3) -sin(theta3) 0 d3*cos(theta3); 0 0 1 0; 0 0 0 1];
    a4 = [sin(theta4) 0 cos(theta4) 0; -cos(theta4) 0 sin(theta4) 0; 0 -1 0 0; 0 0 0 1];
    a5 = [cos(theta5) -sin(theta5) 0 0; sin(theta5) cos(theta5) 0 0; 0 0 1 d5; 0 0 0 1];
    
    jointPositions = zeros(6, 3);
    jp2 = a1*[0;0;0;1];
    jointPositions(2,:) = (jp2(1:3))';
    jp3 = a1*a2*[0;0;0;1];
    jointPositions(3,:) = (jp3(1:3))';
    jp4 = a1*a2*a3*[0;0;0;1];
    jointPositions(4,:) = (jp4(1:3))';
    jp5 = a1*a2*a3*a4*[0;0;34;1];
    jointPositions(5,:) = (jp5(1:3))';
    jointPositions(5, 1) = jointPositions(5, 1);
    jp6 = a1*a2*a3*a4*a5*[0;0;0;1];
    jointPositions(6,:) = (jp6(1:3))';
    jointPositions(6, 1) = jointPositions(6, 1);
    
    T0e = zeros(4, 4);
    a6 = [1 0 0 0; 0 1 0 0; 0 0 1 28.575; 0 0 0 1];
    T0e = a1*a2*a3*a4*a5*a6;
    
end