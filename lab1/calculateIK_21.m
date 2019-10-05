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

    alpha1 = atan2(T0e(2,4),T0e(1,4));
    yc = T0e(2,4) - 28.575*T0e(2,3);
    xc = T0e(1,4) - 28.575*T0e(1,3);
    alpha2 = atan2(yc,xc);    
    alpha3 = alpha2-alpha1;
    feasible = true;
    if (alpha3 > 10^(-4))
        projection = [1, 0, 0, 0; 0, cos(alpha3), -sin(alpha3), 0; ...
            0, sin(alpha3), cos(alpha3), 0; 0, 0, 0, 1];
        T0e = T0e * projection;
        feasible = false;
    end
    
    X = T0e(1,4) - d6*T0e(1,3);
    Y = T0e(2,4) - d6*T0e(2,3);
    Z = T0e(3,4) - d6*T0e(3,3);
    r1 = sqrt(X^2+Y^2);
    r2 = d1-Z;
    r3 = sqrt(r1^2+r2^2);
    phi1 = acos((d3^2-d2^2-r3^2)/(-2*d2*r3));
    phi2 = atan2(r2, r1);
    phi3 = acos((r3^2-d2^2-d3^2)/(-2*d2*d3));
    
    % Facing +x0, Elbow Up
    theta1_1 = atan2(Y,X)
    theta2_1 = pi/2 + (phi2 - phi1);
    theta3_1 = pi - phi3 - pi/2
    
    a1_1 = [cos(theta1_1) 0 -sin(theta1_1) 0; sin(theta1_1) 0 cos(theta1_1) 0; 0 -1 0 d1; 0 0 0 1];
    a2_1 = [sin(theta2_1) cos(theta2_1) 0 d2*sin(theta2_1); -cos(theta2_1) sin(theta2_1) 0 -d2*cos(theta2_1); 0 0 1 0; 0 0 0 1];
    a3_1 = [-sin(theta3_1) -cos(theta3_1) 0 -d3*sin(theta3_1); cos(theta3_1) -sin(theta3_1) 0 d3*cos(theta3_1); 0 0 1 0; 0 0 0 1];

    T03_1 = a1_1*a2_1*a3_1;
    R03_1 = T03_1(1:3,1:3);
    R46_1 = R03_1'*T0e(1:3,1:3);

    theta5_1 = acos(-R46_1(3,2));
    theta4_1 = asin(R46_1(2,3));
    q_1 = [theta1_1, theta2_1, theta3_1, theta4_1, theta5_1];
    
    % Facing +x0, Elbow Down
    theta1_2 = atan2(Y,X)
    theta2_2 = pi/2 + (phi2 + phi1)
    theta3_2 = -theta3_1
    
    a1_2 = [cos(theta1_2) 0 -sin(theta1_2) 0; sin(theta1_2) 0 cos(theta1_2) 0; 0 -1 0 d1; 0 0 0 1];
    a2_2 = [sin(theta2_2) cos(theta2_2) 0 d2*sin(theta2_2); -cos(theta2_2) sin(theta2_2) 0 -d2*cos(theta2_2); 0 0 1 0; 0 0 0 1];
    a3_2 = [-sin(theta3_2) -cos(theta3_2) 0 -d3*sin(theta3_2); cos(theta3_2) -sin(theta3_2) 0 d3*cos(theta3_2); 0 0 1 0; 0 0 0 1];

    T03_2 = a1_2*a2_2*a3_2;
    R03_2 = T03_2(1:3,1:3);
    R46_2 = R03_2'*T0e(1:3,1:3);

    theta5_2 = acos(-R46_2(3,2))
    theta4_2 = asin(R46_2(2,3))
    q_2 = [theta1_2, theta2_2, theta3_2, theta4_2, theta5_2];
    
    q_isPos = [];
    lowerLimit = [-1.4, -1.2, -1.8, -1.9, -2];
    upperLimit = [1.4, 1.4, 1.7, 1.7, 1.5];
    isPos = 1;
    
    if (isreal(q_1))
        q_isPos = [q_isPos; q_1];
    end
    if (isreal(q_2))
        q_isPos = [q_isPos; q_2];
    end
    if (isempty(q_isPos))
        q_isPos = [];
        isPos = 0;
    end
    % add feasibility
    if (feasible == false)
        isPos = 0;
    end   
    
    q = []
    for row=1:size(q_isPos,1)
        if all(q_isPos(row, :) >= lowerLimit & q_isPos(row, :) <= upperLimit)
            q = [q; q_isPos(row, :)]
        end
    end
    q
end