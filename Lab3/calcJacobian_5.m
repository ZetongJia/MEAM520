function J = calcJacobian_5(q, joint, robot)
% CALCJACOBIAN_5 Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [1,7] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%

%%

J = [];

if nargin < 3
    return
elseif joint <= 1 || joint > 7
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J = zeros(6,joint-1);

syms a;
syms r;
syms d;
syms theta;

A(a, r, d, theta) = [cos(theta), -sin(theta)*cos(r), sin(theta)*sin(r), a*cos(theta);
    sin(theta), cos(theta)*cos(r), -cos(theta)*sin(r), a*sin(theta);
    0, sin(r), cos(r), d;
    0, 0, 0, 1];

A0{1} = A(0, -pi/2, robot.d1, q(1));
A0{2} = A(robot.a2, 0, 0, q(2)-pi/2);
A0{3} = A(robot.a3, 0, 0, q(3)+pi/2);
A0{4} = A(0, -pi/2, 0, q(4)-pi/2);
A0{5} = A(0, pi/2, robot.d5, q(5)+pi/2);
A0{6} = A(0, 0, robot.d6, 0);
T{1} = eye(4,4);

for i = 1:joint-1
    T{i+1} = T{i}*A0{i}
end

de = T{joint-1}(1:3, 4)

for i = 1: joint-1
    J(1:3, i) = diff(de, q(i))
    J(4:6, i) = T{i}(1:3,3)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end