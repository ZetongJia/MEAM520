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

% Calculate A matrices
A0{1} = calcA(0, -pi/2, robot.d1, q(1));
A0{2} = calcA(robot.a2, 0, 0, q(2)-pi/2);
A0{3} = calcA(robot.a3, 0, 0, q(3)+pi/2);
A0{4} = calcA(0, -pi/2, 0, q(4)-pi/2);
A0{5} = calcA(0, pi/2, robot.d5, q(5)+pi/2);
A0{6} = calcA(0, 0, robot.d6, 0);

% Calculate T matrices
T0{1} = eye(4,4);
for i = 1:joint-1
    T0{i+1} = T0{i}*A0{i};
end

% Handle joint 4 center different from frame center
if joint == 5
    T0{5} = T0{5}*calcA(0, 0, 34, 0);
end

% Calculate J
oe = T0{joint}(1:3, 4);
for i = 1: joint-1
    J(1:3, i) = cross(T0{i}(1:3, 3), oe-T0{i}(1:3, 4));
    J(4:6, i) = T0{i}(1:3, 3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end