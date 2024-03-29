function [qNext, isDone] = potentialFieldStep_5(qCurr, map, robot)
% POTENTIALFIELDSTEP_GROUPNO Calculates a single step in a potential field
%   planner based on the virtual forces exerted by all of the elements in
%   map. This function will be called over and over until isDone is set.
%   Use persistent variables if you need historical information. CHANGE 
%   GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   qCurr - 1x6 vector representing the current configuration of the robot.
%   map   - a map struct containing the boundaries of the map, any
%           obstacles, the start position, and the goal position.
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   qNext  - 1x6 vector representing the next configuration of the robot
%            after it takes a single step along the potential field.
%   isDone - a boolean flag signifying termination of the potential field
%            algorithm. 
%
%               isDone == 1 -> Terminate the planner. We have either
%                              reached the goal or are stuck with no 
%                              way out.
%               isDone == 0 -> Keep going.

%%

% qNext = zeros(1,6);
% qNext = qCurr;
% qNext(1) = qNext(1)+.01;
% isDone = 0;

% conic to parabolic? 
zeta = [0,0,0,0,0,0,0.01];
alpha = 0.05;
rho0 = 200;
eta = [0,1000,1000,1000,1000,1000,1000];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[jointPositionsCurr,~] = calculateFK_sol(qCurr, robot);
[jointPositionsGoal,~] = calculateFK_sol(map.goal, robot);
[num, ~] = size(map.obstacles);
%attractive force for all joints
Fa = -zeta' .* (jointPositionsCurr - jointPositionsGoal);
tauSum = [0,0,0,0,0,0];
%loop through each joint
for i = 2:7
    J = calcJacobian_5(qCurr, i, robot);
    F = [0,0,0,0,0,0];
    %what is F(4:6) -- can be set to 0, not related to potential field
    F(1:3) = Fa(i,:);
    if (i == 1)
        tau = 0;
    else
        tau = forceToTorque_5(F, J);
    end
    tauSum(1:i-1) = tauSum(1:i-1)+tau';
    for j = 1:num
        [dist, unit] = distPointToBox(jointPositionsCurr(i,:), map.obstacles(j,:));
        if (dist > rho0)
            Fr = [0,0,0,0,0,0];
        else
            Fr = [0,0,0,0,0,0];
            FrLinear = eta(i)*((1/dist)-(1/rho0))*(1/dist^2)*(-unit);
            Fr(1:3) = FrLinear;
        end
        tau2 = forceToTorque_5(Fr, J);
        tauSum(1:i-1) = tauSum(1:i-1)+tau2';
    end 
end 

qNext = qCurr + alpha * (1-0.01*rand) * tauSum/norm(tauSum);
%have a limit for each joint, maybe check the workspace location for end effector only, 
%or have a different limit for each joint
[jointPositionsNext,~] = calculateFK_sol(qNext, robot);
if (norm(jointPositionsNext(7,:)-jointPositionsGoal(7,:)) < 3)
    isDone = 1;
else 
    isDone = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end