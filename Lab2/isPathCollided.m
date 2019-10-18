function [isPathCollided] = isPathCollided(qSample, qClosest, map, robot, numSteps)
% ISPATHCOLLIDED Detect if the path between 2 configurations of the Lynx is
% in collision with any obstacles on the map.
%
% INPUTS:
%   qSample   - a 1x6 configuration of the robot
%   qClosest - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%   numSteps - number of steps in interpolation betweeen the 2
%   configurations
%
% OUTPUTS:
%   isPathCollided - a boolean flag: 1 if the robot is in collision at some
%   point in path, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qSteps = qSample + [0:1/numSteps:1]' * (qClosest - qSample);
isPathCollided = false;
for i = 1:numSteps
    qStep = qSteps(i, :);
    isPathCollided = isPathCollided || isRobotCollided(qStep, map, robot);
end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end