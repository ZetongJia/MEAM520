function [qClosest] = findClosestPoint(qSample, nodes)
%   FINDCLOSESTPATH Find the closest configuration to a given q
%   configuration in a map of configuration nodes
%
% INPUTS:
%   qSample   - a 1x6 configuration of the robot
%   nodes - a map of all configuration nodes in a tree 
%
% OUTPUTS:
%   qClosest - the key to the closest configuration node in the nodes map

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k = keys(nodes);
closestDistance = 1000;
for i = 1:length(nodes)
    currDistance = distance(qSample, nodes(k{i}).q);
    if currDistance < closestDistance
        closestDistance = currDistance;
        qClosest = k{i};
    end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end