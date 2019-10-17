function [path] = rrt(map, start, goal)
% RRT Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The 
%   first row is start and the last row is goal. If no path is found, PATH 
%   is a 0x6 matrix. 
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% constants
robot = load('robot.mat');
numIter = 100;
numSteps = 10;

% initialize variables
startNodes = containers.Map;
endNodes = containers.Map;
startGraph = digraph;
endGraph = digraph;

startGraph = startGraph.addnode('q_start');
endGraph = endGraph.addnode('q_end');
startNodes('q_start') = start(1:4);
endNodes('q_end') = goal(1:4);

found = false;
connectionPoint = 0;
for i = 1:numIter
    isCollided = true;
    while isCollided
        qSample = random('uniform', robot.robot.lowerLim(1:4), robot.robot.upperLim(1:4));
        isCollided = isRobotCollided(qSample, map, robot);
    end
    
    % find closest point in start and goal trees
    [qClosestStart] = findClosestPoint(qSample, startNodes);
    [qClosestEnd] = findClosestPoint(qSample, endNodes);
    
    % discretize these two paths and check if points along path collide
    isStartPathCollided = isPathCollided(qSample, startNodes(qClosestStart), map, robot, numSteps);
    isEndPathCollided = isPathCollided(qSample, endNodes(qClosestEnd), map, robot, numSteps);
    
    % add nodes to tree if not collided
    nodeIndex = int2str(i);
    if ~isStartPathCollided
        startGraph = startGraph.addnode(nodeIndex);
        startNodes(nodeIndex) = qSample;
        startGraph = addedge(startGraph, qClosestStart, nodeIndex);
    end
    if ~isEndPathCollided
        endGraph = endGraph.addnode(nodeIndex);
        endNodes(nodeIndex) = qSample;
        endGraph = addedge(endGraph, qClosestEnd, nodeIndex);
    end 
    if ~isStartPathCollided && ~isEndPathCollided
    	found = true;
        connectionPoint = nodeIndex;
        break;
    end
end 

% find shortest path in the graph
if found
    startPath = shortestpath(startGraph,'q_start',connectionPoint);
    endPath = shortestpath(endGraph,'q_end',connectionPoint);
    endPath = flip(endPath);
    
    % create path matrix
    startTreeSize = size(startPath, 2);
    endTreeSize = size(endPath, 2);
    path = zeros(startTreeSize + endTreeSize - 1, 6);
    for i = 1: startTreeSize - 1
        path(i, :) = [startNodes(startPath{i}) start(5) start(6)];
    end
    for i = startTreeSize: startTreeSize + endTreeSize - 1
        path(i, :) = [endNodes(endPath{i - startTreeSize + 1}) goal(5) goal(6)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end