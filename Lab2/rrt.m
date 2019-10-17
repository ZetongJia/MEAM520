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
goalNodes = containers.Map;
startGraph = digraph;
goalGraph = digraph;

startGraph = startGraph.addnode('start');
goalGraph = goalGraph.addnode('goal');
startNodes('start') = start(1:4);
goalNodes('goal') = goal(1:4);

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
    [qClosestgoal] = findClosestPoint(qSample, goalNodes);
    
    % discretize these two paths and check if points along path collide
    isStartPathCollided = isPathCollided(qSample, startNodes(qClosestStart), map, robot, numSteps);
    isgoalPathCollided = isPathCollided(qSample, goalNodes(qClosestgoal), map, robot, numSteps);
    
    % add nodes to tree if not collided
    nodeIndex = int2str(i);
    if ~isStartPathCollided
        startGraph = startGraph.addnode(nodeIndex);
        startNodes(nodeIndex) = qSample;
        startGraph = addedge(startGraph, qClosestStart, nodeIndex);
    end
    if ~isgoalPathCollided
        goalGraph = goalGraph.addnode(nodeIndex);
        goalNodes(nodeIndex) = qSample;
        goalGraph = addedge(goalGraph, qClosestgoal, nodeIndex);
    end 
    if ~isStartPathCollided && ~isgoalPathCollided
    	found = true;
        connectionPoint = nodeIndex;
        break;
    end
end 

if found
    % find shortest path in the graph
    startPath = shortestpath(startGraph,'start',connectionPoint);
    goalPath = shortestpath(goalGraph,'goal',connectionPoint);
    goalPath = flip(goalPath);
    
    % create path matrix
    startTreeSize = size(startPath, 2);
    goalTreeSize = size(goalPath, 2);
    path = zeros(startTreeSize + goalTreeSize - 1, 6);
    for i = 1: startTreeSize - 1
        path(i, :) = [startNodes(startPath{i}) start(5) start(6)];
    end
    for i = startTreeSize: startTreeSize + goalTreeSize - 1
        path(i, :) = [goalNodes(goalPath{i - startTreeSize + 1}) goal(5) goal(6)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm goals Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end