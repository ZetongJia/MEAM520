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

startNodes('start') = struct('q', start(1:4), 'lastNode', -1);
goalNodes('goal') = struct('q', goal(1:4), 'lastNode', -1);

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
    [qClosestGoal] = findClosestPoint(qSample, goalNodes);
    
    % discretize these two paths and check if points along path collide
    isStartPathCollided = isPathCollided(qSample, startNodes(qClosestStart).q, map, robot, numSteps);
    isGoalPathCollided = isPathCollided(qSample, goalNodes(qClosestGoal).q, map, robot, numSteps);
    
    % add nodes to tree if not collided
    nodeIndex = int2str(i);
    if ~isStartPathCollided
        startNodes(nodeIndex) = struct('q', qSample, 'lastNode', qClosestStart);
    end
    if ~isGoalPathCollided
        goalNodes(nodeIndex) = struct('q', qSample, 'lastNode', qClosestGoal);
    end 
    if ~isStartPathCollided && ~isGoalPathCollided
    	found = true;
        connectionPoint = nodeIndex;
        break;
    end
end 

if found
    startPath = [];
    i = connectionPoint;
    while ~strcmp(i, 'start')
        startPath = [startNodes(i).q start(5) start(6); startPath];
        i = startNodes(i).lastNode;
    end
    startPath = [start; startPath];
    goalPath = [];
    i = connectionPoint;
    while ~strcmp(i, 'goal')
        goalPath = [goalPath; goalNodes(i).q goal(5) goal(6)];
        i = goalNodes(i).lastNode;
    end
    goalPath = [goalPath; goal];
    path = [startPath(1:size(startPath, 1) - 1, :); goalPath]
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end