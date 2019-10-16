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

path = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nodeMap_start = containers.Map;
nodeMap_end = containers.Map;
robot = load('~/Users/tina_jia/Desktop/MEAM520/Lab2/robot.mat');
T_start = digraph;
T_end = digraph;
T_start = T_start.addnode('q_start');
nodeMap_start('q_start') = start;
T_end = T_end.addnode('q_end');
nodeMap_end('q_end') = goal;

isCollided = true;
found = false;
connection_point = 0;
for i = 1:100
    while isCollided
            one_rand = robot.lowerLim(1)+rand*(robot.lowerLim(1)-robot.upperLim(1));
            two_rand = robot.lowerLim(2)+rand*(robot.lowerLim(1)-robot.upperLim(2));
            three_rand = robot.lowerLim(3)+rand*(robot.lowerLim(1)-robot.upperLim(3));
            four_rand = robot.lowerLim(4)+rand*(robot.lowerLim(1)-robot.upperLim(4));
            q_sample = [one_rand, two_rand, three_rand, four_rand];
            isCollided = isRobotCollided(q_sample, map, robot);
    end
    %find closest point in start and goal trees
    [q_closest_start_name] = findClosestPoint(q_sample, nodeMap_start);
    [q_closest_end_name] = findClosestPoint(q_sample, nodeMap_end);
    %discretize these two paths & check sample_start_collide & sample_end_collide
    sample_start_collide = check_path_collide(q_sample, nodeMap_start(q_closest_start_name), map, robot);
    sample_end_collide = check_path_collide(q_sample, nodeMap_end(q_closest_end), map, robot);
    %add to tree if not 

    if ~sample_start_collide
        T_start = T_start.addnode(i);
        T_start = addedge(T_start, q_closest_start_name, i);
    end
    if ~sample_end_collide
        T_end = T_end.addnode(i);
        T_end = addedge(T_end, q_closest_end_name, i);
    end 
    if ~sample_start_collide && ~sample_end_collide
    	found = true;
        connection_point = i;
        break;
    end
end 

%find shortest path in the graph
if found
    p_start = shortestpath(T_start,'q_start',connection_point);
    p_end = shortestpath(T_end,'q_end',connection_point);
    path = [p_start, flip(p_end)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end