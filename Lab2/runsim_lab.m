%% Setup

close all
addpath('utils')
addpath('maps')

profile on

%% Simulation Parameters

load 'robot.mat' robot

start = [0,0,0,0,0,0];
%goal = [1.4,0,0,0,0,0];
goal = [0,0,1.4,0,0,0];

map = loadmap('map1.txt');

start = [-1.2,0,0,0,0,0];
%goal = [1.4,0,0,0,0,0];
goal = [1.4,0,0,0,0,0];

map = loadmap('map2.txt');

%% Run the simulation

% Turn the map into a C-space map
cmap = getCMap(map,robot,[0.2,0.2,0.2],10);
% Solve the path problem
[path, num] = astar(cmap, start, goal, false);

% OR
% [path] = rrt(map, start, goal);

profile off

%% Plot the output
plotLynxPath(map,path,10);

profile viewer
