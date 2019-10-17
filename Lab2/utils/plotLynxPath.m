function plotLynxPath(map, path, totalT)
% PLOTLYNXPATH Plots the lynx following a set of waypoints produced by a
%   mapping algorithm such as Astar or RRT
%
% INPUTS
%   map - a struct containing the workspace map
%   path - a set of Nx6 configurations of the lynx
%   totalT - the total time for the plotting (s)
%
% OUTPUTS
%   N/A
%
% AUTHOR
%   Gedaliah Knizhnik - knizhnik@seas.upenn.edu

if nargin < 2
    totalT = 10;
end

n = 20;
t = linspace(0,1,n)';

lynxStart()
plotmap(map)

for ii=1:size(path,1)-1
    morePath = (1-t)*path(ii,:) + t*path(ii+1,:);
    
    for jj = 1:size(morePath,1)
        lynxServo(morePath(jj,1),morePath(jj,2),morePath(jj,3),0,0,0);
        [~,T] = calculateFK_sol(morePath(jj,:));
        plot3(T(1,4),T(2,4),T(3,4),'k.');
        pause(totalT/n/size(path,1))
    end

end

end