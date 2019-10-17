function [isCollided] = isRobotCollided(q, map, robot)
% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[jointPositions,T0e] = calculateFK_sol([q 0 0]);
jointPositions = [jointPositions(1:4, :); T0e(1:3, 4)'];
jointSizes = 25.4 .* [3.5 3 2 3;3.5 1 1 3.5]; % [link12 link23 link34 link4end] width and height in mm

isCollided = false;
for obstacle = 1:size(map.obstacles,1)
    for joint = 1:4
        for corner = 1:4
            % TODO: add four corners
%             point1 = [jointPositions(joint, :)(
            isCollided = isCollided || detectCollision(jointPositions(joint, :), jointPositions(joint+1, :), map.obstacles(obstacle, :));
        end
    end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end







