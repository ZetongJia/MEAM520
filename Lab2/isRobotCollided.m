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

% calculate FK to get joint positions
[jointPositions,T0e] = calculateFK_sol([q 0 0]);
jointPositions = [jointPositions(1:4, :); T0e(1:3, 4)'];

% calculate FK to get intermediate rotation matrices 
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q(4);
theta5 = 0;
a1 = [cos(theta1) 0 -sin(theta1) 0; sin(theta1) 0 cos(theta1) 0; 0 -1 0 robot.robot.d1; 0 0 0 1];
a2 = [sin(theta2) cos(theta2) 0 robot.robot.a2*sin(theta2); -cos(theta2) sin(theta2) 0 -robot.robot.a2*cos(theta2); 0 0 1 0; 0 0 0 1];
a3 = [-sin(theta3) -cos(theta3) 0 -robot.robot.a3*sin(theta3); cos(theta3) -sin(theta3) 0 robot.robot.a3*cos(theta3); 0 0 1 0; 0 0 0 1];
a4 = [sin(theta4) 0 cos(theta4) 0; -cos(theta4) 0 sin(theta4) 0; 0 -1 0 0; 0 0 0 1];
a5 = [cos(theta5) -sin(theta5) 0 0; sin(theta5) cos(theta5) 0 0; 0 0 1 robot.robot.d5+28.575; 0 0 0 1];

isCollided = false;
for obstacle = 1:size(map.obstacles,1)
    for joint = 1:4
        % check if link 
    	isCollided = isCollided || detectCollision(jointPositions(joint, :), jointPositions(joint+1, :), map.obstacles(obstacle, :));
        
        % giving link 23, 34, 456 volume to avoid collision
        if(joint == 2)
            a = a1*a2*([0.5;0;1.5;1]*25.4);
            b = a1*a2*([0.5;0;-1.5;1]*25.4);
            c = a1*a2*([-0.5;0;-1.5;1]*25.4);
            d = a1*a2*([-0.5;0;1.5;1]*25.4);
            
            aa = a1*a2*a3*([0;0.5;1.5;1]*25.4);
            bb = a1*a2*a3*([0;0.5;-1.5;1]*25.4);
            cc = a1*a2*a3*([0;-0.5;-1.5;1]*25.4);
            dd = a1*a2*a3*([0;-0.5;1.5;1]*25.4);
            isCollided = isCollided || detectCollision(a(1:3).', aa(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(b(1:3).', bb(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(c(1:3).', cc(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(d(1:3).', dd(1:3).', map.obstacles(obstacle, :));
        end 
        if(joint == 3)
            a = a1*a2*a3*([0.5;0;-1;1]*25.4);
            b = a1*a2*a3*([-0.5;0;-1;1]*25.4);
            c = a1*a2*a3*([0.5;0;1;1]*25.4);
            d = a1*a2*a3*([-0.5;0;1;1]*25.4);
            
            aa = a1*a2*a3*a4*([0.5;1;0;1]*25.4);
            bb = a1*a2*a3*a4*([-0.5;1;0;1]*25.4);
            cc = a1*a2*a3*a4*([0.5;-1;0;1]*25.4);
            dd = a1*a2*a3*a4*([0.5;-1;0;1]*25.4);
            isCollided = isCollided || detectCollision(a(1:3).', aa(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(b(1:3).', bb(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(c(1:3).', cc(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(d(1:3).', dd(1:3).', map.obstacles(obstacle, :));
        end
        if(joint == 4)       
            a = a1*a2*a3*a4*([1;1;0;1]*25.4);
            b = a1*a2*a3*a4*([-1;1;0;1]*25.4);
            c = a1*a2*a3*a4*([1;-1;0;1]*25.4);
            d = a1*a2*a3*a4*([1;-1;0;1]*25.4);
            
            aa = a1*a2*a3*a4*a5*([1;1;0;1]*25.4);
            bb = a1*a2*a3*a4*a5*([-1;1;0;1]*25.4);
            cc = a1*a2*a3*a4*a5*([1;-1;0;1]*25.4);
            dd = a1*a2*a3*a4*a5*([1;-1;0;1]*25.4);
            isCollided = isCollided || detectCollision(a(1:3).', aa(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(b(1:3).', bb(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(c(1:3).', cc(1:3).', map.obstacles(obstacle, :));
            isCollided = isCollided || detectCollision(d(1:3).', dd(1:3).', map.obstacles(obstacle, :));
        end
    end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end







