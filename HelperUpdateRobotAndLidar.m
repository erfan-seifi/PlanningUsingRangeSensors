function exampleHelperUpdateRobotAndLidar(robotHandle, lineHandle, pose, ranges, angles, scale)
%exampleHelperUpdateRobotAndLidar Directly updates the graphics objects of 
%   the robot and rangeSensor in CreateEgocentricOccupanyMapsUsingRangeSensorsExample

%   Copyright 2019 The MathWorks, Inc.

    if nargin < 6
        scale = .5;
    end
    
    % Calculate robot/sensor pose relative to the world
    t = pose(1:2);
    a = pose(3);
    R = [cos(a) -sin(a); sin(a) cos(a)];
    G = [R t(:); 0 0 scale];
    
    % Update lidar rays
    lineHandle.XData(1:3:end) = pose(1);
    lineHandle.XData(2:3:end) = pose(1) + ranges.*cos(angles + pose(3));
    lineHandle.YData(1:3:end) = pose(2);
    lineHandle.YData(2:3:end) = pose(2) + ranges.*sin(angles + pose(3));
    
    % Update robot vertex locations
    RobotBodyTriangleVertices = G*[[[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]]; ones(1,4)];
    
    % Update patch
    robotHandle.XData = RobotBodyTriangleVertices(1,:);
    robotHandle.YData = RobotBodyTriangleVertices(2,:);
end