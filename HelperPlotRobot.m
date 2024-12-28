function robotHandle = exampleHelperPlotRobot(ax, pose, scale)
%exampleHelperPlotRobot Plots the robot for CreateEgocentricOccupanyMapsUsingRangeSensorsExample
%
%   Plots the robot as a patch by an SE2 position, pose, and scaling value,
%   scale. Returns a handle to the patch object

%   Copyright 2019 The MathWorks, Inc.

    if nargin < 3
        scale = 0.5;
    end

    % Update robot position
    t = pose(1:2);
    a = pose(3);
    R = [cos(a) -sin(a); sin(a) cos(a)];
    G = [R t(:); 0 0 scale];
    
    % Transform the default robot vertices using the robot pose/orientation
    RobotBodyTriangleVertices = G*[[[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]]; ones(1,4)];
    RobotBodyFaceColor = [0.866 0.918 0.753];
    
    % Create the patch and return the handle
    robotHandle = patch(ax, RobotBodyTriangleVertices(1,:), RobotBodyTriangleVertices(2,:), RobotBodyFaceColor);
end