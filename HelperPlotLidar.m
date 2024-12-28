function lineHandle = exampleHelperPlotLidar(ax, pose, ranges, angles)
%exampleHelperPlotLidar Plots the lidar for CreateEgocentricOccupanyMapsUsingRangeSensorsExample
%
%   Plots the lidar rays specified by ranges and angles overtop the
%   given axes, ax, and returns a handle to the line object

%   Copyright 2019 The MathWorks, Inc.

    x = repmat(pose(1),3*length(ranges),1);
    y = repmat(pose(2),3*length(ranges),1);
    x(3:3:end) = nan;
    y(3:3:end) = nan;
    x(2:3:end) = pose(1) + ranges.*cos(angles + pose(3));
    y(2:3:end) = pose(2) + ranges.*sin(angles + pose(3));
    lineHandle = plot(x,y,'Parent',ax);
end