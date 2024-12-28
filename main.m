% Specify the saving path
outputFolder = '~/Downloads'; 
if ~isfolder(outputFolder)
    mkdir(outputFolder); 
end

% Define the full file path
videoFilename = fullfile(outputFolder, 'simulation_video.avi');

% Create VideoWriter object
v = VideoWriter(videoFilename, 'Motion JPEG AVI');
v.FrameRate = 10; % Set the frame rate

open(v);

% Initialize other variables (retain the rest of your code)
load mapData_rayTracingTrajectory
binaryMatrix = mapData.Data > 0.5;
worldMap = binaryOccupancyMap(binaryMatrix, mapData.Resolution);
worldMap.LocalOriginInWorld = mapData.GridLocationInWorld;

set(gcf, 'Visible', 'on');
worldAx = subplot(1, 2, 1);
worldHandle = show(worldMap, 'Parent', worldAx);
hold all;

% Initialize the sensor, robot, and plots (retain this part)
lidar = rangeSensor;
lidar.Range = [0 5];
lidar.RangeNoise = 0;
pos = [0 0 0];
[ranges, angles] = lidar(pos, worldMap);
hSensorData = HelperPlotLidar(worldAx, pos, ranges, angles);
hRobot = HelperPlotRobot(worldAx, pos);

egoMap = occupancyMap(10, 10, worldMap.Resolution);
egoMap.GridOriginInLocal = -[diff(egoMap.XWorldLimits) diff(egoMap.YWorldLimits)] / 2;
localAx = subplot(1, 2, 2);
show(egoMap, 'Parent', localAx);
hold all;
localMapFig = plot(localAx, egoMap.LocalOriginInWorld + [0 1], egoMap.LocalOriginInWorld + [0 0], 'r-', 'LineWidth', 3);

% Plan the trajectory (retain this part)
binaryMap = binaryOccupancyMap(worldMap);
inflate(binaryMap, 0.1);
stateSpace = stateSpaceDubins;
stateSpace.MinTurningRadius = 0.5;
validator = validatorOccupancyMap(stateSpace);
validator.Map = binaryMap;
validator.ValidationDistance = 0.1;
planner = plannerRRTStar(stateSpace, validator);
planner.MaxConnectionDistance = 2;
planner.MaxIterations = 20000;
rng(1, 'twister');
startPt = [-6 -5 0];
goalPt = [8 7 pi/2];
path = plan(planner, startPt, goalPt);
interpolate(path, size(path.States, 1) * 10);
plot(worldAx, path.States(:, 1), path.States(:, 2), 'b-');

% Simulate robot movement and record the video
pt2ptDist = distance(stateSpace, path.States(1:end-1, :), path.States(2:end, :));
linVel = 0.5; % m/s
tStamps = cumsum(pt2ptDist) / linVel;
traj = waypointTrajectory(path.States, [0; tStamps], 'SampleRate', 10);
reset(traj);
robotCurrentPose = path.States(1, :);
move(egoMap, robotCurrentPose(1:2));
setOccupancy(egoMap, repmat(egoMap.DefaultValue, egoMap.GridSize));

while ~isDone(traj)
    [pts, quat] = step(traj);
    rotMatrix = rotmat(quat, 'point');
    orientZ = rotm2eul(rotMatrix);
    robotCurrentPose = [pts(1:2) orientZ(1)];
    move(egoMap, robotCurrentPose(1:2), 'MoveType', 'Absolute');
    [ranges, angles] = lidar(robotCurrentPose, worldMap);
    insertRay(egoMap, robotCurrentPose, ranges, angles, lidar.Range(2));
    show(egoMap, 'Parent', localAx, 'FastUpdate', 1);
    set(localMapFig, 'XData', robotCurrentPose(1) + [0 cos(robotCurrentPose(3))], 'YData', robotCurrentPose(2) + [0 sin(robotCurrentPose(3))]);
    HelperUpdateRobotAndLidar(hRobot, hSensorData, robotCurrentPose, ranges, angles);

    % Capture the current frame and write it to the video
    frame = getframe(gcf);
    writeVideo(v, frame);

    drawnow limitrate;
end

% Close the video file
close(v);

