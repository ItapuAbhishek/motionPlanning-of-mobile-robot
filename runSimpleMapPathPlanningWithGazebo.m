function runSimpleMapPathPlanningWithGazebo
%converting map into a occupancy grid
%% 
 
filePath = fullfile(fileparts(which('TurtleBotMonteCarloLocalizationExample')),'data','officemap.mat');
load(filePath);
map.GridLocationInWorld=[-7.03,-3.525];
figure();
show(map);
%% 

%inflate the map 
inflatedMap=copy(map);
inflate(inflatedMap,0.1);
figure;
show(inflatedMap)
prm=robotics.PRM(inflatedMap);

%% sampling and finding the path
%% 
prm.NumNodes=300;
prm.ConnectionDistance=1
figure;
ax=axes;
show(prm,'parent',ax);
start=[0,0];
goal=[-4.5 3.2];
path=findpath(prm,start,goal);
hold('on');
show(prm,'Map','off','RoadMap','off');
hold(ax,'off');
%% 
% controller to make robot to follow the path
%% 
controller=robotics.PurePursuit;
controller.Waypoints=path;
robotCurrentLocation=path(1,:);
robotGoal=path(end,:);
startPoint=[robotCurrentLocation,0];
controller.DesiredLinearVelocity=0.3;
controller.MaxAngularVelocity=2;
controller.LookaheadDistance=0.6;
%% 
% create a simulated robot for controller
% 
robot=ExampleHelperDifferentialDriveRobot(startPoint);
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.CurrentPose);

    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);

    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.CurrentPose;

    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);

    waitfor(controlRate);

end