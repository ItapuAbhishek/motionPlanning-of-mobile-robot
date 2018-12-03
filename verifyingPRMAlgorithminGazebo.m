function verifyingPRMAlgorithminGazebo

%converting map into a occupancy grid
%% 
filePath = fullfile(fileparts(which('TurtleBotMonteCarloLocalizationExample')),'data','officemap.mat');
load(filePath);
%xworldlimits=[-6.24,7.07];
%yworldlimits=[-2.25,5.56];
map.GridLocationInWorld=[-7.03 ,-3.525];
figure();
show(map);
% 
%inflate the map 
% 
inflatedMap=copy(map);
inflate(inflatedMap,0.1);
figure;
prm=robotics.PRM(inflatedMap);
% 
prm.NumNodes=300;
figure;
 ax=axes;
show(prm,'parent',ax);
start=[0.0001  -0.0000];
goal=[4.5,-3.0];
path=findpath(prm,start,goal);
hold('on');
show(prm,'Map','off','RoadMap','off');
hold(ax,'off');
%% sampling and finding the path
controller=robotics.PurePursuit;
controller.Waypoints=path;
robotCurrentLocation=path(1,:);
robotGoal=path(end,:);
startPoint=[robotCurrentLocation,0];
controller.DesiredLinearVelocity=0.3;
controller.MaxAngularVelocity=0.2;
controller.LookaheadDistance=0.6;
initialOrientation = 0;
startPoint = [robotCurrentLocation initialOrientation];
%%communicating with ros
%% 
%robot=ExampleHelperDifferentialDriveRobot(startPoint);
updateCounter =1;
%simulating turtlebot
ipaddress='192.168.122.129';
rosshutdown;
rosinit(ipaddress);
odom = rossubscriber('/gazebo/model_states');
odomdata = receive(odom,3);
pose = odomdata.Pose(10);
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = rad2deg(angles(3));
robotpose=[x,y,theta];
[mypub,cmdMsg]=rospublisher('/mobile_base/commands/velocity')
velmsg = rosmessage(robot);
%robotCurrentLocation=[x,y];
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10);
goalRadius = 0.1;
%robot = rospublisher('/mobile_base/commands/velocity');
%velmsg = rosmessage(robot);
theta=0
while( distanceToGoal > goalRadius ) 
    


odomdata = receive(odom,3);
pose = odomdata.Pose(10);
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = rad2deg(angles(3));
robotpose=[x,y,theta];
%[v,omega]= controller(robotpose);

[v,omega]=controller(robotpose);
%robot = rospublisher('/mobile_base/commands/velocity');

velmsg.Linear.X=v;

velmsg.Angular.Z=omega;

send(robot,velmsg);

updateCounter = updateCounter+1;
%Re-compute the distance to the goal
distanceToGoal = norm([x,y] - robotGoal);
waitfor(controlRate);

end
