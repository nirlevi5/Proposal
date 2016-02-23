close all ; clear all ; clc

fig = figure(1)

% ax = axis()
% axis(ax,[-0.3 1.3 -0.7 0.7])
grid on;
axis equal
hold on

% plot table
corners.x = [0 1 1 0 ];
corners.y = [0.4 0.4 -0.4 -0.4];
% patch(corners.x(:),corners.y(:),[1 1 1 1])

% define the robot parameters and initiate the object
% arms 1
links_length    = [0.6];
joints_value    = [0];
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.3;
base_pose.y     = 0.4;
base_pose.yaw   = -pi/2;
arms            = RobotArmMultiLink(links_length,joints_value,base_pose,joints_lim);

% arm 2
links_length    = [0.6];
joints_value    = [0];
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.4;
base_pose.y     = -0.4;
base_pose.yaw   = pi/2;
arms(end+1)     = RobotArmMultiLink(links_length,joints_value,base_pose,joints_lim);

% initiate the object
ma = MultiArm(arms);
% plot configuration
ma.plotAllArms;

% iniitiate task
task.armID                  = 1;
task.target_joints_value    = 16*pi/50;
task.object_position        = ma.arms(task.armID).FK(task.target_joints_value);

% find a solution
ma.GenerateMotionPlan(task);

% plot results on configuration space
ma.generate_configuration_space();
ma.plot2Dsolution();



