clear all ; clf;

fig = figure(1)

ax = axis()
axis(ax,[-0.3 1.3 -0.6 0.6])
grid on;
axis equal
hold on

% plot table
corners.x = [0 1 1 0 ];
corners.y = [0.4 0.4 -0.4 -0.4];
patch(corners.x(:),corners.y(:),[1 1 1 1])

% define the robot parameters
links_length    = [0.3 0.3 0.15];
joints          = [ 0 0 0 ];
joints_lim      = [ -pi  pi ];
base_pose1      = struct('x',0.1,'y',-0.1,'yaw',-0.5);

% initiate the arm object
arm1 = RoboticArm2D(links_length,joints,base_pose1,joints_lim);
arm1.plotArm();

tic
% find path with Astar on the cartesian space
desired_pose = [0.3 0.3 pi];
proximity       = 0.05;
pose_res        = 0.05;
angle_res       = 0.05;
arm1.FindPathAstarPose(desired_pose,proximity,pose_res,angle_res)
arm1.plotPosePath();
t1 = toc;
tic
% find path with Astar on the joint space
desired_pose    = [0.3 0.3 pi];
proximity       = 0.05;
angle_res       = 0.05;
arm1.FindPathAstarJoints(desired_pose,proximity,angle_res)
arm1.plotJointPath();
t2 = toc;