clear all ; clf;

fig = figure(1)

ax = axis()
axis(ax,[-0.3 1.3 -0.7 0.7])
grid on;
axis equal
hold on

% plot table
corners.x = [0 1 1 0 ];
corners.y = [0.4 0.4 -0.4 -0.4];
patch(corners.x(:),corners.y(:),[1 1 1 1])

% define the robot parameters and initiate the object
links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose1.x    = 0.0;
base_pose1.y    = -0.2;
base_pose1.yaw  = pi/2;
arm1            = RoboticArmSingleLink(links_length,joints_value,base_pose1,joints_lim);


% da              = DualArm2D(arm1,arm2);
% da.plotDualArm();
% 
% goal            = [0.85 0 0 0.55 0 0];
% proximity       = 0.1;
% angle_res       = 0.1;
% da.FindPathAstarJoints(goal,proximity,angle_res)

