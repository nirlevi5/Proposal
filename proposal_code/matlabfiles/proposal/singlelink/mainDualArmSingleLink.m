clear all ; clf;

fig = figure(1)

ax = axis()
axis(ax,[-0.5 1.5 -0.7 0.7])
grid on;
axis equal
hold on

% plot table
% corners.x = [0 1 1 0 ];
% corners.y = [0.4 0.4 -0.4 -0.4];
% patch(corners.x(:),corners.y(:),[1 1 1 1])

% define the robot parameters and initiate the object
links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose.x     = -0.1;
base_pose.y     = 0;
base_pose.yaw   = -pi/(pi*rand);
arms            = RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim);

links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.3;
base_pose.y     = 0.35;
base_pose.yaw   = -pi/(pi*rand);
arms            = [ arms ; RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)];

links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.3;
base_pose.y     = -0.35;
base_pose.yaw   = -pi/(pi*rand);
arms            = [ arms ; RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)];

links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.7;
base_pose.y     = 0.2;
base_pose.yaw   = -pi/(pi*rand);
arms            = [ arms ; RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)];

links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.7;
base_pose.y     = -0.2;
base_pose.yaw   = -pi/(pi*rand);
arms            = [ arms ; RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)];

links_length    = 0.3;
joints_value    = 0;
joints_lim      = [ -pi  pi ];
base_pose.x     = 1.1;
base_pose.y     = 0;
base_pose.yaw   = -pi/(pi*rand);
arms            = [ arms ; RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)];

% links_length    = 0.3;
% joints_value    = 0;
% joints_lim      = [ -pi  pi ];
% base_pose.x     = 0.65;
% base_pose.y     = -0.2;
% base_pose.yaw   = pi/2;
% arms            = [ arms ; RoboticArmSingleLink(links_length,joints_value,base_pose,joints_lim)];


da              = MultiArmSingleLink(arms);
da.plotAllArms();
da.plotAllWorkspaces();
% 

task.init       = [-0.2   ; -0.2];
task.target     = [0.8 ; -0.2];

% task_fig = plot(task.init(1),task.init(2),'r*',task.target(1),task.target(2),'k*');

% Create plot
plot(task.init(1),task.init(2),'MarkerFaceColor',[0 1 1],'MarkerSize',15,'Marker','square',...
    'LineStyle','none',...
    'Color',[0 0 0]);

% Create plot
plot(task.target(1),task.target(2),'MarkerFaceColor',[1 0 0],'MarkerSize',15,'Marker','^',...
    'LineStyle','none',...
    'Color',[0 0 0]);

% task_fig(1) = plot(task.init(1),task.init(2),'b*',...
%     'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...'MarkerSize','1',...
%     'LineWidth',3,...
%     'Color',[1 1 1]);


proximity       = 0.02;
angle_res       = 0.1;
% da.FindPathMultiArm(goal,proximity,angle_res)
% da.plotPlan
da.calc_arms_sequence(task)
da.calc_transfer_points();
% da.plot_transfer_points(1);

