clear all ; clf

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
links_length    = [0.3 0.3];
joints_value    = [-1 1];
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.5;
base_pose.y     = 0.4;
base_pose.yaw   = -pi/2;
arms            = RobotArmMultiLink(links_length,joints_value,base_pose,joints_lim);


links_length    = [0.6];
joints_value    = [0];
joints_lim      = [ -pi  pi ];
base_pose.x     = 0.5;
base_pose.y     = -0.4;
base_pose.yaw   = pi/2;
arms(end+1)     = RobotArmMultiLink(links_length,joints_value,base_pose,joints_lim);

ma = MultiArm(arms);
ma.plotAllArms;


% iniitiate task
task.armID                  = 1;
task.target_joints_value    = [1 -1];
task.object_position        = ma.arms(task.armID).FK(task.target_joints_value);

ma.GenerateMotionPlan(task);
% ma.plotPlan
% figure(4)
% load('Cspace.mat')
% contour3(J11,J12,Z,100)
% hold on;
% xlabel('j11');ylabel('j12');zlabel('j21');
% 
% target = [];
% a1.target = [1 -1];
% for kk = 1:length(j21)
%    target(end+1,:) = [a1.target , j21(kk)];  
% end
% 
% %plot initial conf
% plot3(ma.arms(1).joints_value(1),ma.arms(1).joints_value(2),ma.arms(2).joints_value(1),'bo','Markersize',15)
% %plot target
% plot3(target(:,1),target(:,2),target(:,3),'r');
% 
% 
% links_length    = [0.3 0.3];
% joints_value    = [-1.3 2.6];
% joints_lim      = [ -pi  pi ];
% base_pose.x     = 0.5;
% base_pose.y     = 0.4;
% base_pose.yaw   = -pi/2;
% arm3     = RobotArmMultiLink(links_length,joints_value,base_pose,joints_lim);


% plot break point
% plot3(arm3.joints_value(1),arm3.joints_value(2),ma.arms(2).joints_value(1),'mo','Markersize',15)



% j11 = linspace(-pi,pi,70);
% j12 = linspace(-pi,pi,70);
% j21 = linspace(-pi,pi,70);
% 
% [J11,J12] = meshgrid(j11,j12);
% Z = zeros(size(J11));
% b = 1;
% figure(3);
% hold on;
% Cspace = [];
% for ii = 1:length(j11)
%     for jj = 1:length(j12)
%         for kk = 1:length(j21)
%             if ma.CollisionCheck(1,2,[j11(ii),j12(jj)],j21(kk))
%                 Cspace(end+1,:) = [j11(ii),j12(jj),j21(kk)]
%                 Z(ii,jj) = j21(kk);
%             end
%             b = b+1
%         end
%     end
% end
% contour3(J11,J12,Z,100)

