function hed = plotArm(q)

link1length = 0.3;
link2length = 0.3;
link3length = 0.1;

link1 = [0 , link1length 
         0 , 0           
         1 , 1];
link2 = [0 , link2length 
         0 , 0 
         1 , 1];
     
link3 = [0.15   0.1    0.1  0   0.1   0.1     0.15   
         0.025  0.025  0    0   0    -0.025  -0.025
         1      1      1    1   1     1       1    ]

R = @(psi)   [ cos(psi) -sin(psi) 0 ; sin(psi) cos(psi) 0 ; 0 0 1] 
T = @(tx,ty) [1 0 tx ; 0 1 ty ; 0 0 1]

tr1 = R(q(1));
tr2 = R(q(1))*T(link1length,0)*R(q(2));
tr3 = R(q(1))*T(link1length,0)*R(q(2))*T(link2length,0)*R(q(3));

arm1 = tr1*link1; 
arm2 = tr2*link2; 
arm3 = tr3*link3; 

hed(1) = plot(arm1(1,:),arm1(2,:),'MarkerFaceColor',[0 0 1],...
    'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
    'Marker','o',...
    'LineWidth',5,...
    'Color',[1 0 0]);

hold on;

hed(2) = plot(arm2(1,:),arm2(2,:),'MarkerFaceColor',[0 0 1],...
    'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
    'Marker','o',...
    'LineWidth',5,...
    'Color',[1 0 0]);
 

hed(3) = plot(arm3(1,:),arm3(2,:),'MarkerFaceColor',[0 0 1],...
    'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
    'Marker','o',...
    'LineWidth',5,...
    'Color',[1 0 0]);

axis equal
 
