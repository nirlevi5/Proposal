
syms x y yaw x0 y0 psi0 q1 q2 q3 l1 l2 l3 p0 p1 p2 p3 real


TF = @(x,y,psi) [ cos(psi), -sin(psi), x*cos(psi) - y*sin(psi)
                  sin(psi),  cos(psi), y*cos(psi) + x*sin(psi)
                         0,         0,                      1 ];

p0 = [x0 ; y0 ]

tf2 = TF(x,y,0)*TF(0,0,yaw)*TF(-l3 , 0 ,0 );
p2 = tf2(1:2,3)

% calc q1
q11 = atan2(p2(2)-p0(2),p2(1)-p0(1)) - psi0;
q12 = acos((norm(p2-p0)^2 + l1^2 - l2^2) / (2*l1*norm(p2-p0)));
q1 = (q11+q12)
   
% calc q2 using the cosine axiome
q2 = -acos((norm(p2-p0)^2 - l1^2 - l2^2) / (2*l1*l2))

% q1 + q2 + q3 + psi0 = yaw
q3 = yaw - (psi0 + q1 + q2)

sol1 = [q1 q2 q3];

%for solution 2:
q12 = -acos((norm(p2-p0)^2 + l1^2 - l2^2) / (2*l1*norm(p2-p0)));
q1 = (q11+q12)
q2 = acos((norm(p2-p0)^2 - l1^2 - l2^2) / (2*l1*l2))
q3 = yaw - (psi0 + q1 + q2)

sol2 = [q1 q2 q3];


Q = matlabFunction(sol1,sol2,'file','IK','vars',{[x0 y0 psi0],[l1 l2 l3],[x y yaw]})

X0 = [ arm1.base_pose.x arm1.base_pose.y arm1.base_pose.yaw];
L = [arm1.links_length(1) arm1.links_length(2) arm1.links_length(3)];
X = [0.4 -0.3 0];
% x0 = arm1.base_pose.x;
% y0 = arm1.base_pose.y;
% psi0 = arm1.base_pose.yaw;
% l1 = arm1.links_length(1);
% l2 = arm1.links_length(2);
% l3 = arm1.links_length(3);
% x = 0.4;
% y = -0.3;
% yaw = 0;

arm1.plotArm(IK(X0,L,X))


% IK([arm1.base_pose.x,arm1.base_pose.y,arm1.base_pose.yaw],arm1.links_length,[0.55 0.3 0])
            
            % p0 = [x0 ; y0 ]
% 
% tf1 = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1);
% p1  = tf1(1:2,3)
% 
% tf2 = tf1*TF(l2,0,q2);
% p2  = tf2(1:2,3)
% 
% tf3 = tf2*TF(l3,0,q3);
% p3  = tf3(1:2,3)
% 
% sinq3 = simplify(cross([p2-p1;0],[p3-p2;0])/(l2*l3) )



% solve(x == l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3),...
%       y == l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3),...
%       psi == q1+q2+q3,q1,q2,q3)