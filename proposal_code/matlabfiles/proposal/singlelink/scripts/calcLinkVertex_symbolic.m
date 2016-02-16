clear all ; clc;

syms x y yaw x0 y0 psi0 q1 q2 q3 l1 l2 l3 p0 p1 p2 p3 half_link_width real

TF = @(x,y,psi) [ cos(psi), -sin(psi), x*cos(psi) - y*sin(psi)
                  sin(psi),  cos(psi), y*cos(psi) + x*sin(psi)
                         0,         0,                      1 ];

L1      = TF(x0,y0,0)*TF(0,0,psi0)*TF(0,0,q1)*[-half_link_width;half_link_width;1]
L1(:,2) = TF(x0,y0,0)*TF(0,0,psi0)*TF(0,0,q1)*[l1 + half_link_width;half_link_width;1]
L1(:,3) = TF(x0,y0,0)*TF(0,0,psi0)*TF(0,0,q1)*[l1 + half_link_width;-half_link_width;1]
L1(:,4) = TF(x0,y0,0)*TF(0,0,psi0)*TF(0,0,q1)*[-half_link_width;-half_link_width;1]

L1 = L1(1:2,:)

L2      = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(0,0,q2)*[-half_link_width;half_link_width;1]
L2(:,2) = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(0,0,q2)*[l2 + half_link_width;half_link_width;1]
L2(:,3) = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(0,0,q2)*[l2 + half_link_width;-half_link_width;1]
L2(:,4) = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(0,0,q2)*[-half_link_width;-half_link_width;1]

L2 = L2(1:2,:)


L3      = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(l2,0,q2)*TF(0,0,q3)*[-half_link_width;half_link_width;1]
L3(:,2) = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(l2,0,q2)*TF(0,0,q3)*[l3 + half_link_width;half_link_width;1]
L3(:,3) = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(l2,0,q2)*TF(0,0,q3)*[l3 + half_link_width;-half_link_width;1]
L3(:,4) = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(l2,0,q2)*TF(0,0,q3)*[-half_link_width;-half_link_width;1]

L3 = L3(1:2,:)

matlabFunction(L1,L2,L3,'file','calcLinksVertex','vars',{[x0 y0 psi0],[l1 l2 l3],[q1 q2 q3],half_link_width})

