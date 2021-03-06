clear all ; clc;

syms x y yaw x0 y0 psi0 q1 q2 q3 l1 l2 l3 p0 p1 p2 p3 real


TF = @(x,y,psi) [ cos(psi), -sin(psi), x*cos(psi) - y*sin(psi)
                  sin(psi),  cos(psi), y*cos(psi) + x*sin(psi)
                         0,         0,                      1 ];
p0 = [x0 y0] 

tf = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1);
p1 = simplify(tf(1:2,3))'

tf = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(l2,0,q2);
p2 = simplify(tf(1:2,3))'
                    
tf = TF(x0,y0,0)*TF(0,0,psi0)*TF(l1,0,q1)*TF(l2,0,q2)*TF(l3,0,q3);
p3 = simplify(tf(1:2,3))'

p = matlabFunction([p0 ; p1 ; p2 ; p3],'file','calcArmVertex','vars',{[x0 y0 psi0],[l1 l2 l3],[q1 q2 q3]})
