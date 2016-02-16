close all; clear all ; clc

syms a b real

S = solve(a*(1-exp(-0.1/b))==2,a*(1-exp()-0.2/b)==0.5)


% syms x j1 j2 vin vout
% 
% mat = [1+2i , -i ;-i , (2-x)*i]
% V = [vin ; 0]
% 
% J = inv(mat)*V
%  
% S = solve(vout == -j2*x*i,vin == vout/2,j1 == J(1),j2==J(2))
% 
% S.x