function [mat] = eul_jacobian(eul)
% eul: Euler angle in [psi, theta, phi] order
% mat: The jacobian matrix generated from Euler angle
%*******************************************************************************************
% **************************** WRITE YOUR CODE HERE ****************************************

psi = eul(1);
theta = eul(2);
phi = eul(3);
% mat= [YOR CODE HERE];
mat = [0 , sin(phi)/cos(theta), cos(phi)/cos(theta);
       0, cos(phi), -sin(phi);
       1, sin(phi)*tan(theta), cos(phi)*tan(theta)];
% **************************** WRITE YOUR CODE HERE *****************************************
%********************************************************************************************
end
