function [mat] = eul2mat(eul)
% eul: Euler angle in [psi, theta, phi] order
% mat: The rotation matrix generated from Euler angle
%*******************************************************************************************
% **************************** WRITE YOUR CODE HERE ****************************************

psi = eul(1);
theta = eul(2);
phi = eul(3);
% mat= [YOR CODE HERE];
mat = [cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-sin(psi)*cos(phi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
       cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
       -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
% **************************** WRITE YOUR CODE HERE *****************************************
%********************************************************************************************
end
