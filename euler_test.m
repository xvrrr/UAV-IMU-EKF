% There is NOTHING you need to change in this file!
clear;
eul1 = [0.65, 0.832, 1.442];
eul2 = [0.81, pi/2, 0.54];

R_1 = eul2mat(eul1)
J_1 = eul_jacobian(eul1)
R_2 = eul2mat(eul2)
J_2 = eul_jacobian(eul2)
