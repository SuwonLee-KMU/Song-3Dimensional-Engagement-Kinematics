clear all; clc;
addpath(genpath('./'));
M = flightVehicle([0,0,0],1,10,45);
T = flightVehicle([100,00,70],0,5,0);
K = vehicleKinematics(M,T);

[t,Sms,Ims,Sts,Its] = ODERK4([0 140],0.01,M,T);
pnt = painter(t,Sms,Ims,Sts,Its);
fig1 = pnt.traj(1);
fig2 = pnt.acc(2);
fig3 = pnt.LOS(3,1);
fig4 = pnt.LOS(4,2);
fig5 = pnt.LOS(5,3);
fig6 = pnt.LOS(6,4);