% Last modification: 190823
% Author: Suwon Lee from Seoul National University
clear all; clc;
addpath(genpath('./'));

% Initialize missile, target, and kinematics objects
M = flightVehicle([0,0,0],200,10,45);
T = flightVehicle([1000,0,700],0,0,0);
K = vehicleKinematics(M,T);

% select function handles and simulate
rho0 = 1e-3;
dragAccFcnHandle    = @(K) DRAG(K,rho0); 
GLfcnHandle         = @GL_PPNG;
[t,Sms,Ims,Sts,Its] = ODERK4([0 9],0.01,M,T,GLfcnHandle,dragAccFcnHandle);

% Initialize painter object
pnt = painter(t,Sms,Ims,Sts,Its);
%
set(0,'defaultaxesfontsize',20,'defaultaxeslinewidth',2);
fig1 = pnt.traj(1);
fig2 = pnt.acc(2);
fig3 = pnt.LOS(3,1);
fig4 = pnt.LOS(4,2);
fig5 = pnt.LOS(5,3);
fig6 = pnt.LOS(6,4);
fig7 = pnt.spd(7);

