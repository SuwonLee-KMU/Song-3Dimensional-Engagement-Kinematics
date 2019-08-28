% Last modification: 190823
% Author: Suwon Lee from Seoul National University
clear all; clc;
addpath(genpath('./'));

% Initialize missile, target, and kinematics objects
rho0 = 1e-4;
M = flightVehicle([0,0,0],200,10,45,'DragAccFcnHandle',@(FV)DRAG(FV,rho0));
T = flightVehicle([1000,0,700],0,0,0,'DragAccFcnHandle',@(FV)DRAG(FV,rho0));
K = vehicleKinematics(M,T);

% select function handles and simulate 
GLfcnHandle         = @GL_PPNG;
simout = ODERK4([0 9],0.01,M,T,GLfcnHandle);

% Initialize painter object
pnt = painter(simout);

set(0,'defaultaxesfontsize',20,'defaultaxeslinewidth',2);
fig1 = pnt.traj(1);
fig2 = pnt.acc(2);
fig3 = pnt.LOS(3,1);
fig4 = pnt.LOS(4,2);
fig5 = pnt.LOS(5,3);
fig6 = pnt.LOS(6,4);
fig7 = pnt.spd(7);

