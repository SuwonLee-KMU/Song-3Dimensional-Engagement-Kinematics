clear all; clc;

position = [0,0,0];
speed = 100;
gamma = 0;
chi = 45;
missile = flightVehicle(position,speed,chi,gamma)
position = [1000,000,1000];
speed = 10;
gamma = 0;
chi = 0;
target = flightVehicle(position,speed,chi,gamma)
kinematics = vehicleKinematics(missile,target)

missile2 = missile.duplicate

%%
tic
timeser = propagator.ODERK4([0,10],missile,target)
toc
