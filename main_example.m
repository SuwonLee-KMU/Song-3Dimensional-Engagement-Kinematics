clear all; clc;

position = [0,0,0];
speed = 100;
gamma = 0;
chi = 45;
missile = flightVehicle(position,speed,chi,gamma)
position = [1000,1000,1000];
speed = 10;
gamma = 0;
chi = 45;
target = flightVehicle(position,speed,chi,gamma)
kinematics = vehicleKinematics(missile,target)
% kinematics.updateTransients
%%
R1 = rot(2,2*pi)
check = abs(R1-eye(3))<1e-6
any(check(:))
%% methods
function rotMtx = rot(axisNum,angle)
  switch axisNum
    case 1
      R = [1,0,0; 0,cos(angle),-sin(angle); 0,sin(angle),cos(angle)];
    case 2
      R = [cos(angle),0,sin(angle);0,1,0;-sin(angle),0,cos(angle)];
    case 3
      R = [cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
    otherwise
      error('invalid axisNum');
  end
  rotMtx = R;
end