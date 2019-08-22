% Generated on: 190822
% Last modification: 190822
% Author: Suwon Lee from Seoul National University

function [Az,Ay] = GL_test(kinematics)
  N     = 3;  % navigation constant
  Vm    = kinematics.missile.speed;
  [S,I] = kinematics.obj2statesNinputs;
  tL    = kinematics.LOSelevation;
  dX    = kinematics.dynamics(S,I);
  tLDot = dX(2);
  pLDot = dX(3);
  lamyDot = -tLDot;
  lamzDot = pLDot*cosd(tL);
  theta_m = kinematics.missileElevation; % degree
  psi_m   = kinematics.missileAzimuth;   % degree

  Ay = -N*Vm*lamyDot*sind(theta_m)*sind(-psi_m) + N*Vm*lamzDot*cosd(theta_m);
  Az = -N*Vm*lamyDot*cosd(psi_m);
end