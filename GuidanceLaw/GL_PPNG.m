% Generated on: 190822
% Last modification: 190822
% Author: Suwon Lee from Seoul National University

function [Az,Ay] = GL_PPNG(kinematics)
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
  g_body  = kinematics.gravInBody;
  gy = g_body(2);
  gz = g_body(3);
  
  Ay_ = -N*Vm*lamyDot*sind(theta_m)*sind(-psi_m) + N*Vm*lamzDot*cosd(theta_m) + gy;
  Az_ = -N*Vm*lamyDot*cosd(psi_m) + gz;
  
  Ay_lim = 100;
  Az_lim = 100;
  
  Ay = min(max(Ay_,-Ay_lim),Ay_lim);
  Az = min(max(Az_,-Az_lim),Az_lim);
end