% Generated on: 190827
% Author: Suwon Lee from Seoul National University

function dX = vehicleDynamics(statesVector,inputVector,dragAcceleration)
  grav = 9.801*0;
  E = statesVector(1);
  N = statesVector(2);
  U = statesVector(3);
  V = statesVector(4);
  gam = statesVector(5);
  chi = statesVector(6);
  Az = inputVector(1);
  Ay = inputVector(2);
  
  Edot = V*cosd(gam)*cosd(chi);
  Ndot = V*cosd(gam)*sind(chi);
  Udot = V*sind(gam);
  Vdot = -grav*sind(gam) - dragAcceleration;
  if V == 0
    gamdot = 0;
    chidot = 0;
  else
    gamdot = (Az-grav*cosd(gam))/V*180/pi;
    chidot = Ay/V*180/pi;
  end
  dX = [Edot,Ndot,Udot,Vdot,gamdot,chidot];
end