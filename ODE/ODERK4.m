% Generated on: 190822
% Last modification: 190822
% Author: Suwon Lee from Seoul National University
function [t,Sms,Ims,Sts,Its] = ODERK4(timespan,timestep,missile,target,GLfcnHandle)
  K = vehicleKinematics(missile,target);
  [Sm,Im] = missile.obj2statesNinputs;
  [St,It] = target.obj2statesNinputs;

  t = timespan(1):timestep:timespan(end);
  Sms = zeros(numel(t),6);
  Sts = zeros(numel(t),6);
  Ims = zeros(numel(t),2);
  Its = zeros(numel(t),2);
  for i = 1:numel(t)
    Sms(i,:) = Sm;
    Sts(i,:) = St;
    Ims(i,:) = Im;
    Its(i,:) = It;
    
    Sm = updateState_RK4(Sm,Im,timestep);
    St = updateState_RK4(St,It,timestep);
    
%     [Az,Ay] = GL_PPNG(K);
    [Az,Ay] = GLfcnHandle(K);
    Im = [Az,Ay];
    It = [0,0];

    K.missile.updateFromStates(Sm,Im);
    K.target.updateFromStates(St,It);
    K.updateTransients;
  end
end

function nxtState = updateState_RK4(crtState,ctrInput,timeStep)
  k1 = vehicleDynamics(crtState,ctrInput);
  k2 = vehicleDynamics(crtState+timeStep/2*k1,ctrInput);
  k3 = vehicleDynamics(crtState+timeStep/2*k2,ctrInput);
  k4 = vehicleDynamics(crtState+timeStep*k3,ctrInput);
  nxtState = crtState + timeStep/6*(k1+2*k2+2*k3+k4);
end

function dX = vehicleDynamics(statesVector,inputVector)
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
  Vdot = 0;
  if V == 0
    gamdot = 0;
    chidot = 0;
  else
    gamdot = Az/V*180/pi;
    chidot = Ay/V*180/pi;
  end
  dX = [Edot,Ndot,Udot,Vdot,gamdot,chidot];
end