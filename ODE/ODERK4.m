% Generated on: 190822
% Last modification: 190822
% Author: Suwon Lee from Seoul National University
function simOut = ODERK4(timespan,timestep,missile,target,GLfcnHandle)
  K = vehicleKinematics(missile,target);
  M = K.missile;
  T = K.target;
  [Sm,Im] = missile.obj2statesNinputs;
  [St,It] = target.obj2statesNinputs;

  t = timespan(1):timestep:timespan(end);
  Sms    = zeros(numel(t),6);
  Sts    = zeros(numel(t),6);
  Ims    = zeros(numel(t),2);
  Its    = zeros(numel(t),2);
  SmDots = zeros(numel(t),6);
  StDots = zeros(numel(t),6);
  
  for i = 1:numel(t)
    Sms(i,:) = Sm;
    Sts(i,:) = St;
    Ims(i,:) = Im;
    Its(i,:) = It;

    missileDragAcc = M.DragAccFcnHandle(M);
    targetDragAcc  = T.DragAccFcnHandle(T);

    SmDots(i,:) = vehicleDynamics(Sm,Im,missileDragAcc);
    StDots(i,:) = vehicleDynamics(St,It,targetDragAcc);
    
    Sm = updateState_RK4(Sm,Im,timestep,missileDragAcc);
    St = updateState_RK4(St,It,timestep,targetDragAcc);

    [Az,Ay] = GLfcnHandle(K);
    Im = [Az,Ay];
    It = [0,0];

    K.missile.updateFromStates(Sm,Im);
    K.target.updateFromStates(St,It);
    K.updateTransients;
    conds = K.stopCond;
    if any(conds.status)
      break;
    end
  end
  t   = t(1:i);
  Sms = Sms(1:i,:);
  Sts = Sts(1:i,:);
  Ims = Ims(1:i,:);
  Its = Its(1:i,:);
  SmDots = SmDots(1:i,:);
  StDots = StDots(1:i,:);
  simOut = struct('t',t','Sms',Sms,'Ims',Ims,'SmDots',SmDots,'Sts',Sts,'Its',Its,'StDots',StDots);
end

function nxtState = updateState_RK4(crtState,ctrInput,timeStep,dragAcc)
  k1 = vehicleDynamics(crtState,ctrInput,dragAcc);
  k2 = vehicleDynamics(crtState+timeStep/2*k1,ctrInput,dragAcc);
  k3 = vehicleDynamics(crtState+timeStep/2*k2,ctrInput,dragAcc);
  k4 = vehicleDynamics(crtState+timeStep*k3,ctrInput,dragAcc);
  nxtState = crtState + timeStep/6*(k1+2*k2+2*k3+k4);
end