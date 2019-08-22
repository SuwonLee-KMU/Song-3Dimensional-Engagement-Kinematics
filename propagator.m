% Generated on: 190821
% Last modification: 190822
% Author: Suwon Lee
classdef propagator
  properties (Constant)
    timeStep = 1e-2;
  end

  methods (Static)
    function nxtStates = updateState_RK4(statesVector,inputVector,whichClass)
      timeStep = propagator.timeStep;
      k1 = whichClass.dynamics(statesVector,inputVector);
      k2 = whichClass.dynamics(statesVector+timeStep/2*k1,inputVector);
      k3 = whichClass.dynamics(statesVector+timeStep/2*k2,inputVector);
      k4 = whichClass.dynamics(statesVector+timeStep*k3,inputVector);
      nxtStates = statesVector + timeStep/6*(k1+2*k2+2*k3+k4);
    end
    function [t,Sms,Ims,Sts,Its] = ODERK4(timespan,missile,target)
      [Sm,Im] = missile.obj2statesNinputs;
      [St,It] = target.obj2statesNinputs;
      t = timespan(1):propagator.timeStep:timespan(end);
      Sms = zeros(numel(t),6);
      Sts = zeros(numel(t),6);
      Ims = zeros(numel(t),2);
      Its = zeros(numel(t),2);
      for i = 1:numel(t)
        Sms(i,:) = Sm;
        Sts(i,:) = St;
        Sm = propagator.updateState_RK4(Sm,Im,missile);
        St = propagator.updateState_RK4(St,It,target);
        Im = [1000,100];  % missile input vector
        It = [100,100];   % target input vector
      end
    end
  end
end
    