% Generated on: 190821
% Last modification: 190821
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
    function [tsc] = ODERK4(timespan,missile,target)
      [Sm,Im] = missile.obj2statesNinputs;
      [St,It] = target.obj2statesNinputs;
      t = timespan(1):propagator.timeStep:timespan(end);
      pm = timeseries(NaN*ones(numel(t),3),t,'name','missilePosition');
      vm = timeseries(NaN*ones(numel(t),3),t,'name','missileV_chi_gam');
      am = timeseries(NaN*ones(numel(t),2),t,'name','missileControl');
      pt = timeseries(NaN*ones(numel(t),3),t,'name','targetPosition');
      vt = timeseries(NaN*ones(numel(t),3),t,'name','targetV_chi_gam');
      at = timeseries(NaN*ones(numel(t),2),t,'name','targetControl');
      for i = 1:numel(t)
        time = t(i);
        pm = addsample(pm,'Data',Sm(1:3),'Time',time,'OverwriteFlag',true);
        vm = addsample(vm,'Data',Sm(4:6),'Time',time,'OverwriteFlag',true);
        am = addsample(am,'Data',Im(1:2),'Time',time,'OverwriteFlag',true);
        pt = addsample(pt,'Data',St(1:3),'Time',time,'OverwriteFlag',true);
        vt = addsample(vt,'Data',St(4:6),'Time',time,'OverwriteFlag',true);
        at = addsample(at,'Data',It(1:2),'Time',time,'OverwriteFlag',true);

        Sm = propagator.updateState_RK4(Sm,Im,missile);
        St = propagator.updateState_RK4(St,It,target);
        
        Im = [10,0];  % missile input vector
        It = [0,0];   % target input vector
      end
      tsc = tscollection({pm,vm,am,pt,vt,at},'name','simulation');
    end
  end
end
    