% Generated on: 190820
% Last modification: 190820
% Author: Suwon Lee

classdef flightVehicle < handle
  properties (SetObservable)
    position  % in ENU coordinate, [m]
    speed     % aligned with body x-axis, [m/s]
    gamma     % upward angle from EN-plane, [deg]
    chi       % CCW-angle from E-axis direction, [deg]
              % the rotation order: 3-2(-1), the bank is not considered.
    Az = 0    % acceleration in body z-direction, [m/s^2]
    Ay = 0    % acceleration in body y-direction, [m/s^2]  
    DragAccFcnHandle = @(FV) DRAG(FV,0) % default : no drag model
  end
  properties(Transient)
    positionDot
    speedDot
    gammaDot
    chiDot
  end

  methods (Hidden)
    function obj = flightVehicle(position,speed,gamma,chi)
      obj.position  = position;
      obj.speed     = speed;
      obj.gamma     = gamma;
      obj.chi       = chi;
      obj.attachListner;
      obj.updateDerivatives;
    end
    function attachListner(obj)
      addlistener(obj,'position','PostSet',@flightVehicle.propChange);
      addlistener(obj,'speed','PostSet',@flightVehicle.propChange);
      addlistener(obj,'gamma','PostSet',@flightVehicle.propChange);
      addlistener(obj,'chi','PostSet',@flightVehicle.propChange);
      addlistener(obj,'Az','PostSet',@flightVehicle.propChange);
      addlistener(obj,'Ay','PostSet',@flightVehicle.propChange);
      addlistener(obj,'DragAccFcnHandle','PostSet',@flightVehicle.propChange);
    end
  end

  methods
    function set.chi(obj,value)
      angle = wrapTo180(value);
      obj.chi = angle;
    end
    function set.gamma(obj,value)
      angle = wrapTo180(value);
      obj.gamma = angle;
    end
    function updateDerivatives(obj)
      dragAcc = obj.DragAccFcnHandle(obj);
      S       = [obj.position(:)',obj.speed,obj.gamma,obj.chi];
      I       = [obj.Az,obj.Ay];
      dX      = vehicleDynamics(S,I,dragAcc);
      obj.positionDot = dX(1:3);
      obj.speedDot    = dX(4);
      obj.gammaDot    = dX(5);
      obj.chiDot      = dX(6);
    end

    function newObj = duplicate(obj)
      newObj    = flightVehicle(obj.position,obj.speed,obj.chi,obj.gamma);
      newObj.Ay = obj.Ay;
      newObj.Az = obj.Az;
    end
    function [statesVector,inputVector,statesDotVector] = obj2statesNinputs(obj)
      statesVector    = [obj.position(:)',obj.speed,obj.gamma,obj.chi];
      statesDotVector = [obj.positionDot(:)',obj.speedDot, obj.gammaDot,obj.chiDot];
      inputVector     = [obj.Az,obj.Ay];
    end

    function updateFromStates(obj,statesVector,inputVector)
      obj.position = statesVector(1:3);
      obj.speed    = statesVector(4);
      obj.gamma    = statesVector(5);
      obj.chi      = statesVector(6);
      obj.Az = inputVector(1);
      obj.Ay = inputVector(2);
    end
  end

  methods (Static, Hidden)  % For event listner callback
    function propChange(metaProp,eventData)
       h = eventData.AffectedObject;
       propName = metaProp.Name;
       % disp(['The ',propName,' property has changed.'])
       h.updateDerivatives();
    end
  end
end