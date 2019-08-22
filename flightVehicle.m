% Generated on: 190820
% Last modification: 190820
% Author: Suwon Lee

classdef flightVehicle < handle
  properties
    position  % in ENU coordinate, [m]
    speed     % aligned with body x-axis, [m/s]
    gamma     % upward angle from EN-plane, [deg]
    chi       % CCW-angle from E-axis direction, [deg]
              % the rotation order: 3-2(-1), the bank is not considered.
    Az = 0    % acceleration in body z-direction, [m/s^2]
    Ay = 0    % acceleration in body y-direction, [m/s^2]
  end

  methods (Hidden)
    function obj = flightVehicle(position,speed,gamma,chi)
      obj.position  = position;
      obj.speed     = speed;
      obj.gamma     = gamma;
      obj.chi       = chi;
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

    function newObj = duplicate(obj)
      newObj    = flightVehicle(obj.position,obj.speed,obj.chi,obj.gamma);
      newObj.Ay = obj.Ay;
      newObj.Az = obj.Az;
    end
    function [statesVector,inputVector] = obj2statesNinputs(obj)
      statesVector = [obj.position(:)',obj.speed,obj.gamma,obj.chi];
      inputVector  = [obj.Az,obj.Ay];
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

  methods (Static)
    function dX = dynamics(statesVector,inputVector)
      E = statesVector(1);
      N = statesVector(2);
      U = statesVector(3);
      V = statesVector(4);
      gam = statesVector(5);
      chi = statesVector(6);
      Az = inputVector(1);
      Ay = inputVector(2);

      Edot = V*cosd(gam)*sind(chi);
      Ndot = V*cosd(gam)*cosd(chi);
      Udot = V*sind(gam);
      Vdot = 0;
      gamdot = Az/V*180/pi;
      chidot = Ay/V*180/pi;
      dX = [Edot,Ndot,Udot,Vdot,gamdot,chidot];
    end
  end
end