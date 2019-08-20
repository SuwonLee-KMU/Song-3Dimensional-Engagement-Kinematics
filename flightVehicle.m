% Generated on: 190820
% Last modification: 190820
% Author: Suwon Lee

classdef flightVehicle < handle
  properties
    position  % in ENU coordinate, [m]
    speed     % aligned with body x-axis, [m/s]
    chi       % CCW-angle from E-axis direction, [deg]
    gamma     % upward angle from EN-plane, [deg]
              % the rotation order: 3-2(-1), the bank is not considered.
    Ay = 0    % acceleration in body y-direction, [m/s^2]
    Az = 0    % acceleration in body z-direction, [m/s^2]
  end

  methods (Hidden)
    function obj = flightVehicle(position,speed,chi,gamma)
      obj.position  = position;
      obj.speed     = speed;
      obj.chi       = chi;
      obj.gamma     = gamma;
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
  end
end