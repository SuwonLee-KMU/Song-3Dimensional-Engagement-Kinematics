% Generated on: 190823
% Last modification: 190823
% Author: Suwon Lee from Seoul National University

function dragAcc = DRAG(flightVehicleObj,rho0)
  V       = flightVehicleObj;
%   rho0    = 1e-4;
  dragAcc = rho0*V.speed^2;
end