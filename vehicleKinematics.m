% Generated on: 190820
% Last modification: 190820
% Author: Suwon Lee
% Reference:  Seong-Ho Song and In-Joong Ha, ?�A Lyapunov-like approach to performance analysis of 3-dimensional pure PNG laws,?? IEEE Trans. Aerosp. Electron. Syst., vol. 30, no. 1, pp. 238??248, 1994.
%             http://ieeexplore.ieee.org/document/250424/

classdef vehicleKinematics < handle
  properties
    missile           % pursuer, flightVehicle class instance
    target            % target, flightVehicle class instance
  end
  properties (Transient)
    range             % range between target and missile
    LOSazimuth
    LOSelevation
    missileAzimuth
    missileElevation
    targetAzimuth
    targetElevation
    bodyGravity

    rangeDot;
    LOSazimuthDot
    LOSelevationDot
    missileAzimuthDot
    missileElevationDot
    targetAzimuthDot
    targetElevationDot
  end

  methods (Hidden)
    function obj = vehicleKinematics(missile,target)
      obj.missile = missile;
      obj.target  = target;
      obj.updateTransients;
      obj.updateDerivatives;
    end
  end

  methods
    function set.missile(obj,value)
      if isa(value,'flightVehicle')
        obj.missile = value;
      else
        error('invalid missile input');
      end
    end
    function set.target(obj,value)
      if isa(value,'flightVehicle')
        obj.target = value;
      else
        error('invalid target input');
      end
    end
  end

  methods % for computing auxiliary variables
    function [statesVector,inputVector] = obj2statesNinputs(obj) 
      Vt = obj.target.speed;
      tt = obj.targetElevation;
      pt = obj.targetAzimuth;
      Vm = obj.missile.speed;
      tm = obj.missileElevation;
      pm = obj.missileAzimuth;
      tL = obj.LOSelevation;
      pL = obj.LOSazimuth;
      r  = obj.range;
      Azm = obj.missile.Az;
      Aym = obj.missile.Ay;
      Azt = obj.target.Az;
      Ayt = obj.target.Ay;
      statesVector = [r, tL, pL, Vm, tm, pm, Vt, tt, pt];
      inputVector  = [Azm,Aym, Azt,Ayt];
    end
    function updateTransients(obj)
      relPos        = obj.target.position-obj.missile.position;
      range         = norm(relPos);
      LOSazimuth_   = atan2(relPos(2),relPos(1));
      LOSelevation_ = atan2(relPos(3),norm(relPos(1:2)));
      [missileElevation_,missileAzimuth_] = obj.getRelativeAngles(obj.missile.chi*pi/180,obj.missile.gamma*pi/180,LOSazimuth_,LOSelevation_);
      [targetElevation_,targetAzimuth_]   = obj.getRelativeAngles(obj.target.chi*pi/180,obj.target.gamma*pi/180,LOSazimuth_,LOSelevation_);

      obj.range             = range;
      obj.LOSazimuth        = 180/pi*LOSazimuth_;
      obj.LOSelevation      = 180/pi*LOSelevation_;
      obj.missileAzimuth    = 180/pi*missileAzimuth_;
      obj.missileElevation  = 180/pi*missileElevation_;
      obj.targetAzimuth     = 180/pi*targetAzimuth_;
      obj.targetElevation   = 180/pi*targetElevation_;
      obj.bodyGravity       = obj.gravInBody;
    end
    function [tL,pL,tm,pm] = shortRepresentation(obj)
      tL    = obj.LOSelevation;
      pL    = obj.LOSazimuth;
      tm    = obj.missileElevation;
      pm    = obj.missileAzimuth;
    end
    function g_body = gravInBody(obj)
      [tL,pL,tm,pm] = obj.shortRepresentation;
      gx = -cosd(tm)*cosd(pm)*sind(tL) - sind(tm)*cosd(tL);
      gy = -sind(pm)*sin(tL);
      gz = sind(tm)*cosd(pm)*sind(tL) - cosd(tm)*cosd(tL);
      g_body = [gx;gy;gz]*9.801;
    end

    function conditions = stopCond(obj) % develop simulation stop condition
      [SK,IK] = obj.obj2statesNinputs;
      dX      = obj.dynamics(SK,IK);
      rdot = dX(1);
      r    = obj.range;

      conditions = struct('condName',{'interception'},'status',{false});
      if all([rdot>0,r<20])
        conditions.status(1) = true;
      end
      if obj.missile.speed < 100
        conditions.status(2) = true;
      end
    end
    
    function dX = dynamics(obj,varargin)
      if nargin == 1
        r = obj.range;
        tL = obj.LOSelevation;
        pL = obj.LOSazimuth;
        tm = obj.missileElevation;
        pm = obj.missileAzimuth;
        tt = obj.targetElevation;
        pt = obj.targetAzimuth;
        Vm = obj.missile.speed;
        Vt = obj.target.speed;
        Azm = obj.missile.Az;
        Aym = obj.missile.Ay;
        Azt = obj.target.Az;
        Ayt = obj.target.Ay;
      else
        statesVector = varargin{1};
        inputVector  = varargin{2};
        r = statesVector(1);
        tL = statesVector(2);
        pL = statesVector(3);
        Vm = statesVector(4);
        tm = statesVector(5);
        pm = statesVector(6);
        Vt = statesVector(7);
        tt = statesVector(8);
        pt = statesVector(9);
        Azm = inputVector(1);
        Aym = inputVector(2);
        Azt = inputVector(3);
        Ayt = inputVector(4);
      end

      gb    = obj.gravInBody;
      rdot  = Vt*cosd(tt)*cosd(pt)-Vm*cosd(tm)*cosd(pm);
      tLdot = 1/r*(Vt*sind(tt)-Vm*sind(tm));
      pLdot = 1/(r*cosd(tL))*(Vt*cosd(tt)*sind(pt)-Vm*cosd(tm)*sind(pm));
      tmdot = (Azm+gb(3))/Vm-pLdot*sind(tL)*sind(pm)-tLdot*cosd(pm);
      pmdot = (Aym+gb(2))/(Vm*cosd(tm))+pLdot*tand(tm)*cosd(pm)*sind(tL)-tLdot*tand(tm)*sind(pm)-pLdot*cosd(tL);
      ttdot = (Azt+gb(3))/Vt-pLdot*sind(tL)*sind(pt)-tLdot*cosd(pt);
      ptdot = (Ayt+gb(2))/(Vt*cosd(tt))+pLdot*tand(tt)*cosd(pt)*sind(tL)-tLdot*tand(tt)*sind(pt)-pLdot*cosd(tL);
      Vmdot = obj.missile.speedDot;
      Vtdot = obj.target.speedDot;
      dX = [rdot,tLdot,pLdot,Vmdot,tmdot,pmdot,Vtdot,ttdot,ptdot];
    end
    function updateDerivatives(obj)
      dX = obj.dynamics;
      obj.rangeDot            = dX(1);
      obj.LOSelevationDot     = dX(2);
      obj.LOSazimuthDot       = dX(3);
      obj.missileElevationDot = dX(5);
      obj.missileAzimuthDot   = dX(6);
      obj.targetElevationDot  = dX(8);
      obj.targetAzimuthDot    = dX(9);
    end
  end

  methods (Static)
    function rotationMatrix = rot(axisNum,angle)  % inputs in degree
      switch axisNum
        case 1
          R = [1,0,0; 0,cos(angle),-sin(angle); 0,sin(angle),cos(angle)];
        case 2
          R = [cos(angle),0,sin(angle);0,1,0;-sin(angle),0,cos(angle)];
        case 3
          R = [cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
        otherwise
          error('invalid axisNum');
      end
      rotationMatrix = R;
    end
    function [theta,psi] = getRelativeAngles(chi,gamma,LOSazimuth,LOSelevation) % inputs in degree
      RthetaL = vehicleKinematics.rot(2,LOSelevation);
      RpsiL   = vehicleKinematics.rot(3,LOSazimuth);
      Rgamma  = vehicleKinematics.rot(2,gamma);
      Rchi    = vehicleKinematics.rot(3,chi);
      R     = Rgamma*Rchi/(RthetaL*RpsiL);
      theta = atan2(R(1,3),R(3,3));
      psi   = atan2(R(2,1),R(2,2));
    end
  end
end