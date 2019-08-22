% Generated on: 190822
% Last modification: 190822
% Author: Suwon Lee from Seoul National University

classdef painter < handle
  properties
    time
    Sms
    Ims
    Sts
    Its
  end

  methods (Hidden)
    function obj = painter(time,missileStates,missileInputs,targetStates,targetInputs)
      obj.time = time;
      obj.Sms = missileStates;
      obj.Ims = missileInputs;
      obj.Sts = targetStates;
      obj.Its = targetInputs;
    end
    function [t,Em,Nm,Um,Vm,gm,cm,Et,Nt,Ut,Vt,gt,ct] = demuxStates(obj)
      t  = obj.time;
      Em = obj.Sms(:,1);
      Nm = obj.Sms(:,2);
      Um = obj.Sms(:,3);
      Vm = obj.Sms(:,4);
      gm = obj.Sms(:,5);
      cm = obj.Sms(:,6);
      Et = obj.Sts(:,1);
      Nt = obj.Sts(:,2);
      Ut = obj.Sts(:,3);
      Vt = obj.Sts(:,4);
      gt = obj.Sts(:,5);
      ct = obj.Sts(:,6);
    end
    function [Azm,Aym,Azt,Ayt] = demuxInputs(obj)
      Azm = obj.Ims(:,1);
      Aym = obj.Ims(:,2);
      Azt = obj.Its(:,1);
      Ayt = obj.Its(:,2);
    end
    function [r,tL,pL,Vm,tm,pm,Vt,tt,pt] = demuxLOSstates(obj)
      M = flightVehicle(obj.Sms(1,1:3),obj.Sms(1,4),obj.Sms(1,5),obj.Sms(1,6));
      T = flightVehicle(obj.Sts(1,1:3),obj.Sts(1,4),obj.Sts(1,5),obj.Sts(1,6));
      K = vehicleKinematics(M,T);
      ndata = numel(obj.time);
      r = zeros(ndata,1);
      tL = zeros(ndata,1);
      pL = zeros(ndata,1);
      Vm = zeros(ndata,1);
      tm = zeros(ndata,1);
      pm = zeros(ndata,1);
      Vt = zeros(ndata,1);
      tt = zeros(ndata,1);
      pt = zeros(ndata,1);

      for i = 1:ndata
        M.position = obj.Sms(i,1:3);
        M.speed    = obj.Sms(i,4);
        M.gamma    = obj.Sms(i,5);
        M.chi      = obj.Sms(i,6);
        T.position = obj.Sts(i,1:3);
        T.speed    = obj.Sts(i,4);
        T.gamma    = obj.Sts(i,5);
        T.chi      = obj.Sts(i,6);
        K.updateTransients;
        
        r(i) = K.range;
        tL(i) = K.LOSelevation;
        pL(i) = K.LOSazimuth;
        Vm(i) = K.missile.speed;
        tm(i) = K.missileElevation;
        pm(i) = K.missileAzimuth;
        Vt(i) = K.target.speed;
        tt(i) = K.targetElevation;
        pt(i) = K.targetAzimuth;
      end
    end
  end

  methods 
    function fig = traj(obj,fignum)
      fig = figure(fignum);
      [t,Em,Nm,Um,Vm,gm,cm,Et,Nt,Ut,Vt,gt,ct] = obj.demuxStates;
      plot3(Em,Nm,Um,'linewidth',2,'color','k');
      hold on;
      plot3(Et,Nt,Ut,'linewidth',2,'color','r');
      scatter3(Em(1),Nm(1),Um(1),'marker','s','sizedata',200,'markerfacecolor','k');
      scatter3(Em(end),Nm(end),Um(end),'marker','d','sizedata',200,'markerfacecolor','k');
      scatter3(Et(1),Nt(1),Ut(1),'marker','o','sizedata',200,'markerfacecolor','r');
      hold off;
      xlabel('E [m]'); ylabel('N [m]'); zlabel('U [m]');
      grid on; box on; axis equal;
    end
    function fig = acc(obj,fignum)
      fig = figure(fignum);
      [t,Em,Nm,Um,Vm,gm,cm,Et,Nt,Ut,Vt,gt,ct] = obj.demuxStates;
      [Azm,Aym,Azt,Ayt] = obj.demuxInputs;
      plot(t,Azm,'linewidth',2);
      hold on;
      plot(t,Aym,'linewidth',2);
      hold off;
      grid on; box on;
    end
    function fig = LOS(obj,fignum,kindnumber)
      [r,tL,pL,Vm,tm,pm,Vt,tt,pt] = obj.demuxLOSstates;
      fig = figure(fignum);
      switch kindnumber
      case 1
        plot(obj.time,r,'linewidth',2);
        ylabel('r [m]');
      case 2
        p1 = plot(obj.time,tL,'linewidth',2);
        hold on;
        p2 = plot(obj.time,pL,'linewidth',2);
        hold off;
        legend([p1,p2],{'\theta_L','\psi_L'});
      case 3
        p1 = plot(obj.time,tm,'linewidth',2);
        hold on;
        p2 = plot(obj.time,pm,'linewidth',2);
        hold off;
        legend([p1,p2],{'\theta_m','\psi_m'});
      otherwise
        p1 = plot(obj.time,tt,'linewidth',2);
        hold on;
        p2 = plot(obj.time,pt,'linewidth',2);
        hold off;
        legend([p1,p2],{'\theta_t','\psi_t'});
      end
      grid on; box on;
    end
  end
end