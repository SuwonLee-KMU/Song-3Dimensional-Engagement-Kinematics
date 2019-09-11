% Generated on: 190822
% Last modification: 190822
% Author: Suwon Lee from Seoul National University

classdef painter < handle
  properties
    time
    Sms
    Ims
    SmDots
    Sts
    Its
    StDots
    tmd
    pmd
    r
    tL
    pL
    tm
    pm
  end

  methods (Hidden)
    function obj = painter(simout)
      obj.time = simout.t;
      obj.Sms = simout.Sms;
      obj.Ims = simout.Ims;
      obj.Sts = simout.Sts;
      obj.Its = simout.Its;
      obj.SmDots = simout.SmDots;
      obj.StDots = simout.StDots;
      obj.tmd = simout.tmds;
      obj.pmd = simout.pmds;
      obj.r = simout.r;
      obj.tL = simout.tL;
      obj.pL = simout.pL;
      obj.tm = simout.tm;
      obj.pm = simout.pm;
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
      p1 = plot(t,Azm,'linewidth',2);
      hold on;
      p2 = plot(t,Aym,'linewidth',2);
      hold off;
      grid on; box on;
      ylabel('Acelerations [m/s^2]');
      legend([p1,p2],{'A_{z}','A_{y}'},'fontsize',20);
    end
    function fig = LOS(obj,fignum,kindnumber)
      fig = figure(fignum);
      switch kindnumber
      case 1
        plot(obj.time,obj.r,'linewidth',2);
        ylabel('r [m]','fontsize',20);
      case 2
        p1 = plot(obj.time,obj.tL,'linewidth',2);
        hold on;
        p2 = plot(obj.time,obj.pL,'linewidth',2);
        hold off;
        legend([p1,p2],{'\theta_L','\psi_L'});
        ylabel('LOS Angles [deg]');
      case 3
        p1 = plot(obj.time,obj.tm,'linewidth',2);
        hold on;
        p2 = plot(obj.time,obj.pm,'linewidth',2);
        p1d =plot(obj.time,obj.tmd(:,1),'linestyle','--','linewidth',2,'color',p1.Color);
        p2d = plot(obj.time,obj.pmd(:,1),'linestyle',':','linewidth',2,'color',p2.Color);
        set(p1,'color',[p1.Color,0.2]);
        set(p2,'color',[p2.Color,0.2]);
        hold off;
        legend([p1,p2,p1d,p2d],{'\theta_m','\psi_m','\theta_m^d','\psi_m^d'});
        ylabel('Missile Attitude [deg]');
      otherwise
        p1 = plot(obj.time,tt,'linewidth',2);
        hold on;
        p2 = plot(obj.time,pt,'linewidth',2);
        hold off;
        legend([p1,p2],{'\theta_t','\psi_t'});
        ylabel('Target Attitude [deg]');
      end
      grid on; box on;
    end
    function fig = spd(obj,fignum)
      fig = figure(fignum);
      [t,Em,Nm,Um,Vm,gm,cm,Et,Nt,Ut,Vt,gt,ct] = obj.demuxStates;
      p1 = plot(t,Vm,'linewidth',2,'color','k');
      hold on;
      p2 = plot(t,Vt,'linewidth',2,'color','r');
      hold off;
      ylabel('Speed [m/s]');
      legend([p1,p2],{'V_m','V_t'});
      grid on; box on;
    end
  end
end