function plotEvalStatesOverTime(log)
    
%     set(0,'defaulttextinterpreter','latex')
    
    % Data
    time = [3.5:0.05:6];
    x0 = nan(length(time),6);
    u1 = nan(length(time),2);
    k = 1;
    for i = 69:1:119
        x0(k,:) = log.vehicles{1,1}.currLapLog(i).x0';
        u1(k,:) = log.vehicles{1,1}.currLapLog(i).u(:,1)';
        k = k + 1;
    end
    Vx = x0(:,3);
    Vy = x0(:,4);
    W = x0(:,6);
    Delta = u1(:,1);
    T = u1(:,2);
    
    % Plot
    fig = figure;
    set(fig,'color',[1 1 1]);
    set(0,'DefaultFigureColor',[1 1 1]);
    set(fig,'defaultAxesColorOrder',[[0 0 0]; [0 0 0]]);
    
    % Velocities
    ax = subplot(2,1,1);
    yyaxis left
    plot(ax,time,Vx,'--k')
    hold(ax,'on')
    plot(ax,time,Vy,':k')
    ylim(ax,[-2 3])
    ylabel(ax,'Velocity [m/s]','interpreter','latex','FontSize',11)
    yyaxis right
    plot(ax,time,W,'-k')
    ylim(ax,[-2*pi 3*pi])
    yticks([-2*pi -1*pi 0 pi 2*pi 3*pi])
    yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
    ylabel(ax,'Yaw Rate [1/s]','interpreter','latex','FontSize',11)
    grid on
    leg = legend(ax,'$v_x$','$v_y$','$\omega$','Location', 'NorthWest');
    set(leg,'interpreter','latex','FontSize',11);
    set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
    % Delta and T
    ax = subplot(2,1,2);
    yyaxis left
    plot(ax,time,Delta,'--k')
    ylim(ax,[-0.1 0.5])
    ylabel(ax,'Angle [rad]','interpreter','latex','FontSize',11)
    yyaxis right
    plot(ax,time,T,'-k')
    hold on
    plot(ax,[3.9 3.9],[-0.02, 0.1],':k') % Straight vertical line
    hold on
    plot(ax,[4.55 4.55],[-0.02, 0.1],':k') % Straight vertical line
    hold on
    plot(ax,[5.4 5.4],[-0.02, 0.1],':k') % Straight vertical line
    ylim([-0.02 0.1])
%     yticks([-pi 0 2*pi 4*pi])
%     yticklabels({'$-\pi$','$0$','$2\pi$','$4\pi$'})
%     set(gca,'ytick',round([-pi:pi:5*pi],2))
    ylabel(ax,'Torque [Nm]','interpreter','latex','FontSize',11)
    xlabel('Time [s]','interpreter','latex','FontSize',11)
    grid on
    leg = legend(ax,'$\delta$','$T$','Location', 'NorthWest');
    set(leg,'interpreter','latex','FontSize',11);
    set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
    print -depsc 190316ExtremeDriftStatesNInputs
    

    
end

