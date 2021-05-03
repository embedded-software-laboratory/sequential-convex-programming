function plotEvalTimesOverX()
    
    fig = figure;
    set(fig,'color',[1 1 1]);
    set(0,'DefaultFigureColor',[1 1 1]);
    set(fig,'defaultAxesColorOrder',[[0 0 0]; [0 0 0]]);
    color100 = getColors(100);
    
    %% Data
    
    % (1) Experiment Series: Velocity Difference
    velDiff = [0.05	0.10 0.15 0.20 0.25 0.30 0.35 0.40 0.45 0.50];
    velDiff_LapT = [12.0207 12.0902 12.3691 12.1739 12.2199 12.1452 12.1668 12.3090 12.1765 12.1659];
    velDiff_CompTMean = [0.0158 0.0158 0.0162 0.0156 0.0157 0.0157 0.0157 0.0157 0.0156 0.0157];
    velDiff_CompTMax = [0.0470 0.0320 0.0320 0.0310 0.0310 0.0310 0.0310 0.0310 0.0160 0.0320];
    
    velDiff_PosChStep = [156 137 100 94 97 77 70]*0.05;
    velDiff_PosChCP = [579 415 292 238 241 111 94];
    
    % (2) Experiment Series: Position Difference
%     posDiff = [0.4 0.8 1.1 1.3 1.6 1.9 2.1 2.3 2.5 2.8 3.2 3.6];
%     posDiff_LapT = [12.1676 12.1706 12.2240 12.0740 12.2152 12.1062 12.1492 12.0200 12.0600 12.1724 12.0919 12.0519];
%     posDiff_CompTMean = [0.0156 0.0157 0.0158 0.0157 0.0156 0.0157 0.0156 0.0159 0.0157 0.0157 0.0162 0.0158];
%     posDiff_CompTMax = [0.0160 0.0310 0.0320 0.0320 0.0320 0.0310 0.0320 0.0320 0.0310 0.0310 0.0320 0.0320];
%     
%     posDiff_PosChStep = [50 75 96 92 119 122 139 138 150 178 196 209]*0.05;
%     posDiff_PosChCP = [41 106 244 229 351 365 446 457 560 659 723 828];
    
    % (3) Experiment Series: Multiple Vehicles
    multVeh = [1 2 3 4 5 6 7 8];
    multVeh_LapT = [11.9796 12.0673 12.0736 12.4235 12.8558 12.8603 13.0441 13.1290];
    multVeh_CompTMean = [0.0158 0.0158 0.0161 0.0164 0.0164 0.0174 0.0174 0.0182];
    multVeh_CompTMax = [0.0310 0.0320 0.0320 0.0320 0.0320 0.0320 0.0320 0.0320];
    
    %(4) Experiment Series: Velocity Difference incl. Blocking
    velDiffBlock = [0.20 0.25 0.30 0.35 0.40 0.45 0.50];
    velDiffBlock_LapT = [12.2250 12.4224 12.4224 12.1795 12.2737 12.1765 12.1659];
    velDiffBlock_CompTMean = [0.0190 0.0185 0.0190 0.0193 0.0193 0.0191 0.0201];
    velDiffBlock_CompTMax = [0.0320 0.0470 0.0470 0.0470 0.0320 0.0470 0.0470];
 
    velDiffBlock_PosChStep = [164 165 165 95 97 77 70]*0.05;
    velDiffBlock_PosChCP = [614 603 603 243 244 111 94];
    

    %% (1) Experiment Series: Velocity Difference
    
%     % Plot 1: Lap and Computation Times
%     yyaxis left
%     plot(velDiff,velDiff_LapT,'s:k')
%     ylim([11 13])
%     ylabel('Lap Time [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(velDiff,velDiff_CompTMean,'o--k')
%     hold('on')
%     plot(velDiff,velDiff_CompTMax,'*--k')
%     ylim([0.01 0.07])
% %     yticks([0.01 0.04 0.07])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Comp. Time [s]','interpreter','latex','FontSize',11)
%     xlabel('Difference in Maximum Velocity [m/s]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Lap Time','Mean Comp. Time','Max. Comp. Time','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
%     % Plot 2: Position Changes
%     yyaxis left
%     plot(velDiff(4:end),velDiff_PosChStep,'*:k')
%     ylim([0 13])
%     ylabel('Time Step [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(velDiff(4:end),velDiff_PosChCP,'o--k')
%     ylim([0 650])
% %     yticks([-2*pi -1*pi 0 pi 2*pi 3*pi])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Location [Track Checkpoint]','interpreter','latex','FontSize',11)
%     xlabel('Difference in Maximum Velocity [m/s]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Time Step of Position Change','Location of Position Change','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
    
    %% (2) Experiment Series: Position Difference
    
%     % Plot 1: Lap and Computation Times
%     yyaxis left
%     plot(posDiff,posDiff_LapT,'s:k')
%     ylim([11 13])
%     ylabel('Lap Time [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(posDiff,posDiff_CompTMean,'o--k')
%     hold('on')
%     plot(posDiff,posDiff_CompTMax,'*--k')
%     ylim([0.01 0.07])
% %     yticks([0.01 0.04 0.07])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Comp. Time [s]','interpreter','latex','FontSize',11)
%     xlabel('Longitudinal Difference in Start Position [m]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Lap Time','Mean Comp. Time','Max. Comp. Time','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
%     % Plot 2: Position Changes
%     yyaxis left
%     plot(posDiff,posDiff_PosChStep,'*:k')
%     ylim([0 13])
%     ylabel('Time Step [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(posDiff,posDiff_PosChCP,'o--k')
%     ylim([0 1000])
% %     yticks([-2*pi -1*pi 0 pi 2*pi 3*pi])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Location [Track Checkpoint]','interpreter','latex','FontSize',11)
%     xlabel('Longitudinal Difference in Start Position [m]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Time Step of Position Change','Location of Position Change','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
    
    %% (3) Experiment Series: Multiple Vehicles
    
%     % Plot 1: Lap and Computation Times
%     yyaxis left
%     plot(multVeh,multVeh_LapT,'s:k')
%     ylim([11 14])
%     ylabel('Lap Time [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(multVeh,multVeh_CompTMean,'o--k')
%     hold('on')
%     plot(multVeh,multVeh_CompTMax,'*--k')
%     ylim([0.01 0.07])
% %     yticks([0.01 0.04 0.07])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Comp. Time [s]','interpreter','latex','FontSize',11)
%     xlabel('Number of Vehicles','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Lap Time','Mean Comp. Time','Max. Comp. Time','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
    
    %% (4) Experiment Series: Velocity Difference incl. Blocking

%     % Plot 1: Lap and Computation Times
%     yyaxis left
%     plot(velDiffBlock,velDiffBlock_LapT,'s:k')
%     ylim([11 13])
%     ylabel('Lap Time [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(velDiffBlock,velDiffBlock_CompTMean,'o--k')
%     hold('on')
%     plot(velDiffBlock,velDiffBlock_CompTMax,'*--k')
%     ylim([0.01 0.07])
% %     yticks([0.01 0.04 0.07])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Comp. Time [s]','interpreter','latex','FontSize',11)
%     xlabel('Difference in Maximum Velocity [m/s]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Lap Time','Mean Comp. Time','Max. Comp. Time','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
%     % Plot 2: Position Changes
%     yyaxis left
%     plot(velDiffBlock,velDiffBlock_PosChStep,'*:k')
%     ylim([0 13])
%     ylabel('Time Step [s]','interpreter','latex','FontSize',11)
%     yyaxis right
%     plot(velDiffBlock,velDiffBlock_PosChCP,'o--k')
%     ylim([0 650])
% %     yticks([-2*pi -1*pi 0 pi 2*pi 3*pi])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Location [Track Checkpoint]','interpreter','latex','FontSize',11)
%     xlabel('Difference in Maximum Velocity [m/s]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Time Step of Position Change','Location of Position Change','Location', 'NorthEast');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11) 
    
   
    %% New Attempt
    
    %% Plot 2: Lap Times vs Velocity Difference - with and without blocking
%     plot(velDiff,velDiff_LapT,'s--','Color', color100(1,:))
%     hold('on')
%     plot(velDiffBlock,velDiffBlock_LapT,'s--','Color', color100(2,:))    
% %     ylim([11 13])
%     ylabel('Lap Time [s]','interpreter','latex','FontSize',11)
%     xlabel('Difference in Maximum Velocity [m/s]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Time Vehicle 1; Normal Driving Vehicle 2','Time Vehicle 1; Blocking Vehicle 2','Location', 'SouthEast');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
%     
%     print -depsc 20190318Eval_2_LatTimesVSVelDiff
    
    
    %% Plot 3: Computation Times vs Velocity Difference - with and without blocking
%     plot(velDiff,velDiff_CompTMean,'o--','Color', color100(1,:))
%     hold('on')
%     plot(velDiff,velDiff_CompTMax,'*--','Color', color100(1,:))
%     hold('on')
%     plot(velDiffBlock,velDiffBlock_CompTMean,'o--','Color', color100(2,:))
%     hold('on')
%     plot(velDiffBlock,velDiffBlock_CompTMax,'*--','Color', color100(2,:))
%     ylim([0 0.07])
% %     yticks([0.01 0.04 0.07])
% %     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
%     ylabel('Computation Time [s]','interpreter','latex','FontSize',11)
%     xlabel('Difference in Maximum Velocity [m/s]','interpreter','latex','FontSize',11)
%     grid on
%     leg = legend('Mean Time Vehicle 1; Normal Driving Vehicle 2', ...
%         'Max Time Vehicle 1; Normal Driving Vehicle 2',...
%         'Mean Time Vehicle 1; Blocking Vehicle 2',...
%         'Max Time Vehicle 1; Blocking Vehicle 2','Location', 'NorthWest');
%     set(leg,'interpreter','latex','FontSize',11);
%     set(gca,'ticklabelinterpreter','latex','FontSize',11)
%     
%     print -depsc 20190318Eval_3_CompTimesVSVelDiff
    
    
    %% Plot 6: Lap and Computation Times vs Number of Vehicles
    yyaxis left
    plot(multVeh,multVeh_LapT,'s:k')
    ylim([11 14])
    ylabel('Lap Time [s]','interpreter','latex','FontSize',11)
    yyaxis right
    plot(multVeh,multVeh_CompTMean,'o--k')
    hold('on')
    plot(multVeh,multVeh_CompTMax,'*--k')
    ylim([0.01 0.07])
%     yticks([0.01 0.04 0.07])
%     yticklabels({'$-2\pi$','$-\pi$','$0$','$\pi$','$2\pi$','$3\pi$'})
    ylabel('Computation Time Time [s]','interpreter','latex','FontSize',11)
    xlabel('Number of Racing Vehicles','interpreter','latex','FontSize',11)
    grid on
    leg = legend('Lap Time Vehicle 1','Mean Computation Time Vehicle 1','Max Computation Time Vehicle 1','Location', 'NorthWest');
    set(leg,'interpreter','latex','FontSize',11);
    set(gca,'ticklabelinterpreter','latex','FontSize',11)
    
    print -depsc 20190318Eval_6_TimesVSNbVehicles
    
    
end

