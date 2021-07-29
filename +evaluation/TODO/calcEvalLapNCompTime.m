function calcEvalLapNCompTime(log,dt)

    logLength = length(log.vehicles{1,2}.lapLog{1,1});
    
    % Lap Time
    lapTimeBase = (logLength - 1) * dt;
    
    XOldLap = log.vehicles{1,2}.currLapLog(logLength).x0(1);
    XNewLap = log.vehicles{1,2}.currLapLog(logLength+1).x0(1);
    deltaX = norm(XNewLap-XOldLap);
    velXGlobal = deltaX / dt;
    deltaX2Finish = norm(0.088-XOldLap);
    lapTime2Finish = deltaX2Finish / velXGlobal;
    
    lapTime = lapTimeBase + lapTime2Finish;
    
    
    % Computation Times
    optTimesMatrix = nan(logLength+1,2);
    for i = 1:(logLength+1)
       optTimesMatrix(i,1) = log.vehicles{1,2}.currLapLog(i).itResult{1,1}.optimization_log.output.time;
       optTimesMatrix(i,2) = log.vehicles{1,2}.currLapLog(i).itResult{1,2}.optimization_log.output.time;
    end
    optTimesList = [optTimesMatrix(:,1);optTimesMatrix(:,2)];
    compTimeMax = max(optTimesList);
    
    optTimesListNonZero = [];
    for j = 1:length(optTimesList)
       if optTimesList(j) ~= 0
           optTimesListNonZero = [optTimesListNonZero optTimesList(j)];
       end
    end
    compTimeMean = mean(optTimesListNonZero);

    [lapTime,compTimeMean,compTimeMax]

end

