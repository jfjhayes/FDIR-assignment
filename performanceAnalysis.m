function [riseTime, overshoot, settlingTime, steadyStateError] = performanceAnalysis(toutRef, xoutRef, psiTargetRef, manoeuvreStartTime)
    
    psiDegRef = rad2deg(xoutRef(:,5));
    targetDegRef = rad2deg(psiTargetRef);
    tol = 0.5;

    % Rise time %
    idx10Ref = find(psiDegRef >= 0.1 * targetDegRef, 1, 'first');
    idx90Ref = find(psiDegRef >= 0.9 * targetDegRef, 1, 'first');
    if ~isempty(idx10Ref) && ~isempty(idx90Ref)
        riseTime = toutRef(idx90Ref) - toutRef(idx10Ref);
    else
        riseTime = NaN;
    end

    % Overshoot %
    maxPsiRef = max(psiDegRef);
    overshoot = max(maxPsiRef - targetDegRef, 0);

    % Settling time %
    settlingTime = NaN;
    for k = 1:length(toutRef)
        if all(abs(psiDegRef(k:end) - targetDegRef) <= tol)
            settlingTime = toutRef(k) - manoeuvreStartTime;
            break;
        end
    end

    % SS error %
    steadyStateError = abs(psiDegRef(end) - targetDegRef);
end
