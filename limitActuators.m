function u_limited = limitActuators(u, u_prev, deltaMax, deltaMaxRate, stepFaultActuator, driftRateActuator, stepActuator, driftActuator, time, faultStartTime, stepSize)

    persistent actuatorFaultApplied faultedSignal;

    if isempty(actuatorFaultApplied)  % Initialize  first call
        actuatorFaultApplied = false;
        faultedSignal = u(2);
    end

    % Track reference signal until fault is applied %
    if time < faultStartTime
        faultedSignal = u(2);
    end

    % Apply faults ONLY ONCE after faultStartTime %
    if time >= faultStartTime && ~actuatorFaultApplied
        if stepActuator
            faultedSignal = faultedSignal + stepFaultActuator;          % Apply step fault ONCE
        end
        actuatorFaultApplied = true;                                    % Markt fault applied
    end

    if driftActuator && actuatorFaultApplied
        faultedSignal = faultedSignal + driftRateActuator * stepSize;   % drift accumulates on the faulted signal
    end

    u_limited = u;
    u_limited(2) = faultedSignal;

    maxChange = deltaMaxRate * stepSize;                % max allowable rate of change
    deltaChange = u_limited(2) - u_prev(2);             % change in actuator signal

    if abs(deltaChange) > maxChange
        deltaChange = sign(deltaChange) * maxChange; 
    end

    u_limited(2) = u_prev(2) + deltaChange;             % apply rate-limited change

 
    u_limited(2) = max(min(u_limited(2), deltaMax), -deltaMax);         % enforce actuator saturation limits
end
