function u_limited = limitActuators(u, u_prev, deltaMax, deltaMaxRate, stepSize)

    % Applies Rate and Saturation Limits to actuator inputs % 

    u_saturated = max(min(u, deltaMax), -deltaMax);

    maxChange = deltaMaxRate * stepSize;                            % max allowed change per step
    deltaChange = u_saturated - u_prev;                             % change in input

    deltaChange = max(min(deltaChange, maxChange), -maxChange);     % apply rate limiting

    u_limited = u_prev + deltaChange;                               % limited input
end