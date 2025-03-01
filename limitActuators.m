function u_limited = limitActuators(u, u_prev, deltaMax, deltaMaxRate, stepSize)

    % Applies Rate and Saturation Limits to actuator inputs % 

    maxChange = deltaMaxRate * stepSize;                % max allowed change per step
    deltaChange = u - u_prev;                           % change in input
    
    % Apply rate limiting %
    deltaChange = max(min(deltaChange, maxChange), -maxChange);
    
    % Compute new limited input % 
    u_limited = u_prev + deltaChange;
    
    % Apply saturation limits % 
    u_limited = max(min(u_limited, deltaMax), -deltaMax);
end