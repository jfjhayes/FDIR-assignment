%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 2B - Stepwise fault in sensor
clear
%clf

% Initialise Variables %
p = 0;                                      % roll rate (rads^-1)
r = 0;                                      % yaw rate (rads^-1)
beta = 0;                                   % sideslip angle (rad)
phi = 0;                                    % roll angle (rad)
psi = 0;                                    % yaw angle (rad)
deltaA = 0;                                 % aileron deflection angle (rad)
deltaR = 0;                                 % rudder deflection angle (rad)
x = [p, r, beta, phi, psi]';                % state vector
u = [deltaA, deltaR]';                      % input vector
xdot = zeros(5,1);                          % state derivatives

% Initial Conditions %
u0 = 30;                                    % longitudinal velocity (ms^-1)
z0 = 500;                                   % altitude (m)

% Actuator limits %
deltaMax = deg2rad(50);                     % actuator max amplitude deflection (rad)
deltaMaxRate = deg2rad(25);                 % actuator max rate limit (rads^-1)

% Simulation Conditions %
stepSize = 0.01;                            % integration step size
commInterval = 0.01;                        % communications interval
endTime = 120;                              % simulation duration (s)
i = 0;                                      % initialise counter

% Preallocate storage arrays (stops MATLAB being angry w/ me) %
numSteps = endTime / commInterval + 1;
tout = zeros(numSteps, 1);                  % time
xdotout = zeros(numSteps, length(xdot));    % state derivatives
xout = zeros(numSteps, length(x));          % states    
uout = zeros(numSteps, length(u));          % actuator inputs
psiFaultout = zeros(numSteps, 1);           % faulty heading angle

% Zig-zag Setup %  
psiTarget = deg2rad(20);                    % heading target (rad)
deltaRCommand = deg2rad(20);                % initial rudder command (rad)

% Fault setup
stepFaultActuator = deg2rad(1);           % stepwise fault of 10 degrees (rad)
driftRateActuator = deg2rad(0.5);  

% Simulation Loop %
for time = 0:stepSize:endTime

    % Data Logging for each commInterval %
    if rem(time, commInterval)==0
        i = i+1;
        tout(i) = time;                                 % store time
        xdotout(i,:) = xdot;	                        % store state derivatives
        xout(i,:) = x;                                  % store states
        uout(i,:) = u;                                  % store actuators inputs
    end    

    % Zig-zag logic %        
    if abs(x(5)) >= psiTarget                           % If yaw exceeds ±20 deg 
        deltaRCommand = -sign(x(5)) * deg2rad(20);      % Reverse rudder input
    end

    deltaR = u(2) + sign(deltaRCommand - u(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommand - u(2))) + stepFaultActuator;
    
    % Apply Saturation Limit %
    u(2) = max(-deltaMax, min(deltaMax, deltaR));
    
    % Apply Actuator Rate & Saturation Limits %
    if i > 1
        u = limitActuators(u, uout(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    % Store rudder input % 
    uout(i,:) = u'; 

    % State Space Updates %
    xdot = latModel(x, u);                  % update state derivatives
    x = RK4(@latModel, stepSize, x, u);     % find states using RK4

end

% Output Plotting % 
figure;

subplot(3,1,1);
plot(tout, rad2deg(xout(:,5)), 'b', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Heading Angle (°)');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
set(gca,"TickLabelInterpreter",'latex');
title('Effect of Stepwise Actuator Fault on Heading');
grid on;

subplot(3,1,2);
plot(tout, rad2deg(uout(:,2)), 'k', 'LineWidth', 1.5);
ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex'); 
xlabel('Time (s)', 'Interpreter', 'latex');
set(gca,"TickLabelInterpreter",'latex');
title('Rudder Deflection Over Time');
grid on;

subplot(3,1,3);
plot(tout, rad2deg(xout(:,2)), 'm', 'LineWidth', 1.5);
ylabel('$r$ (deg/s)', 'Interpreter', 'latex'); 
xlabel('Time (s)', 'Interpreter', 'latex');
set(gca,"TickLabelInterpreter",'latex');
title('Yaw Rate Over Time');
grid on;