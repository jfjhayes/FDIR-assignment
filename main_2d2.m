%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 2D - Detection & Isolation Attempt 2
clear
exportMode = false;                         % controls plots saving as eps

%% Initial Conditions - Common for all sims
u0 = 30;                                    % longitudinal velocity (ms^-1)
z0 = 500;                                   % altitude (m)

deltaMax = deg2rad(50);                     % actuator max amplitude deflection (rad)
deltaMaxRate = deg2rad(25);                 % actuator max rate limit (rads^-1)

% Simulation Conditions % 
stepSize = 0.001;                           % integration step size
commInterval = 0.001;                       % communications interval
endTime = 120;                              % simulation duration (s)
i = 0;                                      % initialise counter

% Preallocate generic simulation storage arrays (stops MATLAB being angry w/ me) %
numSteps = endTime / commInterval + 1;
tout = zeros(numSteps, 1);                  % time


%% Reference Simulation
pRef = 0;                                           % roll rate (rads^-1)
rRef = 0;                                           % yaw rate (rads^-1)
betaRef = 0;                                        % sideslip angle (rad)
phiRef = 0;                                         % roll angle (rad)
psiRef = 0;                                         % yaw angle (rad)
deltaARef = 0;                                      % aileron deflection angle (rad)
deltaRRef = 0;                                      % rudder deflection angle (rad)

xRef = [pRef, rRef, betaRef, phiRef, psiRef]';      % state vector
xdotRef = zeros(5,1);                               % state derivatives
uRef = [deltaARef, deltaRRef]';                     % input vector

% Preallocate reference simulation storage arrays % 
xoutRef = zeros(numSteps, length(xRef));            % states    
xdotoutRef = zeros(numSteps, length(xdotRef));      % state derivatives
uoutRef = zeros(numSteps, length(uRef));            % actuator inputs

% Zig-zag Setup %
psiTargetRef = deg2rad(20);                         % heading target (rad)
deltaRCommandRef = deg2rad(20);                     % initial rudder command (rad)

% Simulation Loop % 
for time = 0:stepSize:endTime

    % Data logging for each commInterval
    if rem(time, commInterval) == 0
        i = i+1;
        tout(i) = time;
        xoutRef(i,:) = xRef;
        xdotoutRef(i,:) = xdotRef;
        uoutRef(i,:) = uRef;
    end

    % Reference Zig-zag logic % 
    if abs(xRef(5)) >= psiTargetRef                         % if yaw exceeds absolute 20 deg
        deltaRCommandRef = -sign(xRef(5)) * deg2rad(20);    % reverse rudder input
    end

    % Reference Rudder Command & Actuator Rate limit % 
    deltaRRef = uRef(2) + sign(deltaRCommandRef - uRef(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommandRef - uRef(2)));
    uRef(2) = max(-deltaMax, min(deltaMax, deltaRRef));

    % Reference Apply Actuator Saturation and Rate Limits %
    if i > 1
        uRef = limitActuators([uRef(1); uRef(2)], uoutRef(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    uoutRef(i,:) = uRef';

    % Compute reference state derivatives and states %
    xdotRef = latModel(xRef, uRef);
    xRef = RK4(@latModel, stepSize, xRef, uRef);
end

%% Faulty (Real) Simulation
p = 0;                                      % roll rate (rads^-1)
r = 0;                                      % yaw rate (rads^-1)
beta = 0;                                   % sideslip angle (rad)
phi = 0;                                    % roll angle (rad)
psi = 0;                                    % yaw angle (rad)
deltaA = 0;                                 % aileron deflection angle (rad)
deltaR = 0;                                 % rudder deflection angle (rad)

x = [p, r, beta, phi, psi]';                % state vector
xdot = zeros(5,1);                          % state derivatives
u = [deltaA, deltaR]';                      % input vector

% Preallocate Faulty storage arrays %
xout = zeros(numSteps, length(x));          % states    
xdotout = zeros(numSteps, length(xdot));    % state derivatives
uout = zeros(numSteps, length(u));          % actuator inputs

% Reset Simulation Conditions % 
i = 0;                                      % initialise counter

% Zig-zag Setup %  
psiTarget = deg2rad(20);                    % heading target (rad)
deltaRCommand = deg2rad(20);                % initial rudder command (rad)

% Fault setup %
stepFaultSensor = deg2rad(10);              % sensor stepwise fault of 10 degrees (rad)
driftRateSensor = deg2rad(1);               % sensor driftwise fault drift rate of 0.5 deg/s (rad/s)
stepFaultActuator = deg2rad(1);             % actuator stepwise fault of 1 degrees (rad)
driftRateActuator = deg2rad(0.1);           % actuator driftwise fault of 0.1 deg/s (rad/s)

% Fault toggles % 
faultStartTime = 30;                        % fault start time (s)
faultApplied = false;                       % initialise flag

stepSensor = false;
stepActuator = true;
driftSensor = false;
driftActuator = false;

for time = 0:stepSize:endTime

    % Data Logging for each commInterval %
    if rem(time, commInterval)==0
        i = i+1;
        tout(i) = time;                                 % store time
        xdotout(i,:) = xdot;	                        % store state derivatives
        xout(i,:) = x;                                  % store states
        uout(i,:) = u;                                  % store actuator inputs
    end

    % Apply faults after faultStartTime %
    if time >= faultStartTime 
        if stepSensor && ~faultApplied
            x(5) = x(5) + stepFaultSensor;
        end

        if stepActuator
            u(2) = u(2) + stepFaultActuator;
        end

        if driftSensor
            x(5) = x(5) + (driftRateSensor * stepSize);
        end

        if driftActuator
            u(2) = u(2) + (driftRateActuator * stepSize);
        end

        faultApplied = true;
    end

    % Reference Zig-zag logic %        
    if abs(x(5)) >= psiTarget                           % If yaw exceeds ±20 deg 
        deltaRCommand = -sign(x(5)) * deg2rad(20);      % Reverse rudder input
    end

    % Reference Rudder command & Actuator Rate limit %
    deltaR = u(2) + sign(deltaRCommand - u(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommand - u(2)));
    u(2) = max(-deltaMax, min(deltaMax, deltaR));

    % Reference Apply Actuator Saturation and Rate Limits %
    if i> 1
        u = limitActuators([u(1); u(2)], uout(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    uout(i,:) = u';

    % Compute reference state derivatives and states %
    xdot = latModel(x, u);
    x = RK4(@latModel, stepSize, x, u);
end

%% Output Plotting
if exportMode
    % Individual plots
    figure;
    plot(tout, rad2deg(xout(:,5)), 'r', 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(xoutRef(:,5)), 'b--', 'LineWidth', 1.5);
    ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Faulty Heading', 'Reference Heading', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '2d_driftwise_yaw.eps', 'epsc');

    figure;
    stairs(tout, rad2deg(uout(:,2)), 'r', 'LineWidth', 1.5); hold on;
    stairs(tout, rad2deg(uoutRef(:,2)), 'b--', 'LineWidth', 1.5);
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-25 35]);
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Faulty Deflection', 'Reference Deflection', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '2d_driftwise_rudder_deflection.eps', 'epsc');

    figure;
    plot(tout, rad2deg(xout(:,2)), 'r', 'LineWidth', 1.5); hold on
    plot(tout, rad2deg(xoutRef(:,2)),  'b--', 'LineWidth', 1.5);
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Faulty Yaw Rate', 'Reference Yaw Rate', 'Interpreter', 'latex');
    grid on;
    saveas(gcf, '2d_driftwise_yaw_rate.eps', 'epsc');

else
    % Subplots
    figure;

    subplot(3,1,1);
    plot(tout, rad2deg(xout(:,5)), 'r', 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(xoutRef(:,5)), 'b--', 'LineWidth', 1.5);
    ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    title('Effect of Sensor Fault on Heading', 'Interpreter', 'latex');
    legend('Faulty Heading', 'Reference Heading', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,2);
    stairs(tout, rad2deg(uout(:,2)), 'r', 'LineWidth', 1.5); hold on;
    stairs(tout, rad2deg(uoutRef(:,2)), 'b--', 'LineWidth', 1.5);
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-25 35]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Rudder Deflection Over Time', 'Interpreter', 'latex');
    legend('Faulty Deflection', 'Reference Deflection', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,3);
    plot(tout, rad2deg(xout(:,2)), 'r', 'LineWidth', 1.5); hold on
    plot(tout, rad2deg(xoutRef(:,2)),  'b--', 'LineWidth', 1.5);
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Yaw Rate Over Time', 'Interpreter', 'latex');
    legend('Faulty Yaw Rate', 'Reference Yaw Rate', 'Interpreter', 'latex');
    grid on;
    hold off
end