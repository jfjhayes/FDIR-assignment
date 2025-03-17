%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 2D - Detection & Isolation Attempt 2
clear
close all

%% Initial Conditions - Common for all sims
u0 = 30;                                                % longitudinal velocity (ms^-1)
z0 = 500;                                               % altitude (m)

deltaMax = deg2rad(50);                                 % actuator max amplitude deflection (rad)
deltaMaxRate = deg2rad(25);                             % actuator max rate limit (rads^-1)

%% Reference Simulation
pRef = 0;                                               % roll rate (rads^-1)
rRef = 0;                                               % yaw rate (rads^-1)
betaRef = 0;                                            % sideslip angle (rad)
phiRef = 0;                                             % roll angle (rad)
psiRef = 0;                                             % yaw angle (rad)
deltaARef = 0;                                          % aileron deflection angle (rad)
deltaRRef = 0;                                          % rudder deflection angle (rad)

xRef = [pRef, rRef, betaRef, phiRef, psiRef]';          % state vector
xdotRef = zeros(5,1);                                   % state derivatives
uRef = [deltaARef, deltaRRef]';                         % input vector

% Zig-zag Setup %
psiTargetRef = deg2rad(20);                             % heading target (rad)
deltaRCommandRef = deg2rad(20);                         % initial rudder command (rad)

% Simulation Conditions % 
stepSizeRef = 0.01;                                     % integration step size
commIntervalRef = 0.01;                                 % communications interval
endTimeRef = 120;                                       % simulation duration (s)
iRef = 1;                                               % initialise counter

% Preallocate reference simulation storage arrays (stops MATLAB being angry w/ me) %
numStepsRef = endTimeRef / commIntervalRef + 1;
toutRef = zeros(numStepsRef, 1);                       % time 
xoutRef = zeros(numStepsRef, length(xRef));            % states    
xdotoutRef = zeros(numStepsRef, length(xdotRef));      % state derivatives
uoutRef = zeros(numStepsRef, length(uRef));            % actuator inputs

% Simulation Loop % 
for timeRef = 0:stepSizeRef:endTimeRef

    % Data logging for each commInterval
    if rem(timeRef, commIntervalRef) == 0
        iRef = iRef+1;
        toutRef(iRef) = timeRef;
        xoutRef(iRef,:) = xRef;
        xdotoutRef(iRef,:) = xdotRef;
        uoutRef(iRef,:) = uRef;
    end

    % Reference Zig-zag logic % 
    if abs(xRef(5)) >= psiTargetRef                         % if yaw exceeds absolute 20 deg
        deltaRCommandRef = -sign(xRef(5)) * deg2rad(20);    % reverse rudder input
    end

    % Reference Apply Actuator Saturation and Rate Limits %
    if iRef > 1
        uRef = limitActuatorsRef([uRef(1); uRef(2)], uoutRef(iRef-1,:)', deltaMax, deltaMaxRate, stepSizeRef);
    end
 
    % Reference Rudder Command & Actuator Rate limit %
    deltaRRef = uRef(2) + sign(deltaRCommandRef - uRef(2)) * min(deltaMaxRate * stepSizeRef, abs(deltaRCommandRef - uRef(2)));
    uRef(2) = max(-deltaMax, min(deltaMax, deltaRRef));

    uoutRef(iRef,:) = uRef';

    % Compute reference state derivatives and states %
    xdotRef = latModel(xRef, uRef);
    xRef = RK4(@latModel, stepSizeRef, xRef, uRef);
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

% Zig-zag Setup %  
psiTarget = deg2rad(20);                    % heading target (rad)
deltaRCommand = deg2rad(20);                % initial rudder command (rad)

% Simulation Conditions % 
stepSize = 0.01;                            % integration step size
commInterval = 0.01;                        % communications interval
endTime = 120;                              % simulation duration (s)
i = 1;                                      % initialise counter

% Preallocate simulation storage arrays (stops MATLAB being angry w/ me) %
numSteps = endTime / commInterval + 1;
tout = zeros(numSteps, 1);                  % time
xout = zeros(numSteps, length(x));          % states    
xdotout = zeros(numSteps, length(xdot));    % state derivatives
uout = zeros(numSteps, length(u));          % actuator inputs
residualout = zeros(numSteps, 2);   % residuals

% Fault setup %
stepFaultSensor = deg2rad(10);              % sensor stepwise fault of 10 degrees (rad)
stepFaultActuator = deg2rad(10);            % actuator stepwise fault of 10 degree (rad)
driftRateSensor = deg2rad(2.5);             % sensor driftwise fault of 2.5 deg/s (rad/s)
driftRateActuator = deg2rad(0.1);           % actuator driftwise fault of 0.1 deg/s (rad/s)
sensorFaultApplied = false;                 % initialise sensor flag (FAULT EYES ONLY)
actuatorFaultApplied = false;               % initialise actuator flag (FAULT EYES ONLY)

% Fault toggles % 
faultStartTime = 30;        % fault start time (s)
stepSensor = false;         % works
stepActuator = false;       % works
driftSensor = true;        % works
driftActuator = false;      % works

% FDI setup % 
stepThreshold = deg2rad(1);                             % step fault threshold
driftThreshold = deg2rad(0.01);                         % drift fault threshold                       
faultDetected = false(numSteps, size(xout, 2) - 1);     % fault detected boolean
stepDetected = false(numSteps, size(xout, 2) - 1);      % step fault boolean
driftDetected = false(numSteps, size(xout, 2) - 1);     % drift fault boolean
firstOccurence = true;                                  % stops repetition
sensorFaultDetected = false;                            % for controller
actuatorFaultDetected = false;                          % for controller

for time = 0:stepSize:endTime

    % Data Logging for each commInterval %
    if rem(time, commInterval)==0
        i = i+1;
        tout(i) = time;                                 % store time
        xdotout(i,:) = xdot;	                        % store state derivatives
        xout(i,:) = x;                                  % store states
        uout(i,:) = u;                                  % store actuator inputs
    end

    if abs(x(5)) >= psiTarget
        deltaRCommand = -sign(x(5)) * deg2rad(20);
    end

    % Rudder Command & Actuator Rate limit %
    deltaR = u(2) + sign(deltaRCommand - u(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommand - u(2)));
    u(2) = max(-deltaMax, min(deltaMax, deltaR));

    % Actuator Faults % 
    if time >= faultStartTime
        if stepActuator %&& ~actuatorFaultApplied
            actuatorFaultOffset = stepFaultActuator; 
            actuatorFaultApplied = true;
        end
    
        if driftActuator
            actuatorFaultOffset = actuatorFaultOffset + (driftRateActuator * stepSize);
        end
    else
        actuatorFaultOffset = 0;
    end
    
    % Apply actuator faults BEFORE limiting %
    if time >= faultStartTime
        u(2) = deltaRCommand + actuatorFaultOffset;
    end

    % Apply Actuator Saturation and Rate Limits %
    if i > 1
        u = limitActuators(u, uout(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    uout(i,:) = u';

    % Compute faulty state derivatives and states %
    xdot = latModel(x, u);
    x = RK4(@latModel, stepSize, x, u);

    % Sensor faults % 
    if time >= faultStartTime
        if stepSensor && ~sensorFaultApplied
            x(5) = x(5) + stepFaultSensor;
            sensorFaultApplied = true;
        else
            x(5) = x(5);
        end
        if driftSensor
            x(5) = x(5) + (driftRateSensor * stepSize);
        else
            x(5) = x(5);
        end
    end

    % Fault Detection %
    sensorResidual = xout(i,5) - xoutRef(i,5);                          % calculate state vector residual for yaw angle ONLY
    actuatorResidual = uout(i,2) - uoutRef(i,2);                        % calculate input vector residual for rudder deflection ONLY
    residual = [sensorResidual actuatorResidual];                       % residual matrix

    faultDetected(i,:) = abs(residual(1)) > stepThreshold || abs(residual(2)) > driftThreshold;     % set generic faultDetected HIGH if above either threshold

    % Residual Loop %
    for j = 1:length(residual)
        if i > 1
            stepDetected(i,j) = abs(residual(j)) > stepThreshold;   % step?
        end

        if i > 10 && all(residualout(i-6:i-1, j) ~= 0)                  % wait long enough to establish trend
            recentTrend = mean(diff(residualout(i-6:i-1, j)));          % take mean of last 5 results
            driftDetected(i,j) = abs(recentTrend) > driftThreshold;     % drift?
        else
            driftDetected(i,j) = false;
        end
        
        % Fault Isolation %
        if stepDetected(i,j) || driftDetected(i,j)
            if j == 1
                faultLocation = "Sensor";
                sensorFaultDetected = true;
            elseif j== 2
                faultLocation = "Actuator";
                actuatorFaultDetected = true;
            else
                faultLocation = "Unknown";
            end

            if stepDetected(i,j)
                faultType = "Step Fault";
            elseif driftDetected(i,j)
                faultType = "Drift Fault";
            else
                faultType = "Unknown";
            end

            if firstOccurence
                fprintf("Fault detected in %s: %s above threshold at time %ds\n ", faultLocation, faultType, round(tout(i)));     % what's up, Doc?
                firstOccurence = false;
            end
        end
    end

    % Store residual %
    residualout(i,:) = residual;



end

figure;
plot(tout, rad2deg(residualout(:,1)), 'r', 'LineWidth', 1.5); hold on;
yline(rad2deg(driftThreshold), 'k--');
yline(-rad2deg(driftThreshold), 'k--');
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Residual (deg)', 'Interpreter', 'latex');
set(gca, "TickLabelInterpreter", 'latex');
grid on;
saveas(gcf, '2d_sensor_drift.eps', 'epsc');




%% Output Plotting
exportMode = false;                         % controls plots saving as eps

if exportMode
    % Individual plots
    figure;
    plot(tout, rad2deg(residualout(:,1)), 'r', 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(residualout(:,2)), 'b', 'LineWidth', 1.5);
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
    saveas(gcf, '2c_driftwise_rudder_deflection.eps', 'epsc');

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