%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 3B - Closed-loop Operation with Sensor & Actuator Faults
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
xDesRef = zeros(1,5);

% Manoeuvre Setup %
psiTargetRef = deg2rad(35);                             % heading target (rad)
deltaRCommandRef = deg2rad(0);                          % initial rudder command (rad)
manoeuvreStartTime = 20;                                % manoeuvre start time (s)

% Simulation Conditions % 
stepSizeRef = 0.01;                                     % integration step size
commIntervalRef = 0.01;                                 % communications interval
endTimeRef = 60;                                        % simulation duration (s)
iRef = 1;                                               % initialise counter

% LQR setup %
A = [-8.5905, 2.0389, -14.5793, 0, 0; 0.3801, -2.2406,  17.5075, 0, 0; 0.0692, -0.7468, -0.3510, 0.2442, 0; 1, 0.0694, 0, 0, 0; 0, 1, 0, 0, 0]; 
B = [19.6050, -0.6821; -0.8672, 3.4921; 0, 0; 0, 0; 0, 0];
Q = diag([40, 1.5, 22, 0.68, 270]); 
R = diag([0.196, 0.9]); 
[K,~,~] = lqr(A, B, Q, R);

% Preallocate reference simulation storage arrays (stops MATLAB being angry w/ me) %
numStepsRef = endTimeRef / commIntervalRef + 1;
toutRef = zeros(numStepsRef, 1);                       % time 
xoutRef = zeros(numStepsRef, length(xRef));            % states    
xdotoutRef = zeros(numStepsRef, length(xdotRef));      % state derivatives
uoutRef = zeros(numStepsRef, length(uRef));            % actuator inputs
xDesoutRef = zeros(numStepsRef, length(xDesRef));

% Simulation Loop % 
for timeRef = 0:stepSizeRef:endTimeRef

    % Data logging for each commInterval
    if rem(timeRef, commIntervalRef) == 0
        iRef = iRef+1;
        toutRef(iRef) = timeRef;
        xoutRef(iRef,:) = xRef;
        xdotoutRef(iRef,:) = xdotRef;
        uoutRef(iRef,:) = uRef;
        xDesoutRef(iRef,:) = xDesRef;
    end

    % LQR controller % 
    if timeRef >= manoeuvreStartTime 
        xDesRef = [0 0 0 0 psiTargetRef]';
        errorRef = xRef - xDesRef;
        uRef = -0.06 * K * errorRef;
    else
        xDesRef = [0 0 0 0 0]';
    end

    % Reference Apply Actuator Saturation and Rate Limits %
    if iRef > 1
        uRef = limitActuators([uRef(1); uRef(2)], uoutRef(iRef-1,:)', deltaMax, deltaMaxRate, stepSizeRef);
    end

    uoutRef(iRef,:) = uRef';

    % Compute reference state derivatives and states %
    xdotRef = latModel(xRef, uRef);
    xRef = RK4(@latModel, stepSizeRef, xRef, uRef);
end

% Performance Analysis % 
[riseTimeRef, overshootRef, settlingTimeRef, steadyStateErrorRef] = performanceAnalysis(toutRef, xoutRef, psiTargetRef, manoeuvreStartTime);
fprintf('Reference Rise Time: %.2f sec\n Reference Overshoot: %.2f deg\n Reference Settling Time: %.2f sec\n Reference Steady-State Error: %.2f deg\n', riseTimeRef, overshootRef, settlingTimeRef, steadyStateErrorRef);

%% Faulty Simulation
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
xDes = zeros(1,5);

% Manoeuvre Setup %
psiTarget = deg2rad(35);                             % heading target (rad)
deltaRCommand = deg2rad(0);                          % initial rudder command (rad)
manoeuvreStartTime = 20;                                % manoeuvre start time (s)

% Simulation Conditions % 
stepSize = 0.01;                                     % integration step size
commInterval = 0.01;                                 % communications interval
endTime = 60;                                        % simulation duration (s)
i = 1;                                               % initialise counter

% LQR setup %
A = [-8.5905, 2.0389, -14.5793, 0, 0; 0.3801, -2.2406,  17.5075, 0, 0; 0.0692, -0.7468, -0.3510, 0.2442, 0; 1, 0.0694, 0, 0, 0; 0, 1, 0, 0, 0]; 
B = [19.6050, -0.6821; -0.8672, 3.4921; 0, 0; 0, 0; 0, 0];
Q = diag([40, 1.5, 22, 0.68, 270]); 
R = diag([0.196, 0.9]); 
[K,~,~] = lqr(A, B, Q, R);

% Preallocate reference simulation storage arrays (stops MATLAB being angry w/ me) %
numSteps = endTime / commInterval + 1;
tout = zeros(numSteps, 1);                       % time 
xout = zeros(numSteps, length(x));            % states    
xdotout = zeros(numSteps, length(xdot));      % state derivatives
uout = zeros(numSteps, length(u));            % actuator inputs
xDesout = zeros(numSteps, length(xDes));
residualout = zeros(numSteps, 2);   % residuals

% Fault setup %
stepFaultSensor = deg2rad(10);              % sensor stepwise fault of 10 degrees (rad)
stepFaultActuator = deg2rad(10);            % actuator stepwise fault of 10 degree (rad)
driftRateSensor = deg2rad(0.5);             % sensor driftwise fault of 2.5 deg/s (rad/s)
driftRateActuator = deg2rad(0.1);           % actuator driftwise fault of 0.1 deg/s (rad/s)
sensorFaultApplied = false;                 % initialise sensor flag (FAULT EYES ONLY)
actuatorFaultApplied = false;               % initialise actuator flag (FAULT EYES ONLY)

% Fault toggles % 
faultStartTime = 10;        % fault start time (s)
stepSensor = true;         % works
stepActuator = false;       % works
driftSensor = false;        % works
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

    % Data logging for each commInterval
    if rem(time, commIntervalRef) == 0
        i = i+1;
        tout(i) = time;
        xout(i,:) = x;
        xdotout(i,:) = xdot;
        uout(i,:) = u;
        xDesout(i,:) = xDes;
    end

    % Control logic % 
    if time >= manoeuvreStartTime 
        xDes = [0 0 0 0 psiTarget]';
    else
        xDes = [0 0 0 0 0]';
    end

    % Controller % 
    error = x - xDes;
    u = -0.06 * K * error;

    % Actuator Faults % 
    if time >= faultStartTime
        if stepActuator && ~actuatorFaultApplied
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

% Performance Analysis % 
[riseTime, overshoot, settlingTime, steadyStateError] = performanceAnalysis(tout, xout, psiTarget, manoeuvreStartTime);
fprintf('Faulty Rise Time: %.2f sec\n Faulty Overshoot: %.2f deg\n Faulty Settling Time: %.2f sec\n Faulty Steady-State Error: %.2f deg\n', riseTime, overshoot, settlingTime, steadyStateError);


%% Output Plotting
exportMode = false;                         % controls plots saving as eps

if exportMode
    % Individual plots
    figure;
    plot(toutRef, rad2deg(xoutRef(:,5)), 'b'); hold on;
    plot(toutRef, rad2deg(xDesoutRef(:,5)), 'k');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Heading', ' Command', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '3a_yaw.eps', 'epsc');

    figure;
    plot(toutRef, rad2deg(uoutRef(:,2)), 'b'); hold on;
    plot(toutRef, rad2deg(uoutRef(:,1)), 'r');
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Rudder', 'Aileron', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '3a_deflections.eps', 'epsc');

    figure;
    plot(toutRef, rad2deg(xoutRef(:,2)),  'b');
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    grid on;
    hold off;
    saveas(gcf, '3a_yaw_rate.eps', 'epsc');

else
    % Subplots
    figure;

    subplot(3,1,1);
    plot(tout, rad2deg(xout(:,5)), 'r', 'LineWidth', 1.5); hold on;
    plot(toutRef, rad2deg(xoutRef(:,5)), 'b', 'LineWidth', 1.5); hold on;
    plot(toutRef, rad2deg(xDesoutRef(:,5)), 'k');
    %ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    title('Effect of Sensor Fault on Heading', 'Interpreter', 'latex');
    legend('Heading', ' Command', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,2);
    plot(tout, rad2deg(uout(:,2)), 'r', 'LineWidth', 1.5); hold on;
    plot(toutRef, rad2deg(uoutRef(:,2)), 'b'); hold on;
    plot(toutRef, rad2deg(uoutRef(:,1)), 'r');
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    %ylim([-25 35]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Rudder Deflection Over Time', 'Interpreter', 'latex');
    legend('Rudder', 'Aileron', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,3);
    %plot(tout, rad2deg(xout(:,2)), 'r', 'LineWidth', 1.5); hold on
    plot(toutRef, rad2deg(xoutRef(:,2)),  'b');
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    %ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Yaw Rate Over Time', 'Interpreter', 'latex');
    %legend('Faulty Yaw Rate', 'Reference Yaw Rate', 'Interpreter', 'latex');
    grid on;
    hold off
end