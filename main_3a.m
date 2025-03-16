%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 3A - Closed-loop Operation
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
xDesRef = [0 0 0 0 0];

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


%% Performance Analysis
psiDegRef = rad2deg(xoutRef(:,5));
targetDegRef = rad2deg(psiTargetRef); 
tol = 0.5;

% Rise time %
idx10Ref = find(psiDegRef >= 0.1 * targetDegRef, 1, 'first');
idx90Ref = find(psiDegRef >= 0.9 * targetDegRef, 1, 'first');
if ~isempty(idx10Ref) && ~isempty(idx90Ref)
    riseTimeRef = toutRef(idx90Ref) - toutRef(idx10Ref);
else
    riseTimeRef = NaN;
end

% Overshoot %
maxPsiRef = max(psiDegRef);
overshootRef = max(maxPsiRef - targetDegRef, 0);

% Settling time %
settlingTimeRef = NaN;
for k = 1:length(toutRef)
    if all(abs(psiDegRef(k:end) - targetDegRef) <= tol)
        settlingTimeRef = toutRef(k) - manoeuvreStartTime;
        break;
    end
end

% SS error %
steadyStateErrorRef = abs(psiDegRef(end) - targetDegRef);

fprintf('Rise Time: %.2f sec\n', riseTimeRef);
fprintf('Overshoot: %.2f deg\n', overshootRef);
fprintf('Settling Time: %.2f sec\n', settlingTimeRef);
fprintf('Steady-State Error: %.2f deg\n', steadyStateErrorRef);

%% Output Plotting
exportMode = true;                         % controls plots saving as eps

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
    %plot(tout, rad2deg(xout(:,5)), 'r', 'LineWidth', 1.5); hold on;
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
    %plot(tout, rad2deg(uout(:,2)), 'r', 'LineWidth', 1.5); hold on;
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