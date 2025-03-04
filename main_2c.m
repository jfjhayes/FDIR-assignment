%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 2C - Faults in actuator
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
xFaulty = zeros(5,1);                       % faulty state vector
uFaulty = zeros(2,1);                       % faulty input vector

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
xoutFaulty = zeros(numSteps, length(x));    % faulty states
uout = zeros(numSteps, length(u));          % actuator inputs
uoutFaulty = zeros(numSteps, length(u));    % faulty actuator inputs

% Zig-zag Setup %  
psiTarget = deg2rad(20);                    % heading target (rad)
deltaRCommand = deg2rad(20);                % initial rudder command (rad)

% Fault setup
stepFaultActuator = deg2rad(1);             % stepwise fault of 1 degrees (rad)
driftRateActuator = deg2rad(0.1);           % driftwise fault of 0.1 deg/s (rad/s)

% Simulation Loop %
for time = 0:stepSize:endTime

    % Data Logging for each commInterval %
    if rem(time, commInterval)==0
        i = i+1;
        tout(i) = time;                                 % store time
        xdotout(i,:) = xdot;	                        % store state derivatives
        xout(i,:) = x;                                  % store states
        uout(i,:) = u;                                  % store actuator inputs
        xoutFaulty(i,:) = xFaulty;                      % store faulty states
        uoutFaulty(i,:) = uFaulty;                      % store faulty actuator inputs
    end    

    % Apply faults, comment both for unfaulty, uncomment both for something messed up and evil %
    uFaulty = u;
    %uFaulty(2) = u(2) + stepFaultActuator;              % STEPWISE
    uFaulty(2) = u(2) + (driftRateActuator * time);     % DRIFTWISE


    % Zig-zag logic %        
    if abs(x(5)) >= psiTarget                           % If yaw exceeds ±20 deg 
        deltaRCommand = -sign(x(5)) * deg2rad(20);      % Reverse rudder input
    end

    % Rudder command & Actuator Rate limit %
    deltaR = u(2) + sign(deltaRCommand - u(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommand - u(2)));
    u(2) = max(-deltaMax, min(deltaMax, deltaR));

    % Apply Actuator Saturation and Rate Limits %
    if i > 1
        u = limitActuators([u(1); u(2)], uout(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    % Apply Faulty Actuator Constraints %
    uFaulty(2) = max(-deltaMax, min(deltaMax, uFaulty(2)));

    % Store rudder Inputs %
    uout(i,:) = u'; 
    uoutFaulty(i,:) = uFaulty';

    % Compute State Derivatives %
    xdot = latModel(x, u);
    xdotFaulty = latModel(x, uFaulty);

    % RK4 Integration %
    x = RK4(@latModel, stepSize, x, u);
    xFaulty = RK4(@latModel, stepSize, xFaulty, uFaulty);
end

% Output Plotting %
exportMode = true;

if exportMode
    % Individual plots
    figure;
    plot(tout, rad2deg(xout(:,5)), 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(xoutFaulty(:,5)), 'r--', 'LineWidth', 1.5);
    ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    legend('True Heading', 'Faulty Heading', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '2c_driftwise_yaw.eps', 'epsc');

    figure;
    stairs(tout, rad2deg(uout(:,2)), 'LineWidth', 1.5);
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-25 25]);
    set(gca, "TickLabelInterpreter", 'latex');
    grid on;
    saveas(gcf, '2c_driftwise_rudder_deflection.eps', 'epsc');

    figure;
    plot(tout, rad2deg(xout(:,2)), 'LineWidth', 1.5);
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    grid on;
    saveas(gcf, '2c_driftwise_yaw_rate.eps', 'epsc');

else
    % Subplots
    figure;

    subplot(3,1,1);
    plot(tout, rad2deg(xout(:,5)), 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(xoutFaulty(:,5)), 'r--', 'LineWidth', 1.5);
    ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    title('Effect of Sensor Fault on Heading', 'Interpreter', 'latex');
    legend('True Heading', 'Faulty Heading', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,2);
    stairs(tout, rad2deg(uout(:,2)), 'LineWidth', 1.5);
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-25 25]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Rudder Deflection Over Time', 'Interpreter', 'latex');
    grid on;

    subplot(3,1,3);
    plot(tout, rad2deg(xout(:,2)), 'LineWidth', 1.5);
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Yaw Rate Over Time', 'Interpreter', 'latex');
    grid on;
end