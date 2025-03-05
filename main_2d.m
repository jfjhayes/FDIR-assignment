%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
% Part 2D - Detection & Isolation
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
deltaRFaulty = 0;
x = [p, r, beta, phi, psi]';                % state vector
u = [deltaA, deltaR]';                      % input vector
xdot = zeros(5,1);                          % state derivatives
xFaulty = zeros(5,1);                       % faulty state vector
uFaulty = zeros(2,1);                       % faulty input vector
xdotFaulty = zeros(5,1);

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
xdotoutFaulty = zeros(numSteps, length(xdotFaulty));

xout = zeros(numSteps, length(x));          % states    
xoutFaulty = zeros(numSteps, length(xFaulty));    % faulty states

uout = zeros(numSteps, length(u));          % actuator inputs
uoutFaulty = zeros(numSteps, length(uFaulty));    % faulty actuator inputs

% Zig-zag Setup %  
psiTarget = deg2rad(20);                    % heading target (rad)
deltaRCommand = deg2rad(20);                % initial rudder command (rad)
deltaRCommandFaulty = deg2rad(20);

% Fault setup %
stepFaultSensor = deg2rad(10);              % sensor stepwise fault of 10 degrees (rad)
driftRateSensor = deg2rad(5);              % sensor driftwise fault drift rate of 0.5 deg/s (rad/s)
stepFaultActuator = deg2rad(0.1);             % actuator stepwise fault of 1 degrees (rad)
driftRateActuator = deg2rad(0.1);           % actuator driftwise fault of 0.1 deg/s (rad/s)

% Fault toggles & output settings % 
faultStartTime = 30;                        % fault start time (s)
stepSensor = false;
driftSensor = false;
stepActuator = true;
driftActuator = false;
exportMode = false;                         % controls plots saving as eps


% Simulation Loop %
for time = 0:stepSize:endTime

    % Data Logging for each commInterval %
    if rem(time, commInterval)==0
        i = i+1;
        tout(i) = time;                                 % store time
        xdotout(i,:) = xdot;	                        % store state derivatives
        xout(i,:) = x;                                  % store states
        uout(i,:) = u;                                  % store actuator inputs
        xdotoutFaulty(i,:) = xdotFaulty;                % store faulty state derivatives
        xoutFaulty(i,:) = xFaulty;                      % store faulty states
        uoutFaulty(i,:) = uFaulty;                      % store faulty actuator inputs
    end    

    % Apply faults after faultStartTime %
    if time >= faultStartTime
        if stepSensor
            xFaulty(5) = xFaulty(5) + stepFaultSensor;
        end
        if driftSensor
            xFaulty(5) = xFaulty(5) + driftRateSensor * (time - faultStartTime);
        end
        if stepActuator
            uFaulty(2) = uFaulty(2) + stepFaultActuator;
        end
        if driftActuator
            uFaulty(2) = uFaulty(2) + driftRateActuator * (time - faultStartTime);
        end
    else
        xFaulty = x;
        uFaulty = u;
    end

    % Reference Zig-zag logic %        
    if abs(x(5)) >= psiTarget                           % If yaw exceeds ±20 deg 
        deltaRCommand = -sign(x(5)) * deg2rad(20);      % Reverse rudder input
    end

    % Reference Rudder command & Actuator Rate limit %
    deltaR = u(2) + sign(deltaRCommand - u(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommand - u(2)));
    u(2) = max(-deltaMax, min(deltaMax, deltaR));

    % Reference Apply Actuator Saturation and Rate Limits %
    if i > 1
        u = limitActuators([u(1); u(2)], uout(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    % store actuator inputs %
    uout(i,:) = u';

    % Compute reference state derivatives and states %
    xdot = latModel(x, u);
    x = RK4(@latModel, stepSize, x, u);

    % FAULTY STUFF BELOW CHECK YOUR SENSORS VS YOUR ACTUATORS IDIOT% 
    % Fault Zig-zag logic %        
    if abs(xFaulty(5)) >= psiTarget                                 % If yaw exceeds ±20 deg 
        deltaRCommandFaulty = -sign(xFaulty(5)) * deg2rad(20);       % Reverse rudder input
    end

    % Faulty Rudder command & Actuator Rate limit %
    deltaRFaulty = uFaulty(2) + sign(deltaRCommandFaulty - uFaulty(2)) * min(deltaMaxRate * stepSize, abs(deltaRCommandFaulty - uFaulty(2)));
    uFaulty(2) = max(-deltaMax, min(deltaMax, deltaRFaulty));

    % Fault Apply Actuator Saturation and Rate Limits %
    if i > 1
        uFaulty = limitActuators([uFaulty(1); uFaulty(2)], uoutFaulty(i-1,:)', deltaMax, deltaMaxRate, stepSize);
    end

    % store faulty actuator inputs %
    uoutFaulty(i,:) = uFaulty';

    % Compute Faulty State Derivatives and States %
    xdotFaulty = latModel(xFaulty, uFaulty);
    xFaulty = RK4(@latModel, stepSize, xFaulty, uFaulty);

end

% Output Plotting %
if exportMode
    % Individual plots
    figure;
    plot(tout, rad2deg(xoutFaulty(:,5)), 'r', 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(xout(:,5)), 'b--', 'LineWidth', 1.5);
    ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Faulty Heading', 'Reference Heading', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '2c_driftwise_yaw.eps', 'epsc');

    figure;
    stairs(tout, rad2deg(uoutFaulty(:,2)), 'r', 'LineWidth', 1.5); hold on;
    stairs(tout, rad2deg(uout(:,2)), 'b--', 'LineWidth', 1.5);
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-25 35]);
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Faulty Deflection', 'Reference Deflection', 'Interpreter', 'latex');
    grid on;
    hold off;
    saveas(gcf, '2c_driftwise_rudder_deflection.eps', 'epsc');

    figure;
    plot(tout, rad2deg(xoutFaulty(:,2)), 'r', 'LineWidth', 1.5); hold on
    plot(tout, rad2deg(xout(:,2)),  'b--', 'LineWidth', 1.5);
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    legend('Faulty Yaw Rate', 'Reference Yaw Rate', 'Interpreter', 'latex');
    grid on;
    saveas(gcf, '2c_driftwise_yaw_rate.eps', 'epsc');

else
    % Subplots
    figure;

    subplot(3,1,1);
    plot(tout, rad2deg(xoutFaulty(:,5)), 'r', 'LineWidth', 1.5); hold on;
    plot(tout, rad2deg(xout(:,5)), 'b--', 'LineWidth', 1.5);
    ylim([-275 600]);
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel('$\psi$ (deg)', 'Interpreter', 'latex');
    set(gca, "TickLabelInterpreter", 'latex');
    title('Effect of Sensor Fault on Heading', 'Interpreter', 'latex');
    legend('Faulty Heading', 'Reference Heading', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,2);
    stairs(tout, rad2deg(uoutFaulty(:,2)), 'r', 'LineWidth', 1.5); hold on;
    stairs(tout, rad2deg(uout(:,2)), 'b--', 'LineWidth', 1.5);
    ylabel('$\delta_r$ (deg)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-25 35]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Rudder Deflection Over Time', 'Interpreter', 'latex');
    legend('Faulty Deflection', 'Reference Deflection', 'Interpreter', 'latex');
    grid on;
    hold off;

    subplot(3,1,3);
    plot(tout, rad2deg(xoutFaulty(:,2)), 'r', 'LineWidth', 1.5); hold on
    plot(tout, rad2deg(xout(:,2)),  'b--', 'LineWidth', 1.5);
    ylabel('$r$ (deg/s)', 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylim([-40 40]);
    set(gca, "TickLabelInterpreter", 'latex');
    title('Yaw Rate Over Time', 'Interpreter', 'latex');
    legend('Faulty Yaw Rate', 'Reference Yaw Rate', 'Interpreter', 'latex');
    grid on;
    hold off
end