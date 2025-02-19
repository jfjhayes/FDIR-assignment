%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
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
endTime = 60;                               % simulation duration (s)
i = 0;                                      % initialise counter

% Preallocate storage arrays (stops MATLAB being angry w/ me) %
numSteps = endTime / commInterval + 1;
tout = zeros(numSteps, 1);                  % time
xdotout = zeros(numSteps, length(xdot));    % state derivatives
xout = zeros(numSteps, length(x));          % states    

% Simulation Loop %
for time = 0:stepSize:endTime

    % Data Logging for each commInterval %
    if rem(time, commInterval)==0
        i = i+1;
        tout(i) = time;	 	                % store time
        xdotout(i,:) = xdot;	            % store state derivatives
        xout(i,:) = x;		                % store states
    end    

    % State Space Updates %
    xdot = latModel(x, u);                  % update state derivatives
    x = RK4(@latModel, stepSize, x, u);     % find states using RK4

end

% Output Plotting %