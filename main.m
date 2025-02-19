%% ENG5031: Fault Detection, Isolation, & Recovery 5 - Assignment
clear variables
clf

% Initialise Variables & Constants %
x = zeros(5,5);                             % states
xdot = zeros(5,1);                          % state derivatives

% Initial Conditions %
u0 = 30;                                    % longitudinal velocity (ms^-1)
h = 500;                                    % altitude (m)

% Simulation Conditions %
stepSize = 0.01;                            % integration step size
commInterval = 0.01;                        % communications interval
endTime = 60;                               % simulation duration (s)
i = 0;                                      % initialise counter


% Simulation Loop %
for time = 0:stepSize:endTime

end

% Output Plotting %