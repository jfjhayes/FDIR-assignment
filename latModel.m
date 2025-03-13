function xdot = latModel(x, u)

    % State Space Model (from assignment brief) %

    % A Matrix %
    A = [-8.5905, 2.0389, -14.5793, 0, 0;       % pdot
        0.3801, -2.2406,  17.5075, 0, 0;        % rdot
        0.0692, -0.7468, -0.3510, 0.2442, 0;    % betadot
        1, 0.0694, 0, 0, 0;                     % phidot
        0, 1, 0, 0, 0];                         % psidot

    % B Matix %
    B = [19.6050, -0.6821;                      % pdot
        -0.8672, 3.4921;                        % rdot
        0, 0;                                   % betadot
        0, 0;                                   % phidot
        0, 0];                                  % psidot

    % Update State Space %
    xdot = (A * x) + (B * u);
end
