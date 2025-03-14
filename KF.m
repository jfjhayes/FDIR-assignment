function [x_est_hist, P_hist] = KF(A, B, C, Q, R, x0, P0, u_hist, y_hist)
    % Number of time steps
    N = size(y_hist, 1);
    
    % Initialize state estimate and covariance
    x_est = x0;
    P = P0;

    % Store results
    x_est_hist = zeros(N, length(x0));
    P_hist = zeros(N, numel(P));

    % Run Kalman Filter loop
    for i = 1:N
        % Get control input and measurement
        u = u_hist(i, :)';
        y = y_hist(i, :)';

        % Prediction step
        x_pred = A * x_est + B * u;
        P_pred = A * P * A' + Q;

        % Kalman gain
        K = P_pred * C' / (C * P_pred * C' + R);

        % Correction step
        x_est = x_pred + K * (y - C * x_pred);
        P = (eye(size(P)) - K * C) * P_pred;

        % Store results
        x_est_hist(i, :) = x_est';
        P_hist(i, :) = P(:)';
    end
end
