function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    A = [1 0 0.033 0; 0 1 0 0.033; 0 0 1 0; 0 0 0 1];
    C = eye(2, 4);
    sig_m = 0.1 * eye(4);
    sig_o = 0.05 * eye(2);

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = sig_m;
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    param.P = A*param.P*A' + sig_m; % covariance prediction
    R = C*param.P*C' + sig_o;
    
    state_pred = A*state'; % prediction
    K = param.P*C'*inv(R+C*param.P*C'); % Kalman gain
    state = (state_pred + K*([x;y] - C*state_pred))'; % update
    param.P = param.P - K*C*param.P; % update covariance of state
    
    % Predict 330ms into the future
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;

end
