function [trajectory, torques] = attitude_controller(params, state, trajectory, n, kr, kw, error_acc)
    %% Extract variables
    w_act     = state.angle(:, 1, n);
    dw_act    = state.angle(:, 2, n);
    acc_des   = trajectory.position(:, 3, n);
    jerk_des  = trajectory.position(:, 4, n);
    snap_des  = trajectory.position(:, 5, n);
    w_des     = trajectory.angle(:, 1, n);
    dw_des    = trajectory.angle(:, 2, n);
    psi_des   = trajectory.angle(3, 1, n);
    dpsi_des  = trajectory.angle(3, 2, n);
    ddpsi_des = trajectory.angle(3, 3, n);

    %% Desired angular position (direct substitution)
    % w_des = (1/params.gravity).*[sin(psi_des) -cos(psi_des); cos(psi_des) sin(psi_des)] * (error_acc(1:2) + acc_des(1:2));
    % w_des = [w_des; psi_des];
    Rd = eul2rotm([flipud(w_des)']);
    Re = eul2rotm([flipud(w_act)']);
    Rt = 0.5 * (Rd'*Re - Re'*Rd);
    eR = [Rt(3, 2); Rt(1, 3); Rt(2, 1)];

    %% Desired angular velocity (direct substitution)
    % dw_des = angular_velocity(error_acc(1), error_acc(2), acc_des(1), acc_des(2), jerk_des(1), jerk_des(2), psi_des, dpsi_des, params.gravity, 0);
    % dw_des = [dw_des; dpsi_des];
    eWR = state.angle(:, 2, n) - (Re'*Rd) * trajectory.angle(:, 2, n);

    %% Desired angular acceleration (direct substitution)
    dww_des = angular_acceleration(error_acc(1), error_acc(2), 0, 0, acc_des(1), acc_des(2), jerk_des(1), jerk_des(2), snap_des(1), snap_des(2), psi_des, dpsi_des, ddpsi_des, params.gravity, 0);
    dww_des = [dww_des; ddpsi_des];

    %% Update desired angle trajectory
    trajectory.angle(:, :, n) = [w_des, dw_des, dww_des];

    %% Calculate torques
    torques = params.inertia * (-kr.*eR -kw.*eWR + dww_des);
end
