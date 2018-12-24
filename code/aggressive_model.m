function [trajectory, state, motor_rpm, force_des, torque_des] = aggressive_model(params, state, trajectory, gains, gains_agg, motor_rpm, n, dt)
    % Get forces from position controller
    [force_des, error_dxx] = aggressive_position_controller(params, state, trajectory, n-1, gains.Kp, gains.Kv);
    
    % Get torques from attitude controller
    % if (n*diff(dt) >= 3) && (n*diff(dt) <= 4)
    % if trajectory.angle(1, 1, n-1) ~= 0
    %     % force_des = 0;
    %     torque_des = params.inertia * (...
    %         -gains_agg.Kr.*[state.angle(:, 1, n-1) - trajectory.angle(:, 1, n-1)] ...
    %         -gains_agg.Kw.*[state.angle(:, 2, n-1) - trajectory.angle(:, 2, n-1)]);
    %     torque_des
    % else
        [trajectory, torque_des] = aggressive_attitude_controller(params, state, trajectory, n-1, gains.Kr, gains.Kw, error_dxx);
    % end

    % Get motor RPMs from motor controller
    motor_rpm = motor_controller(params, [force_des; torque_des], motor_rpm, dt);
    
    % Calculate new state using dynamic model
    [state.position(:, :, n), state.angle(:, :, n)] = dynamic_model(params, state, n-1, motor_rpm, dt);
end
