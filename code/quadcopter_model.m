function [trajectory, state, motor_rpm] = quadcopter_model(params, state, trajectory, gains, motor_rpm, n, dt)
    % Get forces from position controller
    [force_des, error_dxx] = position_controller(params, state, trajectory, n-1, gains.Kp, gains.Kv);
    
    % Get torques from attitude controller
    [trajectory, torque_des] = attitude_controller(params, state, trajectory, n-1, gains.Kr, gains.Kw, error_dxx);

    % Get motor RPMs from motor controller
    motor_rpm = motor_controller(params, [force_des; torque_des], motor_rpm, dt);
    
    % Calculate new state using dynamic model
    [state.position(:, :, n), state.angle(:, :, n)] = dynamic_model(params, state, n-1, motor_rpm, dt);
end
