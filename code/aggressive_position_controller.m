function [f_des, error_acc] = position_controller(params, state, trajectory, n, kp, kd)
    error_acc = -kp.*[state.position(:, 1, n) - trajectory.position(:, 1, n)] - kd.*[state.position(:, 2, n) - trajectory.position(:, 2, n)];
    b3 = [0 0 1];
    f_des = params.mass * b3 * ([0; 0; params.gravity] + error_acc + trajectory.position(:, 3, n));
    f_des = max(0, f_des);
end