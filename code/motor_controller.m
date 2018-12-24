function W_output = motor_controller(params, forces, W_initial, dt)
%% Compute desired motor RPMs
W_des_sq = mixing_matrix(params)\forces;
W_des = sqrt(W_des_sq);
% disp("Motor Desired RPMs: " + W_des);

%% Integrate to calculate output motor RPMs
% W_act = (params.motor_constant * (W_des - W_initial) * dt) + W_initial
[t, W_act] = ode45(@(t, wi) params.motor_constant.*(W_des - wi), dt, W_initial);
W_act = W_act(end, :)';
% disp("Motor Actual RPMs: " + W_act(:, end));

%% Limit motor RPMs and discard imaginary part
W_output = clip(real(W_act), params.rpm_min, params.rpm_max);
% disp("Motor Actual Clipped RPMs: " + W_act(:, end));

end