function [state, motor_rpm] = state_space_model(params, state, trajectory, gains, motor_rpm, m, dt)
    n = m - 1;    
    roll_des = 0;
    pitch_des = 0;

    %% State space matrices
    A = state_space_A(state.angle(1, 1, n), state.angle(2, 1, n), state.angle(3, 1, n), ...
        roll_des, pitch_des, trajectory.angle(3, 1, n), ...
        params.inertia(1, 1), params.inertia(2, 2), params.inertia(3, 3), ...
        params.mass, params.gravity, params.mass * params.gravity);

    B = state_space_B(state.angle(1, 1, n), state.angle(2, 1, n), state.angle(3, 1, n), ...
        roll_des, pitch_des, trajectory.angle(3, 1, n), ...
        params.inertia(1, 1), params.inertia(2, 2), params.inertia(3, 3), ...
        params.mass, params.gravity, params.mass * params.gravity);

    C = zeros(4, 12);
    C(1, 1) = 1; C(2, 2) = 1; C(3, 3) = 1; C(4, 6) = 1;

    D = 0;

    X = [state.position(1:3, 1, n); state.angle(1:3, 1, n); ...
         state.position(1:3, 2, n); state.angle(1:3, 2, n)];

    sys = ss(A, B, C, D);
    K = lqr(A, B, gains.Q, gains.R);

    %% State space model
    Y_des = [trajectory.position(1:3, 1, n+1); trajectory.angle(3, 1, n+1)];
    V = (-C * ((A - B*K)\B))\Y_des;
    U_fb = -K * X;
    U_error = V + U_fb;
    
    % Get motor RPMs from motor controller
    U_error(1) = U_error(1) + (params.mass * params.gravity);
    motor_rpm = motor_controller(params, U_error, motor_rpm, dt);
    
    % Calculate new state using dynamic model
    U_act = mixing_matrix(params) * motor_rpm.^2;
    U_act(1) = U_act(1) - (params.mass * params.gravity);

    dX = (A * X) + (B * U_act);
    [t, X_new] = ode45(@(t, X_n) ((A * X_n) + (B * U_act)), dt, X);
    X_new = X_new(end, :)';

    state.position(1:3, 1, n+1) = X_new(1:3);
    state.position(1:3, 2, n+1) = X_new(7:9);
    state.angle(1:3, 1, n+1) = X_new(4:6);
    state.angle(1:3, 2, n+1) = X_new(10:12);

    %% Unit Testing
    % syms r p y rdes pdes ydes J11 J22 J33 m g force;
    % A = state_space_A(0, 0, y, 0, 0, ydes, J11, J22, J33, m, g, m * g);
    % B = state_space_B(0, 0, y, 0, 0, ydes, J11, J22, J33, m, g, m * g);
end