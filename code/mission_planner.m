function [all_trajectory, model, gains] = mission_planner(mission, sim_dt)
    disp('Planning mission...');

    IDLE = 0;
    TAKEOFF = 1;
    HOVER = 2;
    TRACK = 3;
    HOVER2 = 4;
    LAND = 5;

    PID_MODEL = 1;
    LQR_MODEL = 2;
    AGG_MODEL = 3;

    %% PID gains
    def_pid_gains = struct(...
        'Kp', [17; 17; 20], ...
        'Kv', [6.6; 6.6; 9], ...
        'Kr', [190; 198; 80], ...
        'Kw', [30; 30; 17.88]);

    %% LQR gains
    def_lqr_gains = struct(...
        'Q', 0.1 * eye(12), ...
        'R', 2 * eye(4));

    gains = cell(1, 5);
    model = zeros(1, 5);

    %% Default trajectory
    trajectory = struct(...
        'position', zeros(3, 5, 12000), ...
        'angle', zeros(3, 3, 12000));

    %% Plan missions (NOTE: Matlab doesn't need break after every case)
    switch mission

        % Position tracking
        case 3
            [t1, takeoff] = make_trajectory(trajectory, 0, sim_dt);
            [t2, hover] = make_trajectory(trajectory, 0, sim_dt);
            [t3, tracking] = make_trajectory(trajectory, 41, sim_dt);
            [t4, land] = make_trajectory(trajectory, 0, sim_dt);

            pid_gains = struct(...
                'Kp', [22; 17; 42], ...
                'Kv', [6.6; 6.6; 10], ...
                'Kr', [190; 250; 80], ...
                'Kw', [30; 20; 17.88]);

            lqr_gains = struct(...
                'Q', diag([20 0.1 20 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]), ...
                'R', diag([2 2 2 2]));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = PID_MODEL; gains{TRACK} = pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Velocity tracking
        case 4
            [t1, takeoff] = make_trajectory(trajectory, 0, sim_dt);
            [t2, hover] = make_trajectory(trajectory, 0, sim_dt);
            [t3, tracking] = make_trajectory(trajectory, 42, sim_dt);
            [t4, land] = make_trajectory(trajectory, 0, sim_dt);

            pid_gains = struct(...
                'Kp', [17; 17; 50], ...
                'Kv', [6.6; 6.6; 70], ...
                'Kr', [190; 198; 80], ...
                'Kw', [30; 30; 17.88]);

            lqr_gains = struct(...
                'Q', diag([0.1 0.1 100 0.1 0.1 0.1 0.1 0.1 20 0.1 0.1 0.1]), ...
                'R', diag([2 2 1 2]));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = LQR_MODEL; gains{TRACK} = lqr_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Basic takeoff, track and land using state machine
        case 5
            [t1, takeoff] = make_trajectory(trajectory, 1, sim_dt, struct('htime', 5, 'halt', 1));
            [t2, hover] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 5, 'halt', 1, 'hyaw', 0));
            [t3, tracking] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 5, 'halt', 1));
            [t4, land] = make_trajectory(trajectory, 2, sim_dt, struct('htime', 5, 'halt', 1, 'hyaw', 0));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = PID_MODEL; gains{TRACK} = def_pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Gain selection and tuning
        case 6
            [t1, takeoff] = make_trajectory(trajectory, 0, sim_dt);
            [t2, hover] = make_trajectory(trajectory, 0, sim_dt);
            [t3, tracking] = make_trajectory(trajectory, 5, sim_dt);
            [t4, land] = make_trajectory(trajectory, 50, sim_dt);

            pid_gains = struct(...
                'Kp', [17; 17; 20], ...
                'Kv', [6.6; 6.6; 9], ...
                'Kr', [190; 198; 80], ...
                'Kw', [30; 30; 30]);

            lqr_gains = struct(...
                'Q', diag([0.1 0.1 20 0.1 0.1 5 0.1 0.1 0.1 0.1 0.1 0.1]), ...
                'R', diag([2 2 2 2]));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = LQR_MODEL; gains{TRACK} = lqr_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Bounded acceleration trajectory
        case 7
            [t1, takeoff] = make_trajectory(trajectory, 1, sim_dt, struct('htime', 5, 'halt', 1));
            [t2, hover] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 5, 'halt', 1, 'hyaw', 0));
            [t3, tracking] = make_trajectory(trajectory, 7, sim_dt);
            [t4, hover2] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 5, 'halt', 10));
            [t5, land] = make_trajectory(trajectory, 2, sim_dt, struct('htime', 5, 'halt', 10, 'hyaw', 0));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = PID_MODEL; gains{TRACK} = def_pid_gains;
            model(HOVER2) = PID_MODEL; gains{HOVER2} = def_pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Elliptical trajectory
        case 8
            [t1, takeoff] = make_trajectory(trajectory, 1, sim_dt, struct('htime', 5, 'halt', 1));
            [t2, hover] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 3, 'halt', 1, 'hyaw', 0));
            [t3, tracking] = make_trajectory(trajectory, 6, sim_dt, struct('pirouette', false));
            [t4, land] = make_trajectory(trajectory, 2, sim_dt, struct('htime', 5, 'halt', 1, 'hyaw', 0));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = PID_MODEL; gains{TRACK} = def_pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Elliptical trajectory - Pirouette
        case 9
            [t1, takeoff] = make_trajectory(trajectory, 1, sim_dt, struct('htime', 5, 'halt', 1));
            [t2, hover] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 3, 'halt', 1, 'hyaw', 0));
            [t3, tracking] = make_trajectory(trajectory, 6, sim_dt, struct('pirouette', true));
            [t4, hover2] = make_trajectory(trajectory, 3, sim_dt, struct('htime', 3, 'halt', 1, 'hyaw', 5*2*pi));
            [t5, land] = make_trajectory(trajectory, 2, sim_dt, struct('htime', 5, 'halt', 1, 'hyaw', 5*2*pi));

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = PID_MODEL; gains{TRACK} = def_pid_gains;
            model(HOVER2) = PID_MODEL; gains{HOVER2} = def_pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        % Flip
        case 10
            [t1, takeoff] = make_trajectory(trajectory, 0, sim_dt, 0);
            [t2, hover] = make_trajectory(trajectory, 0, sim_dt, 0);
            [t3, tracking] = make_trajectory(trajectory, 10, sim_dt, 0);
            [t4, land] = make_trajectory(trajectory, 0, sim_dt, 0);

            agg_pid_gains = struct(...
                'Kp', [17; 17; 20], ...
                'Kv', [6.6; 6.6; 9], ...
                'Kr', [190; 198; 80], ...
                'Kw', [30; 30; 17.88]);

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = AGG_MODEL; gains{TRACK} = agg_pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        case 11
            [t1, takeoff] = make_trajectory(trajectory, 0, sim_dt, 0);
            [t2, hover] = make_trajectory(trajectory, 0, sim_dt, 0);
            [t3, tracking] = make_trajectory(trajectory, 11, sim_dt, 0);
            [t4, land] = make_trajectory(trajectory, 0, sim_dt, 0);

            agg_pid_gains = struct(...
                'Kp', [17; 17; 20], ...
                'Kv', [6.6; 6.6; 9], ...
                'Kr', [190; 198; 80], ...
                'Kw', [30; 30; 17.88]);

            model(TAKEOFF) = PID_MODEL; gains{TAKEOFF} = def_pid_gains;
            model(HOVER) = PID_MODEL; gains{HOVER} = def_pid_gains;
            model(TRACK) = AGG_MODEL; gains{TRACK} = agg_pid_gains;
            model(LAND) = PID_MODEL; gains{LAND} = def_pid_gains;

        otherwise
            disp('Mission not implemeneted!')
    end

    all_trajectory = cell(1, 5);
    all_trajectory{TAKEOFF} = takeoff;
    all_trajectory{HOVER} = hover;
    all_trajectory{TRACK} = tracking;
    all_trajectory{HOVER2} = hover;
    all_trajectory{LAND} = land;
    model(HOVER2) = model(HOVER); 
    gains{HOVER2} = gains{HOVER};
end
