function [all_trajectory] = state_machine(params, MISSION)

    IDLE = 0;
    TAKEOFF = 1;
    HOVER = 2;
    TRACK = 3;
    HOVER2 = 4;
    LAND = 5;

    PID_MODEL = 1;
    LQR_MODEL = 2;
    AGG_MODEL = 3;

    %% Default simulation parameters
    sim_start_time = 0;
    sim_dt = 0.005;
    sim_end_time = 60.0;
    TN = sim_end_time/sim_dt;
    lookahead_time = 0.01;
    lookahead = lookahead_time/sim_dt;
    error_tol = 1e-4;
    max_error_settling_time = 5;
    EN = max_error_settling_time/sim_dt;

    %% Mission parameters
    [all_trajectory, models, gains] = mission_planner(MISSION, sim_dt);

    %% Variables
    all_state = cell(1, 5);
    for i = 1:5
        all_state{i} = struct(...
            'position', zeros(3, 2, TN), ...
            'angle', zeros(3, 2, TN));
    end
    motor_rpm = clip(zeros(4, 1), params.rpm_min, params.rpm_max);

    % Debug
    % forces = zeros(1, N);
    % torques = zeros(3, N);

    % Main loop
    sprintf('\n');
    disp('Running simulation...');
    actual_time = zeros(1, 5);

    for s = 1:5
        disp([newline + "Trajectory: " + num2str(s)]);
        state = all_state{s};
        trajectory = all_trajectory{s};

        if s >= 2
            state.position(:, :, 1) = last_state_pos;
            state.angle(:, :, 1) = last_state_ang;
        end

        N = numel(trajectory.position(1, 1, :));
        time = linspace(sim_start_time, (N+EN)*sim_dt, N+EN+1); time = time(1:end-1);

        % Extend trajectory
        final_pos = repmat(trajectory.position(:, :, N), 1, 1, EN);
        final_ang = repmat(trajectory.angle(:, :, N), 1, 1, EN);
        trajectory.position = cat(3, trajectory.position, final_pos);
        trajectory.angle = cat(3, trajectory.angle, final_ang);
        
        % close all;
        % plot(time, mat2row(trajectory.position, 3, 1));
        % w = waitforbuttonpress;

        model = models(s);
        mgain = gains{s};
        
        stable_count = 0;
        last_stable = 0;
        stable_time = 0.5;
        DONE = false;

        for n = 2:(N + EN)
            % Time step and lookahead
            dt = [0 sim_dt];

            % Quadcopter PID model
            if model == PID_MODEL
                [trajectory, state, motor_rpm] = quadcopter_model(params, state, trajectory, mgain, motor_rpm, n, dt);
            end

            % State space model
            if model == LQR_MODEL
                [state, motor_rpm] = state_space_model(params, state, trajectory, mgain, motor_rpm, n, dt);
            end

            % Aggressive model
            if model == AGG_MODEL
                [trajectory, state, motor_rpm, force, torque] = aggressive_model(params, state, trajectory, gains{PID_MODEL}, gains{AGG_MODEL}, motor_rpm, n, dt);
            end

            % Error checking
            if n > N
                if ~DONE 
                    DONE = true;
                    disp('Trajectory complete, waiting for stabilisation');
                end
            end

            if DONE
                if abs(state.position(3, 1, n) - trajectory.position(3, 1, n)) < error_tol 
                % if abs(state.position(1, 1, n) - trajectory.position(1, 1, n)) < error_tol ...
                % && abs(state.position(2, 1, n) - trajectory.position(2, 1, n)) < error_tol ...
                % && abs(state.position(3, 1, n) - trajectory.position(3, 1, n)) < error_tol ...
                % && abs(state.position(1, 2, n) - trajectory.position(1, 2, n)) < error_tol ...
                % && abs(state.position(2, 2, n) - trajectory.position(2, 2, n)) < error_tol ...
                % && abs(state.position(3, 2, n) - trajectory.position(3, 2, n)) < error_tol

                    if last_stable == 0 last_stable = n; end
                    if (n - last_stable) <= 1
                        stable_count = stable_count + 1;
                        last_stable = n;
                    else
                        stable_count = 0;
                    end

                    if (stable_count >= stable_time/sim_dt)
                        disp('Stabilisation complete');
                        actual_time(s) = n * sim_dt;
                        last_state_pos = state.position(:, :, n);
                        last_state_ang = state.angle(:, :, n);
                        break;
                    end
                end
            end

            if n == N + EN
                disp('State did not stabilise');
                actual_time(s) = n * sim_dt;
                break;
            end
        end

        state.position = state.position(:, :, 1:n);
        state.angle = state.angle(:, :, 1:n);
        trajectory.position = trajectory.position(:, :, 1:n);
        trajectory.angle = trajectory.angle(:, :, 1:n);

        all_state{s} = state;
        all_trajectory{s} = trajectory;
    end    

    %% Save data for simulation
    save_sim_data(all_state, all_trajectory, actual_time, sim_dt);

    %% Plot all graphs
    for s = 1:5
        % hold off; close all;
        if s ~= 3 continue; end
        state = all_state{s};
        trajectory = all_trajectory{s};
        time = linspace(0, actual_time(s), actual_time(s)/sim_dt + 1); time = time(1:end-1);
        % plotter(trajectory, state, time);
        % w = waitforbuttonpress;
    end

    % stepinfo(mat2row(state.position(:, :, 1:numel(time)), 3, 1), time, 0.1, 'SettlingTimeThreshold', 0.1)
    % stepinfo(mat2row(state.position(:, :, 1:numel(time)), 3, 2), time, 0, 'SettlingTimeThreshold', 0.1)
    % stepinfo(mat2row(state.angle(:, :, 1:numel(time)), 3, 1), time, deg2rad(15), 'SettlingTimeThreshold', 0.1)
    % stepinfo(mat2row(state.angle(:, :, 1:numel(time)), 3, 2), time, 0, 'SettlingTimeThreshold', 0.1)
end 
