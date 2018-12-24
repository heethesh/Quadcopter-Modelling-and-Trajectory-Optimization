function [trajectory, samples] = trajectory_planner()
    % Optimize trajectories
    time_scale = 1;
    x = zeros(12, 4);
    y = zeros(12, 4);
    A = [1 0 0 0; 1 1 1 1; 0 1 0 0; 0 1 2 3];
    B = struct('traj1', [], 'traj2', [], 'traj3', [], 'traj4', []);
    
    %% Trajectory X - Phase 1 - Constraints
    B.traj1 = [0; 2; 0; 0]; B.traj2 = [2; 0; 0];
    B.traj3 = [0; -2; NaN; 0]; B.traj4 = [-2; 0; 0; 1];
    x(1:4, :) = trajectory_optimizer(A, B, time_scale, [0, 0, 3, 0]);

    % Trajectory Y - Phase 1 - Constraints
    B.traj1 = [0; 1; 0]; B.traj2 = [1; 2; NaN; 0];
    B.traj3 = [2; 1; 0]; B.traj4 = [1; 0; NaN; 0];
    y(1:4, :) = trajectory_optimizer(A, B, time_scale, [0, 3, 0, 3]);

    %% Trajectory X - Phase 2 - Constraints
    B.traj1 = [0; 2; 1; 0]; B.traj2 = [2; 0; 0];
    B.traj3 = [0; -2; NaN; 0]; B.traj4 = [-2; 0; 0; 1];
    x(5:8, :) = trajectory_optimizer(A, B, time_scale, [0, 0, 3, 0]);

    % Trajectory Y - Phase 2 - Constraints
    B.traj1 = [0; 1; 0]; B.traj2 = [1; 2; NaN; 0];
    B.traj3 = [2; 1; 0]; B.traj4 = [1; 0; NaN; 0];
    y(5:8, :) = trajectory_optimizer(A, B, time_scale, [0, 3, 0, 3]);

    %% Trajectory X - Phase 3 - Constraints
    B.traj1 = [0; 2; 1; 0]; B.traj2 = [2; 0; 0];
    B.traj3 = [0; -2; NaN; 0]; B.traj4 = [-2; 0; 0; 0];
    x(9:12, :) = trajectory_optimizer(A, B, time_scale, [0, 0, 3, 0]);

    % Trajectory Y - Phase 3 - Constraints
    B.traj1 = [0; 1; 0]; B.traj2 = [1; 2; NaN; 0];
    B.traj3 = [2; 1; 0]; B.traj4 = [1; 0; NaN; 0];
    y(9:12, :) = trajectory_optimizer(A, B, time_scale, [0, 3, 0, 3]);

    % Generate trajectories
    samples = time_scale/0.005;
    t = linspace(0, time_scale, samples);
    x = fliplr(x); y = fliplr(y);
    trajx = zeros(12, samples);
    trajy = zeros(12, samples);
    trajectory = zeros(2, 12*samples, 4);
    p = @(c, t) polyval(c, t);

    % Plot trajectories and compute differentials
    for d = 1:4
        hold on; grid on;
        for i = 1:12
            trajx(i, :) = p(x(i, 1:5-d), t);
            trajy(i, :) = p(y(i, 1:5-d), t);
            if 4-d > 0
                x(i, 1:4-d) = polyder(x(i, 1:5-d)); x(i, 5-d:4) = zeros(1, d); 
                y(i, 1:4-d) = polyder(y(i, 1:5-d)); y(i, 5-d:4) = zeros(1, d); 
            end
            if d == 2
                plot(trajx(i, :), trajy(i, :), '.');
            end
        end
        % hold off;
        trajectory(1, :, d) = reshape(trajx, 1, 12*samples);
        trajectory(2, :, d) = reshape(trajy, 1, 12*samples);
        % pause(4);
    end
end