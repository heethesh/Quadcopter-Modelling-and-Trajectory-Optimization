function [time, trajectory] = make_trajectory(trajectory, case_, dt, opt)
    disp('Creating trajectory...');

    switch case_
        %% Null case
        case 0
            N = 1;
            time = dt;

        %% Takeoff trajectory
        case 1
            traj_z = [zeros(1, 2/dt) ones(1, opt.htime/dt)*opt.halt];
            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_z(n);
            end

        %% Land trajectory
        case 2
            traj_z = [ones(1, opt.htime/dt)*opt.halt zeros(1, 2/dt)];
            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_z(n);
                trajectory.angle(3, 1, n) = opt.hyaw;
            end

        %% Hover trajectory
        case 3
            traj_z = [ones(1, opt.htime/dt)*opt.halt];
            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_z(n);
                trajectory.angle(3, 1, n) = opt.hyaw;
            end

        %% Q2 - Position trajectory
        case 41
            traj_x = [zeros(1, 4/dt) ones(1, 3/dt)*0.1 ones(1, 3/dt)*0.2 ones(1, 3/dt)*0.3 ones(1, 7/dt)*0.4];
            traj_z = [zeros(1, 3/dt) ones(1, 14/dt)*0.5 zeros(1, 3/dt)];
            
            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(1, 1, n) = traj_x(n);
                trajectory.position(3, 1, n) = traj_z(n);
            end

        %% Q3 - Velocity trajectory
        case 42
            poz_z = 1; vel = 0.5; acc = 0.25;
            traj_z = [zeros(1, 2/dt) linspace(0, poz_z, 4/dt) linspace(poz_z, 0, 4/dt) zeros(1,  2/dt)];
            vel_z = [zeros(1, 2/dt) linspace(0, vel, 2/dt), linspace(vel, 0, 2/dt), linspace(0, -vel, 2/dt), linspace(-vel, 0, 2/dt), zeros(1,  2/dt)];
            acc_z = [zeros(1, 2/dt) ones(1, 2/dt)*acc, ones(1, 4/dt)*-acc, ones(1, 2/dt)*acc, zeros(1,  2/dt)];

            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_z(n);
                trajectory.position(3, 2, n) = vel_z(n);
                trajectory.position(3, 3, n) = acc_z(n);
            end

        %% Q5 - Gain selection trajectory
        case 5
            % traj_z = [ones(1, 10/dt)*0.1];
            traj_z = [ones(1, 10/dt)*0.1];
            traj_yaw = [ones(1, 10/dt)*deg2rad(15)];
            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_z(n);
                trajectory.angle(3, 1, n) = traj_yaw(n);
            end

        %% Q5 - Land trajectory
        case 50
            % traj_z = [ones(1, 10/dt)*0.1];
            traj_z = [ones(1, 10/dt)*0.1 linspace(0, 0.1, 1/dt)];
            traj_yaw = [ones(1, 10/dt)*deg2rad(15) linspace(0, deg2rad(15), 1/dt)];
            [ret, N] = size(traj_z);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_z(n);
                trajectory.angle(3, 1, n) = traj_yaw(n);
            end

        %% Elliptical trajectory
        case 6
            q8_ellipse = load('../data/Q8-XYZW-SSAA-T2-L3.mat');
            % q8_ellipse = load('../data/Q8-POS-SSAA-T1_5-L3-V3.mat');
            % vel = load('../data/Q8-VEL-SSAA-T2-L3.mat');
            % acc = load('../data/Q8-ACC-SSAA-T2-L3.mat');
            % jrk = load('../data/Q8-JRK-SSAA-T2-L3.mat');
            % snp = load('../data/Q8-SNP-SSAA-T2-L3.mat');

            raw_yaw = pi + atan2(q8_ellipse.y-1, q8_ellipse.x);
            traj_yaw2 = raw_yaw;
            EN = size(raw_yaw);

            offset = 0;
            for n = 3:EN(2)
                if abs(raw_yaw(n) - raw_yaw(n-2)) > deg2rad(170) offset = offset + pi; end
                traj_yaw2(n) = raw_yaw(n) + offset;
            end

            offset = 0;
            traj_yaw3 = traj_yaw2;
            for n = 3:EN(2)
                if abs(traj_yaw2(n) - traj_yaw2(n-2)) > deg2rad(200) offset = offset - 2*pi; end
                traj_yaw3(n) = traj_yaw2(n) + offset;
            end

            for n = 2:EN(2)
                if abs(traj_yaw2(n) - traj_yaw2(n-1)) > deg2rad(5) traj_yaw3(n) = traj_yaw3(n-1); end
            end

            traj_x = [zeros(1, 100) q8_ellipse.x zeros(1, 100)];
            traj_y = [zeros(1, 100) q8_ellipse.y zeros(1, 100)];
            % vel_x = [zeros(1, 100) vel.x zeros(1, 100)];
            % vel_y = [zeros(1, 100) vel.y zeros(1, 100)];
            % acc_x = [zeros(1, 100) acc.x zeros(1, 100)];
            % acc_y = [zeros(1, 100) acc.y zeros(1, 100)];
            % jrk_x = [zeros(1, 100) jrk.x zeros(1, 100)];
            % jrk_y = [zeros(1, 100) jrk.y zeros(1, 100)];
            % snp_x = [zeros(1, 100) snp.x zeros(1, 100)];
            % snp_y = [zeros(1, 100) snp.y zeros(1, 100)];

            traj_yaw = [zeros(1, 100) traj_yaw3 linspace(traj_yaw3(end), 5*2*pi, 100)];

            [ret, N] = size(traj_x);
            time = N * dt;

            for n = 1:N
                trajectory.position(1, 1, n) = traj_x(n);
                trajectory.position(2, 1, n) = traj_y(n);
                % trajectory.position(1, 2, n) = vel_x(n);
                % trajectory.position(2, 2, n) = vel_y(n);
                % trajectory.position(1, 3, n) = acc_x(n);
                % trajectory.position(2, 3, n) = acc_y(n);
                % trajectory.position(1, 4, n) = jrk_x(n);
                % trajectory.position(2, 4, n) = jrk_y(n);
                % trajectory.position(1, 5, n) = snp_x(n);
                % trajectory.position(2, 5, n) = snp_y(n);

                trajectory.position(3, 1, n) = 1;
                if opt.pirouette
                    trajectory.angle(3, 1, n) = traj_yaw(n);
                end
            end

        %% Q5 - Position trajectory
        case 7
            [traj_p, traj_v, traj_a] = bounded_acceleration(15);
            [ret, N] = size(traj_p);
            time = N * dt;
            for n = 1:N
                trajectory.position(3, 1, n) = traj_p(n);
                trajectory.position(3, 2, n) = traj_v(n);
                trajectory.position(3, 3, n) = traj_a(n);
            end

        %% Aggressive maneuvers
        case 10
            % vel_roll = [zeros(1, 3/dt) ones(1, 0.25/dt)*deg2rad(30) ones(1, 0.25/dt)*-deg2rad(180) zeros(1, 10.5/dt)];
            % roll = [zeros(1, 3/dt) linspace(0, deg2rad(5), 1/dt) linspace(deg2rad(5), deg2rad(0), 1/dt) ones(1, 10/dt)*deg2rad(0)];
            roll = [zeros(1, 3/dt) linspace(0, deg2rad(90), 0.5/dt) linspace(deg2rad(90), deg2rad(-110), 0.5/dt) zeros(1, 10/dt)];
            traj_x = [zeros(1, 3/dt) ones(1, 1/dt)*0.5 ones(1, 10/dt)*0.5];
            traj_z = [zeros(1, 3/dt) ones(1, 1/dt)*2 ones(1, 10/dt)*2.5];
            % acc_x = [zeros(1, 2/dt) ones(1, 0.2/dt)*50 zeros(1, 2/dt)];

            [ret, N] = size(roll);
            time = N * dt;
            for n = 1:N
                trajectory.angle(1, 1, n) = roll(n);
                % trajectory.angle(1, 2, n) = vel_roll(n);
                trajectory.position(1, 1, n) = traj_x(n);
                trajectory.position(3, 1, n) = traj_z(n);
                % trajectory.position(2, 3, n) = acc_x(n);
            end

        case 11
            vel_roll = [zeros(1, 3/dt) ones(1, 0.375/dt)*deg2rad(300) ones(1, 0.375/dt)*-deg2rad(300) zeros(1, 20/dt)];
            % roll = [zeros(1, 3/dt) linspace(deg2rad(719), deg2rad(539), 1/dt) linspace(deg2rad(359), deg2rad(539), 0.75/dt) ones(1, 10/dt)*deg2rad(360)];
            roll = [zeros(1, 3/dt) linspace(0, deg2rad(360), 2/dt) ones(1, 8.5/dt)*deg2rad(360)]; % linspace(deg2rad(90), 0, 0.5/dt)
            pitch = [zeros(1, 5/dt) linspace(deg2rad(0), deg2rad(-32), 1/dt) linspace(deg2rad(-32), deg2rad(0), 0.75/dt) ones(1, 10/dt)*deg2rad(0)];
            
            traj_x = [zeros(1, 0.5/dt) ones(1, 5/dt)*2 ones(1, 15/dt)*50];
            traj_y = [zeros(1, 6/dt) ones(1, 20/dt)];
            traj_z = [zeros(1, 3/dt) linspace(10, 15, 1/dt) linspace(0, 1, 0.80/dt) ones(1, 20/dt)*0];
            % acc_x = [zeros(1, 2/dt) ones(1, 0.2/dt)*50 zeros(1, 2/dt)];

            [ret, N] = size(roll);
            time = N * dt;
            for n = 1:N
                trajectory.angle(2, 1, n) = roll(n);
                trajectory.angle(1, 1, n) = pitch(n);
                trajectory.position(1, 1, n) = traj_x(n);
                trajectory.position(2, 1, n) = traj_y(n);
                trajectory.position(3, 1, n) = traj_z(n);
                % trajectory.position(2, 3, n) = acc_x(n);
            end

        %% Default case
        otherwise
            disp('Trajectory mode not implemeneted!')
    end

    trajectory.position = trajectory.position(:, :, 1:N);
    trajectory.angle = trajectory.angle(:, :, 1:N);

end