function [outputs] = plotter(trajectory, state, time)
    der = ["Position - ", "Velocity - "];
    lin = ["Linear ", "Angular "];
    axs1 = ["X (m)", "Y (m)", "Z (m)"; "X (m/s)", "Y (m/s)", "Z (m/s)"];
    axs2 = ["Roll (rad)", "Pitch (rad)", "Yaw (rad)"; "Roll (rad/s)", "Pitch (rad/s)", "Yaw (rad/s)"];
    err_title = ["Pose Error Plots", "Velocity Error Plots"];

    %% Master Plotter (All)
    for ord = 1:2
        figure(ord);
        suptitle([der(ord) + "Responses"]);

        for ax = 1:3
            subplot(2, 3, ax);
            grid on; hold on;
            plot(time, mat2row(trajectory.position, ax, ord));
            plot(time, mat2row(state.position, ax, ord));
            ylabel([lin(1) + der(ord) + axs1(ord, ax)]);
            xlabel("Time (s)");
            legend({"Desired", "Actual"});
        end

        for ax = 1:3
            subplot(2, 3, ax+3);
            grid on; hold on;
            plot(time, mat2row(trajectory.angle, ax, ord));
            plot(time, mat2row(state.angle, ax, ord));
            ylabel([lin(2) + der(ord) + axs2(ord, ax)]);
            xlabel("Time (s)");
            legend({"Desired", "Actual"});
        end
    end

    % Question 2/3
    % figure(1);
    % size(time)

    % Q2 only Position X
    % suptitle("Linear Position, Velocity and Desired Acceleration in Z");
    % subplot(2, 1, 1);
    % grid on; hold on;
    % plot(time, mat2row(trajectory.position, 1, 1));
    % plot(time, mat2row(state.position, 1, 1));
    % ylabel([lin(1) + der(1) + axs1(1, 1)]);
    % xlabel("Time (s)");
    % legend({"Desired", "Actual"});

    % Q2/Q3 Position Z
    % subplot(3, 1, 1);
    % subplot(2, 1, 2);
    % grid on; hold on;
    % plot(time, mat2row(trajectory.position, 3, 1));
    % plot(time, mat2row(state.position, 3, 1));
    % ylabel([lin(1) + der(1) + axs1(1, 3)]);
    % xlabel("Time (s)");
    % legend({"Desired", "Actual"});

    % Q3 only (Velocity Z)
    % subplot(3, 1, 2);
    % grid on; hold on;
    % plot(time, mat2row(trajectory.position, 3, 2));
    % plot(time, mat2row(state.position, 3, 2));
    % ylabel([lin(1) + der(2) + axs1(1, 3)]);
    % xlabel("Time (s)");
    % legend({"Desired", "Actual"});

    % Q3 only (Acceleration Z)
    % subplot(3, 1, 3);
    % grid on; hold on;
    % plot(time, mat2row(trajectory.position, 3, 3));
    % ylabel([lin(1) + "Acceleration - " + axs1(1, 3)]);
    % xlabel("Time (s)");

    %% Error plots
    for ord = 1:2
        figure(ord+2);
        suptitle(err_title(ord));
        % suptitle(["Cumulative " + err_title(ord)]);
        err_pos = trajectory.position(:, 1:2, :) - state.position;
        err_ang = trajectory.angle(:, 1:2, :) - state.angle;

        for ax = 1:3
            subplot(2, 3, ax);
            grid on; hold on;
            plot(time, mat2row(err_pos, ax, ord));
            % plot(time, cumsum(mat2row(err_pos, ax, ord)));
            ylabel(["Error in " + lin(1) + der(ord) + axs1(ord, ax)]);
            xlabel("Time (s)");
        end

        for ax = 1:3
            subplot(2, 3, ax+3);
            grid on; hold on;
            plot(time, mat2row(err_ang, ax, ord));
            % plot(time, cumsum(mat2row(err_ang, ax, ord)));
            ylabel(["Error in " + lin(2) + der(ord) + axs2(ord, ax)]);
            xlabel("Time (s)");
        end

    % Trajectory Plotter
    % figure(5);
    % grid on; hold on;
    % plot3(mat2row(trajectory.position, 1, 1), mat2row(trajectory.position, 2, 1), mat2row(trajectory.position, 3, 1))
    % plot3(mat2row(state.position, 1, 1), mat2row(state.position, 2, 1), mat2row(state.position, 3, 1))
end
