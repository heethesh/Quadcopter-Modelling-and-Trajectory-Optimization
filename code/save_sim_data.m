function [outputs] = save_sim_data(all_state, all_traj, actual_time, sim_dt)

    for i = 1:5
        cur_state = all_state{i};
        cur_traj = all_traj{i};

        if i == 1
            state.position = cur_state.position;
            state.angle = cur_state.angle;
            trajectory.position = cur_traj.position;
            trajectory.angle = cur_traj.angle;
        else
            state.position = cat(3, state.position, cur_state.position);
            state.angle = cat(3, state.angle, cur_state.angle);
            trajectory.position = cat(3, trajectory.position, cur_traj.position);
            trajectory.angle = cat(3, trajectory.angle, cur_traj.angle);
        end
    end

    disp([newline + "State Size: " + num2str(size(state.position))]);
    disp(["Trajectory Size: " + num2str(size(trajectory.position))]);
    disp(["Actual Time: " + num2str(actual_time)]);
    disp(["Total Time: " + num2str(sum(actual_time))]);
    disp(["Total Samples: " + num2str(sum(actual_time) / sim_dt)]);

    csvwrite('../data/sim_params.csv', [int32(sum(actual_time) / sim_dt)]);
    csvwrite('../data/state_position.csv', state.position);
    csvwrite('../data/state_angle.csv', state.angle);
    csvwrite('../data/trajectory_position.csv', trajectory.position);
    csvwrite('../data/trajectory_angle.csv', trajectory.angle);
    disp([newline + "Simulation data saved"]);
end
