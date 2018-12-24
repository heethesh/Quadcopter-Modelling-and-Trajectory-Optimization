function [pos_act, angle_act] = dynamic_model(params, state, n, W_act, dt)
%% Calculate forces
W_act_sq = W_act.^2;
forces = mixing_matrix(params) * W_act_sq;

%% Linear states
R = eul2rotm(flipud(state.angle(:, 1, n))');
a_act = ((1/params.mass)*R*[0; 0; forces(1)]) - [0; 0; params.gravity];

% Find linear velocity
[t, dx_act] = ode45(@(t, dx_act) a_act, dt, state.position(:, 2, n));
dx_act = dx_act(end, :)';

% Find linear position
[t, x_act] = ode45(@(t, x_act) dx_act, dt, state.position(:, 1, n));
x_act = x_act(end, :)';

%% Calculate angular states
ddw_act = params.inertia\forces(2:4);

% Find angular velocity
[t, dw_act] = ode45(@(t, dw_act) ddw_act, dt, state.angle(:, 2, n));
dw_act = dw_act(end, :)';

% Find angular position
[t, w_act] = ode45(@(t, w_act) dw_act, dt, state.angle(:, 1, n));
w_act = w_act(end, :)';

% Angular velocity to RPY transformation for aggressive model only
T = eul2rotm([0 flipud(state.angle(1:2, 1, n))']);
w_act = T * w_act;

%% Return states
pos_act = [x_act dx_act];
angle_act = [w_act dw_act];

end