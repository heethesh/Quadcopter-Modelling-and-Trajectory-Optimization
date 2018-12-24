close all;
clear;
clc;

%% True plant parameters
plant_params = struct(...
    'mass',                   0.770, ...
    'gravity',                9.80665, ...
    'arm_length',             0.1103, ...
    'motor_spread_angle',     0.925, ...
    'thrust_coefficient',     8.07e-9, ...
    'moment_scale',           0.017, ...
    'motor_constant',         36.5, ...
    'rpm_min',                3000, ...
    'rpm_max',                20000, ...
    'inertia',                diag([0.0033 0.0033 0.005]),...
    'zoffset',                0.2);

%% Create reference trajectory
% waypoints are [x y z yaw]
wpvec = [ 0  0 0.2 0;
          0  0 0.8 0;
         -0.5 0.5 1.0 0;
         -1  1 1.0 pi/2;
         1.5 1 1.0 pi;
         1.5 -1 0.5 pi];

wptimes = [0 2 5 10 15 20];
     
spline_order = 5;
deriv_to_minimize = 3;     
show_plots = false;

sfvec = make_trajectory(wpvec,wptimes,spline_order,deriv_to_minimize,show_plots);

%% Controller Parameters
lookahead_time = 0.01;

ctrl = struct();
ctrl.Kp = [17 17 20];
ctrl.Kv = [6.6 6.6 9];
ctrl.Kr = [190 198 80];
ctrl.Kw = [30 30 17.88];
ctrl.inertia = plant_params.inertia;
ctrl.ct = plant_params.thrust_coefficient;
ctrl.ms = plant_params.moment_scale;

%% Simulation parameters
sim_start_time = 0;
sim_dt = 0.005;
sim_end_time = 20.0;

%% Set initial conditions
init_pos = [0; 0; plant_params.zoffset];
init_vel = zeros(3,1);
init_rpy = zeros(3,1);
init_avl = zeros(3,1);
init_rpm = plant_params.rpm_min*ones(4,1);

%% Initialize containers for simulation variables
tvec = sim_start_time:sim_dt:sim_end_time;

N = numel(tvec);
posvec = zeros(3,N);
velvec = zeros(3,N);
accvec = zeros(3,N);
rpyvec = zeros(3,N);
avlvec = zeros(3,N);
rpmvec = zeros(4,N);

posvec(:,1) = init_pos;
velvec(:,1) = init_vel;
rpyvec(:,1) = init_rpy;
avlvec(:,1) = init_avl;
rpmvec(:,1) = init_rpm;

posdes = zeros(3,N);
veldes = zeros(3,N);
rpydes = nan(3,N);
accdes = zeros(3,N);
avldes = zeros(3,N);
rpmdes = zeros(4,N);

%% Run simulation
for kk = 2:N
    % Get position and yaw reference and derivatives
    tquery = tvec(kk) + lookahead_time;
    posyawdes = zeros(4,3);
    
    for dd = 1:4
        for pp = 1:3
            posyawdes(dd,pp) = sfvec{dd}.evalAtTime(tquery,pp-1); 
        end
    end
    
    posdes(:,kk) = posyawdes(1:3,1);
    veldes(:,kk) = posyawdes(1:3,2);
    accdes(:,kk) = posyawdes(1:3,3);
    rpydes(3,kk) = posyawdes(4,1);
    
    % Run controller
    rpmdes(:,kk) = controller(posvec(:,kk-1),velvec(:,kk-1),rpyvec(:,kk-1),...
        avlvec(:,kk-1),posdes(:,kk),veldes(:,kk),accdes(:,kk),rpydes(3,kk),...
        plant_params,ctrl);
                                          
    % Run plant model
    dt = tvec(kk) - tvec(kk-1);
    
    % Get state derivatives
    [dpos,dvel,drpy,davl,drpm] = quadrotor_model(posvec(:,kk-1),...
        velvec(:,kk-1),rpyvec(:,kk-1),avlvec(:,kk-1),rpmvec(:,kk-1),...
        rpmdes(:,kk), plant_params);
    
    % Apply Euler integration to update simulation state variables
    posvec(:,kk) = posvec(:,kk-1) + dt * dpos;
    velvec(:,kk) = velvec(:,kk-1) + dt * dvel;
    rpyvec(:,kk) = rpyvec(:,kk-1) + dt * drpy;
    avlvec(:,kk) = avlvec(:,kk-1) + dt * davl;
    accvec(:,kk) = dvel;
    rpmtmp = rpmvec(:,kk-1) + dt * drpm;
    
    % Saturate RPM
    rpmtmp(rpmtmp > plant_params.rpm_max) = plant_params.rpm_max;
    rpmtmp(rpmtmp < plant_params.rpm_min) = plant_params.rpm_min;
    
    rpmvec(:,kk) = rpmtmp;
end

%% Plot desired vs. actual motion
opt = struct('ylabels',{{'x [m]','y [m]','z [m]'}});
opt.title = 'Desired vs. Actual Position';
opt.legend = {'Desired','Actual'};
plotvecntimeseries(tvec,posdes,tvec,posvec,opt);

opt = struct('ylabels',{{'v_x [m/s]','v_y [m/s]','v_z [m/s]'}});
opt.title = 'Desired vs. Actual Velocity';
opt.legend = {'Desired','Actual'};
plotvecntimeseries(tvec,veldes,tvec,velvec,opt);

opt = struct('ylabels',{{'roll [rad]','pitch [rad]','yaw [rad]'}});
opt.title = 'Desired vs. Actual Attitude';
opt.legend = {'Desired','Actual'};
plotvecntimeseries(tvec,rpydes,tvec,rpyvec,opt);

opt = struct('ylabels',{{'a_x [m/s^2]','a_y [m/s^2]','a_z [m/s^2]'}});
opt.title = 'Desired vs. Actual Acceleration';
opt.legend = {'Desired','Actual'};
plotvecntimeseries(tvec,accdes,tvec,accvec,opt);

opt = struct('ylabels',{{'\omega_x [rad/s]','\omega_y [rad/s]','\omega_z [rad/s]'}});
opt.title = 'Desired vs. Actual Angular Velocity';
opt.legend = {'Desired','Actual'};
plotvecntimeseries(tvec,avldes,tvec,avlvec,opt);

opt = struct('ylabels',{{'\varpi_1','\varpi_2','\varpi_3','\varpi_4'}});
opt.title = 'RPM';
opt.legend = {'Desired','Actual'};
plotvecntimeseries(tvec, rpmdes, tvec, rpmvec, opt);