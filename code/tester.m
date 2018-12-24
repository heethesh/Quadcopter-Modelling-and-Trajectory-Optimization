clear; clc;

MIN_POS = 0; MIN_VEL = 1; MIN_ACC = 2;
MIN_JRK = 3; MIN_SNP = 4; MIN_CRK = 5;

t0 = 0;
tn = 1;

% Trajectory --> X | Y
pos = [0, 0;
       2, 1;
       0, 2;
      -2, 1;
       0, 0]

vel = [0, 0;
       NaN, NaN;
       NaN, NaN;
       NaN, NaN;

       1, 0;
       0, 1;
       -1, 0;
       0, -1;
       
       1, 0;
       0, NaN;
       NaN, 0;
       0, NaN;
       
       0, 0]

coef = cell(1, 6);
vel_act = cell(1, 6);
m = 1; v_off = 0;
last_vel = [NaN, NaN];

% Constraints --> Order | Time (@t) | Value
for i = 1:3
    for j = 1:2
        constraints = cell(1, 4);
        for k = 1:4
            constraints{k} = [1, t0, pos(k, j); 1, tn, pos(k+1, j); ...
                2, t0, vel(k+v_off, j); 2, tn, vel(k+v_off+1, j)];
            if isnan(vel(k+1, j)) constraints{k}(end, :) = []; end
        end
        [coef{m}, vel_act{m}] = trajectory_generator(MIN_ACC, t0, tn, constraints, last_vel(j));
        last_vel(j) = vel_act{m}(end);
        vel_act{m}
        m = m + 1;
    end
    v_off = v_off + 4;
end

% Plot trajectories
samples = tn/0.005;
t = linspace(t0, tn, samples);

p = @(c, t) polyval(c, t);
dp = @(c, t) polyval(polyder(c), t);

trajx = zeros(12, samples);
trajy = zeros(12, samples);
velx = zeros(12, samples);
vely = zeros(12, samples);

% hold on; grid on;
for i = 3:2:4
    for j = 1:4
        trajx(j, :) = p(coef{i}(j, :), t);
        trajy(j+1, :) = p(coef{i+1}(j, :), t);
        
        velx(j, :) = dp(coef{i}(j, :), t);
        vely(j+1, :) = dp(coef{i+1}(j, :), t);
        
        subplot(1, 3, 1); hold on; grid on; plot(trajx(j, :), trajy(j+1, :));
        subplot(1, 3, 2); hold on; grid on; plot(t, velx(j, :));
        subplot(1, 3, 3); hold on; grid on; plot(t, vely(j+1, :));
    end
end
