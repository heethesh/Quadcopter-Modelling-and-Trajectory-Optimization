clear; clc;

syms a0 a1 a2 a3 t k1 k2 k;
x = a0 + a1*t + a2*t^2 + a3*t^3;
dx = a1 + 2*a2*t + 3*a3*t^2;

% c = k1 * exp(-k2 * t);
% c = k * t * exp(t/10);
% c = k * exp(-t);
% c = k / (1 + exp(-t));
c = k * t;

scaled_dx = x * c;
scaled_x = int(scaled_dx, t);

oldT = 1;
newT = 10;

pos_x0 = 0;
pos_xf = 2;

vel_x0 = 0;
vel_xf = 0;

eval_sx = eval(subs(scaled_x, [t a0 a1 a2 a3], [newT 0 0 6 -4]));
eval_sdx = eval(subs(scaled_dx, [t a0 a1 a2 a3], [newT 0 0 6 -4]));
eval_kx = eval(solve(eval_sx==pos_xf, k))
eval_kdx = eval(solve(eval_sdx==vel_xf, k))

% Test back
eval(subs(scaled_x, [t a0 a1 a2 a3 k], [0 0 0 6 -4 eval_kx]))
eval(subs(scaled_x, [t a0 a1 a2 a3 k], [newT 0 0 6 -4 eval_kx]))
eval(subs(scaled_dx, [t a0 a1 a2 a3 k], [0 0 0 6 -4 eval_kdx]))
eval(subs(scaled_dx, [t a0 a1 a2 a3 k], [newT 0 0 6 -4 eval_kdx]))

otime = linspace(0, oldT);
ntime = linspace(0, newT);

% Position
es_xt = eval(subs(scaled_x, {t a0 a1 a2 a3 k}, {ntime 0 0 6 -4 eval_kx}));
o_xt = eval(subs(x, {t a0 a1 a2 a3}, {otime 0 0 6 -4}));

subplot(1,2,1);
hold on; grid on;
plot(ntime, es_xt);
plot(otime, o_xt);

% Velocity
es_dxt = eval(subs(scaled_dx, {t a0 a1 a2 a3 k}, {ntime 0 0 6 -4 eval_kdx}));
o_dxt = eval(subs(dx, {t a0 a1 a2 a3}, {otime 0 0 6 -4}));

subplot(1,2,2);
hold on; grid on;
plot(ntime, es_dxt);
plot(otime, o_dxt);


% eval_sdx = ek = eval(solve(esx==2, k))
