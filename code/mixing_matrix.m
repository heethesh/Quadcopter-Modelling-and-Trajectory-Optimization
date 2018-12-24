function ret = mixing_matrix(params)
ct = params.thrust_coefficient;
cq = params.thrust_coefficient * params.moment_scale;
d = params.arm_length;
ret = [ct ct ct ct; 0 d*ct 0 -d*ct; -d*ct 0 d*ct 0; -cq cq -cq cq];
end