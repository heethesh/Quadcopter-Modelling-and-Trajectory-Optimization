function coef = trajectory_optimizer(A, B, time_scale, vel_update)
    % Optimization problem parameters
    syms c1 c2 c3 c4 t;
    x = c1 + c2*t + c3*t^2 + c4*t^3;
    dx = diff(x, t);
    dxx = diff(dx, t);
    costf = int(dxx^2, t, 0, time_scale);
    H = eval(hessian(costf, [c1, c2, c3, c4]));
    f = zeros(1, 4);

    % Return co-efficients and end velocity components
    coef = zeros(4);
    vel = zeros(1, 4);

    % Quadratic programming with velocity update
    fields = fieldnames(B);
    for i = 1:numel(fields)
        % Extract field name
        fname = fields{i}(1:end-1);
        B_ = B.([fname num2str(i)]); Bn = size(B_);
        
        % QP minimization
        [coef(i, :), fval, exitflag, output, lambda] = ...
        quadprog(H, f, [], [], A(1:Bn(1), :), B_, [], [], [], struct('Display', 'off'));
        
        % Calculate velocity and update for next iteration
        vel(i) = eval(subs(dx, [c1 c2 c3 c4 t], [coef(i, :) time_scale]));
        if (i < 4 && vel_update(i+1) ~= 0) B.([fname num2str(i+1)])(vel_update(i+1)) = vel(i); end
    end
    % disp(vel);
end
