function [pos, vel, acc] = bounded_acceleration(acc_des)
    % Initial time range
    t0 = 0;
    tn = 1;

    % Polynomial and cost function order
    ord = 7;
    mord = 4;

    % Line search ptimization parameters
    niters = 0;
    max_iters = 100;
    time_step = 0.05;

    % Outer loop optimization
    while (niters < max_iters)
        niters = niters + 1;

        % Constraints
        constraints = [[1, t0, 1]; [1, tn, 10];
                       [2, t0, 0]; [2, tn, 0];
                       [3, t0, 0]; [3, tn, 0];
                       [4, t0, 0]; [4, tn, 0]];

        syms t;
        c = fliplr(sym('c', [1 ord+1]));

        % Symbolic equations and derivatives
        sym_eqs = cell(1, mord+1);
        sym_eqs{1} = poly2sym(c .* ones(1, ord+1), t);
        for i = 2:mord+1 sym_eqs{i} = diff(sym_eqs{i-1}, t); end

        % QP parameters
        costf = int(sym_eqs{mord+1}^2, t, t0, tn);
        H = eval(hessian(costf, c));
        f = zeros(1, ord+1);

        % QP constraints
        csize = size(constraints);
        Ceq = sym('Ceq', [1 csize(1)]);
        for j = 1:csize(1)
            Ceq(j) = subs(sym_eqs{constraints(j, 1)}, ...
                t, constraints(j, 2))==constraints(j, 3);
        end

        % Quadratic programming
        [Aeq, Beq] = equationsToMatrix(Ceq);
        [coefs, fval, exitflag, output, lambda] = ...
            quadprog(H, f, [], [], eval(Aeq), eval(Beq), [], [], [], struct('Display', 'off'));
        fcoef = fliplr(coefs');

        % Evaluate acceleration with new coefficients
        samples = tn/0.005;
        time = linspace(t0, tn, samples);
        acc = polyval(polyder(polyder(fcoef)), time);
        acc_act = max(abs(acc));

        % Check if outer loop constraints met
        if acc_act <= acc_des break;
        else tn = tn + time_step; end
    end

    % Display optimization results
    disp(['Maximum acceleration: ' num2str(acc_act)]);
    disp(['Time range: ' num2str(tn)]);
    disp(['Coefficients: ' num2str(fcoef)]);
    disp(['Iterations: ' num2str(niters)]);

    % Plot graphs
    pos = polyval(fcoef, time);
    vel = polyval(polyder(fcoef), time);

    % hold on; grid on;
    % title("Bounded Acceleration Trajectory");
    % plot(time, pos);
    % plot(time, vel);
    % plot(time, acc);
    % legend({"Position", "Velocity", "Acceleration"});
    % xlabel("Time (s)");
    % ylabel("Position / Velocity / Acceleration");
end