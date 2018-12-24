function [coef, vel] = trajectory_generator(mord, t0, tn, constraints, last_vel)
    switch mord
        case 0
        case 1 
            ord = 1;
        case 2 
            ord = 3;
        case 3 
            ord = 5;
        case 4 
            ord = 7;
        case 5 
            ord = 9;
        otherwise
            disp('Order not supported');
            return;
    end

    syms t;
    c = fliplr(sym('c', [1 ord+1]));

    sym_eqs = cell(1, mord+1);
    sym_eqs{1} = poly2sym(c .* ones(1, ord+1), t);
    for i = 2:mord+1 sym_eqs{i} = diff(sym_eqs{i-1}, t); end

    costf = int(sym_eqs{mord+1}^2, t, t0, tn);
    H = eval(hessian(costf, c));
    f = zeros(1, ord+1);
    
    % Return co-efficients and end velocity components
    nc = numel(constraints);
    coef = zeros(nc, ord+1);
    vel = zeros(1, nc);

    % Quadratic programming with velocity update
    for i = 1:nc
        if i == 1 
            if ~isnan(last_vel)
                % last_vel
                constraints{i}(isnan(constraints{i})) = last_vel; 
            end
        else
            constraints{i}(isnan(constraints{i})) = vel(i-1); 
        end

        csize = size(constraints{i});
        Ceq = sym('Ceq', [1 csize(1)]);

        for j = 1:csize(1)
            Ceq(j) = subs(sym_eqs{constraints{i}(j, 1)}, ...
                t, constraints{i}(j, 2))==constraints{i}(j, 3);
        end
        [Aeq, Beq] = equationsToMatrix(Ceq);
        [coefs, fval, exitflag, output, lambda] = ...
            quadprog(H, f, [], [], eval(Aeq), eval(Beq), [], [], [], struct('Display', 'off'));
        coef(i, :) = fliplr(coefs');
        
        % Calculate velocity and update for next iteration
        vel(i) = eval(subs(sym_eqs{2}, [c t], [coef(i, :) tn]));
    end
end
