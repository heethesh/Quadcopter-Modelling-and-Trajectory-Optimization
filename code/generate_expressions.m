function ret = generate_expressions()
%% Desired angular velocity (symbolic)
syms exx(t) eyy(t) xx_des(t) yy_des(t) psid(t) g;
error_matrix = [(exx + xx_des)*diff(psid, t) - diff(yy_des, t);
                (eyy + yy_des)*diff(psid, t) + diff(xx_des, t)];
dw_des_sym = (1/g).*[cos(psid) sin(psid); -sin(psid) cos(psid)] * error_matrix
dw_des_sym = simplify(dw_des_sym);

% Desired angular velocity (symbolic substitution)
syms exx_ eyy_ xx_des_ yy_des_ xxx_des_ yyy_des_ psid_ psid_dot_;
dw_des = subs(dw_des_sym, [exx eyy xx_des yy_des diff(xx_des, t) diff(yy_des, t) psid diff(psid, t)], ...
    [exx_ eyy_ xx_des_ yy_des_ xxx_des_ yyy_des_ psid_ psid_dot_])

%% Desired angular acceleration (symbolic)
dww_des_sym = simplify(diff(dw_des_sym));

% Desired angular acceleration (symbolic substitution)
syms exx_ eyy_ exxx_ eyyy_;
syms xx_des_ yy_des_ xxx_des_ yyy_des_ xxxx_des_ yyyy_des_;
syms psid_ psidd_ psiddd_;
dww_des = subs(dww_des_sym, [exx eyy diff(exx, t) diff(eyy, t) ...
    xx_des yy_des diff(xx_des, t) diff(yy_des, t) diff(xx_des, t, t) diff(yy_des, t, t) ...
    psid diff(psid, t) diff(psid, t, t)], ...
    [exx_ eyy_ exxx_ eyyy_ ...
    xx_des_ yy_des_ xxx_des_ yyy_des_ xxxx_des_ yyyy_des_ ...
    psid_ psidd_ psiddd_])

%% Save to file
matlabFunction(dw_des, 'File', 'angular_velocity', 'vars', ...
    {exx_ eyy_ xx_des_ yy_des_ xxx_des_ yyy_des_ psid_ psid_dot_ g t});
matlabFunction(dww_des, 'File', 'angular_acceleration', 'vars', ...
    {exx_ eyy_ exxx_ eyyy_ ...
    xx_des_ yy_des_ xxx_des_ yyy_des_ xxxx_des_ yyyy_des_ ...
    psid_ psidd_ psiddd_ g t});

end