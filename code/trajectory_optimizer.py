import scipy.io
import numpy as np
import sympy as sp
from numpy import nan
from sympy.abc import x, t
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers
import matplotlib.animation as anim
import mpl_toolkits.mplot3d.axes3d as p3

min_order = {'POS': 1, 'VEL': 1, 'ACC': 2, 'JRK': 3, 'SNP': 4}
poly_order = {'POS': 1, 'VEL': 1, 'ACC': 3, 'JRK': 5, 'SNP': 7}
dt = 0.005
eps = 1e-6

def sym2cvx(M):
    return matrix(np.asarray(M).astype(np.float64))

def generate_eqn(coef_con, coef_sym):
    return np.dot([co*t**i for i, co in enumerate(reversed(coef_con))], coef_sym)

def generate_constraints(path, npath, ti, tf, waypoint, axis, update, fval):
    # print('\nLast values', fval)
    constraints = []
    for i, p in enumerate(path[:npath]):
        val_int = p[waypoint, axis] 
        if (i in update) and (i in fval):
            if np.isnan(p[waypoint, axis]):
                # print('>>>\033[93m REPLACEMENT HERE \\/\033[0m')
                val_int = fval[i]
            elif abs(fval[i] - p[waypoint, axis]) > eps: 
                print('\n>>>\033[91m WARNING: DISCONTINUITY\033[0m -->', fval[i], p[waypoint, axis])

        # print('\nInitial value for order: %d, value:'%i, val_int)
        constraints.append([i, ti, val_int])
        
        # print('Final value for order: %d, value:'%i, p[waypoint+1, axis])
        if not np.isnan(p[waypoint+1, axis]):
            constraints.append([i, tf, p[waypoint+1, axis]])
        # else: print('>>>\033[92m CONSTRAINT IGNORED ^\033[0m')

    # print('Total constraints:', len(constraints))
    return constraints

def quadprog(minimize, ti, tf, constraints, update=[], neq_constraints=None):
    # Order of cost function and output polynomial
    order = poly_order[minimize]
    morder = min_order[minimize]
    ncoef = order + 1

    # Generate symbolic equations and derivatives
    coef_con = np.ones(ncoef)
    coef_sym = np.asarray(sp.symbols('a0:%d'%(ncoef)))
    sym_eqn = [generate_eqn(coef_con, coef_sym)]

    for i in range(1, order):
        coef_con = np.polyder(coef_con)
        sym_eqn.append(generate_eqn(coef_con, coef_sym[i:]))

    # Generate QP parameters
    cost_fun = sp.integrate(sym_eqn[morder]**2, (t, ti, tf))
    H = sym2cvx(sp.hessian(cost_fun, coef_sym))
    f = sym2cvx(np.ones(ncoef))

    # Generate QP constraints
    Ceq = [ sym_eqn[pord].subs(t, tn) - bval for pord, tn, bval in constraints ]
    Aeq, Beq = sp.linear_eq_to_matrix(Ceq, coef_sym)
    if neq_constraints is not None: 
        Cneq = [ sym_eqn[pord].subs(t, tn) - bval for pord, tn, bval in neq_constraints ]
        Aneq, Bneq = sp.linear_eq_to_matrix(Cneq, coef_sym)
    
    # Solve QP
    if neq_constraints is not None: 
        sol = solvers.qp(H, f, sym2cvx(Aneq), sym2cvx(Bneq), sym2cvx(Aeq), sym2cvx(Beq))
    else: sol = solvers.qp(H, f, A=sym2cvx(Aeq), b=sym2cvx(Beq))
    
    # Ignore very small coefficients
    sol_coefs = np.asarray(sol['x']).flatten()
    sol_coefs[abs(sol_coefs) < eps] = 0

    # Evaluate the final values for next continuity in next trajectory
    subs = { sym: num for sym, num in zip(coef_sym, sol_coefs) }; subs[t] = tf
    feval = { i: float(sym_eqn[i].subs(subs)) for i in update }
 
    return sol_coefs, feval

def generate_trajectory(poly_coefs, ti, tf, n, savename=None):
    samples = int(((tf - ti)/dt) + 1)
    time = np.linspace(ti, tf, samples)

    # Generate trajectories
    xval = []; yval = []; zval = []; wval = []
    for i in range(n):
        xval.append(np.polyval(np.flip(poly_coefs[0][i]), time))
        yval.append(np.polyval(np.flip(poly_coefs[1][i]), time))
        zval.append(np.polyval(np.flip(poly_coefs[2][i]), time))
        wval.append(np.polyval(np.flip(poly_coefs[3][i]), time))

    xval = np.asarray(xval).flatten()
    yval = np.asarray(yval).flatten()
    zval = np.asarray(zval).flatten()
    wval = np.asarray(wval).flatten()    

    # Save mat files
    if savename:
        print('\nTotal samples:', xval.shape)
        scipy.io.savemat(savename, dict(x=xval, y=yval, z=zval, w=wval))

    return xval, yval, zval, wval
