from trajectory_optimizer import *

def plotter():
    samples = int(((tf - ti)/dt) + 1)
    time = np.linspace(ti, tf, samples)
    ftime = np.linspace(ti, tf*20, samples*20)

    # 2D path plot
    fig1, ax1 = plt.subplots()
    plt.grid()
    ax1.plot(xval, yval)
    plt.show(block=True)
    
    # 3D animation path plot
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot([0, 0], [0, 0], [0, 1], c='r')
    plt.subplots_adjust(left=0, bottom=0, right=1, top=1)
    plt.grid()
    plt.ion()
    
    last = 0
    skip = 100
    phase = -1
    cmap = plt.cm.get_cmap('tab20', 50)

    for i in range(0, samples*ntraj):
        if i % ((samples*ntraj)//nphase)==0: phase = np.random.randint(50)
        if i % skip==0:
            ax.plot(xval[last:i], yval[last:i], zval[last:i], c=cmap(phase))
            last = i
            plt.pause(0.001)
    plt.show(block=True)

    # Yaw vs time plot
    fig2, ax2 = plt.subplots()
    plt.grid()
    ax2.plot(ftime, wval)
    plt.show(block=True)

if __name__ == '__main__':
    # Set path and constraints here
    v = 1
    pos = np.asarray([[0, 0, 1, 0], [2, 1, 1, 0], [0, 2, 1, 0], [-2, 1, 1, 0], [0, 0, 1, 0]])
    vel1 = np.asarray([[0, 0, 0, 0], [nan, nan, nan, nan], [nan, nan, nan, nan], [nan, nan, nan, nan], [v, 0, nan, nan]])
    vel2 = np.asarray([[v, 0, nan, nan], [0, v, nan, nan], [-v, 0, nan, nan], [0, -v, nan, nan], [v, 0, nan, nan]])
    vel3 = np.flipud(vel1)
    acc = np.asarray([[0, 0, 0, 0]]*5)
    jrk = np.asarray([[0, 0, 0, 0]]*5)
    
    loops = 3
    vel_all = [vel1] + [vel2]*loops + [vel3]

    # Set parameters
    ti = 0; tf = 2
    naxes = 4
    ntraj = 8 + (4*loops)
    npath = [4, 4, 2, 2]
    nphase = loops + 2
    minimize = ['SNP', 'SNP', 'ACC', 'ACC']
    update = [[1, 2, 3, 4], [1, 2, 3, 4], [1, 2], [1, 2]]
    
    # Dicts to store data
    fval = { i: {} for i in range(naxes) }
    poly_coefs = { i: [] for i in range(naxes) }
    final_values = { i: [] for i in range(naxes) }

    # Optimise all paths
    for ph in range(nphase):
        if ph in [0, 4]: path = [pos, vel_all[ph], acc, jrk]
        else: path = [pos, vel_all[ph]]
        
        for ax in range(naxes):
            for wp in range(len(pos)-1):
                # print('\n---------- Phase %d | Waypoint %d | Axis %d ----------'%(ph+1, wp+1, ax))
                constraints = generate_constraints(path, npath[ax], ti, tf, wp, ax, update[ax], fval[ax])
                coef, fval[ax] = quadprog(minimize[ax], ti, tf, constraints, update[ax])
                poly_coefs[ax].append(coef)
                final_values[ax].append(fval[ax])

    # Verify constraints
    print('\nFinal Velocities:')
    for ph in range(nphase):
        vel = vel_all[ph]
        for i in range(0, ntraj//nphase):
            xvel = 0.0 if abs(final_values[0][i+ph*4][1]) < eps else final_values[0][i+ph*4][1]
            yvel = 0.0 if abs(final_values[1][i+ph*4][1]) < eps else final_values[1][i+ph*4][1]
            zvel = 0.0 if abs(final_values[2][i+ph*4][1]) < eps else final_values[2][i+ph*4][1]
            wvel = 0.0 if abs(final_values[3][i+ph*4][1]) < eps else final_values[3][i+ph*4][1]
            
            xvel_req = vel[i+1, 0]
            yvel_req = vel[i+1, 1]
            zvel_req = vel[i+1, 2]
            wvel_req = vel[i+1, 3]

            print('Xdes: %.3f \tXact: %.3f \t| Ydes: %.3f \tYact: %.3f'%(xvel_req, xvel, yvel_req, yvel))
            print('Zdes: %.3f \tZact: %.3f \t| Wdes: %.3f \tWact: %.3f\n'%(zvel_req, zvel, wvel_req, wvel))

    # Plot and save trajectory
    xval, yval, zval, wval = generate_trajectory(poly_coefs, ti, tf, 
        ntraj, savename='../data/Q8-POS-SSAA-T1_5-L3-V3.mat')
    plotter()
