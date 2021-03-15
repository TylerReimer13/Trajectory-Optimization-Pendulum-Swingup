import numpy as np
import matplotlib.pyplot as plt
import scipy.optimize as opt
from math import sin, cos
from pendulum_swingup_animation import animated_plot


def motion_model(states, u):
    """
    Simulates the single time-step pendulum response to an input force
    """
    x1, x1_dot, x2, x2_dot = states

    x1_ddot_num = l*m2*sin(x2)*(x2_dot**2) + u + m2*g*cos(x2)*sin(x2)
    x1_ddot_den = m1 + m2*(1-(cos(x2)**2))
    x1_ddot = x1_ddot_num/x1_ddot_den

    x2_ddot_num = l*m2*cos(x2)*sin(x2)*(x2_dot**2) + u*cos(x2) + (m1+m2)*g*sin(x2)
    x2_ddot_den = l*m1 + l*m2*(1-(cos(x2))**2)
    x2_ddot = -(x2_ddot_num/x2_ddot_den)

    x1_dot += x1_ddot * hk
    x1 += x1_dot * hk

    x2_dot += x2_ddot * hk
    x2 += x2_dot * hk

    return np.array([x1_ddot, x2_ddot]), np.array([x1, x1_dot, x2, x2_dot])


def cost_func(u):
    cost = 0.
    for k in range(N-1):
        cost += (hk/2) * ((u[k]**2) + (u[k+1]**2))
    return cost


def x_bounds(x):
    i = len(u_est)
    xi_const = x[i] - init_state[0]
    vi_const = x[i + 1] - init_state[1]
    ti_const = x[i + 2] - init_state[2]
    wi_const = x[i + 3] - init_state[3]

    xf_const = x[-1 - 3] - final_state[0]
    vf_const = x[-1 - 2] - final_state[1]
    tf_const = x[-1 - 1] - final_state[2]
    wf_const = x[-1] - final_state[3]

    return [xi_const, vi_const, ti_const, wi_const, xf_const, vf_const, tf_const, wf_const]


def dynamic_cons(x, k=0):
    """
    Uses trapezoidal collocation to rewrite dynamic constraints in a form that works with the solver
    x = cart pos, v = cart vel, t = pend. angle, w = pend. angular vel
    """

    xk_idx = NUM_STATES*k + len(u_est)
    xk = x[xk_idx]
    xk_1 = x[xk_idx + NUM_STATES]

    vk_idx = xk_idx + 1
    vk = x[vk_idx]
    vk_1 = x[vk_idx + NUM_STATES]

    tk_idx = vk_idx + 1
    tk = x[tk_idx]
    tk_1 = x[tk_idx + NUM_STATES]

    wk_idx = tk_idx + 1
    wk = x[wk_idx]
    wk_1 = x[wk_idx + NUM_STATES]

    uk = x[k]
    uk_1 = x[k+1]

    k_accels, _ = motion_model(np.array([xk, vk, tk, wk]), uk)
    k1_accels, _ = motion_model(np.array([xk_1, vk_1, tk_1, wk_1]), uk_1)

    x_const = (.5 * hk) * (vk_1 + vk) + -1 * (xk_1 - xk)
    v_const = (.5 * hk) * (k1_accels[0] + k_accels[0]) + -1 * (vk_1 - vk)
    t_const = (.5 * hk) * (wk_1 + wk) + -1 * (tk_1 - tk)
    w_const = (.5 * hk) * (k1_accels[1] + k_accels[1]) + -1 * (wk_1 - wk)

    return [x_const, v_const, t_const, w_const]


if __name__ == "__main__":
    hk = .025  # tk_1-tk
    
    l = .5
    g = 9.81
    m1 = 1.
    m2 = .3
    
    N = 75  # Num collocation points
    NUM_STATES = 4

    dmin = -2.
    dmax = 2.

    umin = -20.
    umax = 20.

    # [0] = cart pos, [1] = cart vel, [2] = pendulum angle, [3] = pendulum angular rate
    init_state = np.array([0., 0., 0., 0.])
    final_state = np.array([2., 0., np.pi, 0.])

    u_est = np.zeros(N)
    x_est = np.ones((NUM_STATES*N))
    est = np.hstack((u_est, x_est))

    u_bds = [(umin, umax)] * N
    x_bds = [(dmin, dmax), (None, None), (None, None), (None, None)] * N
    bds = u_bds + x_bds

    cons = []
    xi_constraint = {'type': 'eq', 'fun': x_bounds}
    cons.append(xi_constraint)

    for idx in range(N-1):
        constraint = {'type': 'eq', 'fun': dynamic_cons, 'args': [idx]}
        cons.append(constraint)

    res = opt.minimize(cost_func, est, bounds=bds, constraints=cons)
    results = res.x
    u_opt = results[:N]

    states = np.reshape(results[N:], (-1, NUM_STATES))
    print('Optimal Inputs')
    print(u_opt)
    print('----------------------------')
    print('System States')
    print(states)

    plt.plot([n for n in range(N)], u_opt, 'bX', label='Optimal Inputs')
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot([n*hk for n in range(N)], states[:, 0], label='Cart Pos')
    plt.plot([n*hk for n in range(N)], states[:, 1], label='Cart Vel')
    plt.plot([n*hk for n in range(N)], states[:, 2], label='Pendulum Angle')
    plt.plot([n*hk for n in range(N)], states[:, 3], label='Pendulum Angular Vel')
    plt.legend()
    plt.xlabel('Time (sec.)')
    plt.ylabel('States')
    plt.grid()
    plt.show()

    animated_plot(states[:, 0].flatten(), states[:, 2].flatten(), hk, l, N)
