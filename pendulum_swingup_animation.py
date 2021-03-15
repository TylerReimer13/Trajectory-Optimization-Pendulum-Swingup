import matplotlib.pyplot as plt
from math import sin, cos, pi
import matplotlib.animation as animation


def animated_plot(cart_pos, pend_ang, dt, l, nsim):
    fig = plt.figure()
    ax = plt.axes(xlim=(-5., 5.), ylim=(-.5, 2.))
    time_text = ax.text(2.75, 1.75, '')
    angle_text = ax.text(2.75, 1.65, '')
    pos_text = ax.text(2.75, 1.55, '')
    times = range(nsim)

    # writer = animation.writers['ffmpeg']
    # writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

    def update(i):
        time_text.set_text("Time: {0:0}".format(round(times[i]*dt, 2)))
        angle_text.set_text("Angle: {0:0.2f}".format((pend_ang[i])*57.3))
        pos_text.set_text("Pos: {0:0.2f}".format(cart_pos[i]))

        cart_x = cart_pos[i]
        pend_pos = -pend_ang[i] - pi
        pendulum_x0 = cart_x
        pendulum_y0 = 0.
        pendulum_x1 = cart_x + l*sin(pend_pos)
        pendulum_y1 = l*cos(pend_pos)

        cart.set_data([cart_x-.25, cart_x+.25], 0.)
        pend.set_data([pendulum_x0, pendulum_x1], [pendulum_y0, pendulum_y1])
        ball.set_data(pendulum_x1, pendulum_y1)
        return (cart,) + (pend,) + (ball,)

    cart, = plt.plot([], [], 'black', linewidth=5.)
    pend, = plt.plot([], [], 'b-')
    ball, = plt.plot([], [], 'ro', markersize=8.)
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    ani = animation.FuncAnimation(fig, update, int(nsim), interval=40, repeat=True)
    ani.save("movie.gif")
    plt.show()

