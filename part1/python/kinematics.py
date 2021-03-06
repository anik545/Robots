from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import collections
import matplotlib
import matplotlib.pylab as plt
import numpy as np
import time


# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Drawing constants.
REFRESH_RATE = 1. / 15.


def euler(current_pose, t, dt):
    next_pose = current_pose.copy()
    u = 0.25
    w = np.cos(t)

    theta = current_pose[YAW]

    dtheta = w * dt
    dx = u * np.cos(theta) * dt
    dy = u * np.sin(theta) * dt

    next_pose[X] = current_pose[X] + dx
    next_pose[Y] = current_pose[Y] + dy
    next_pose[YAW] = current_pose[YAW] + dtheta

    return next_pose


def euler_floor_t(current_pose, t, dt):
    return euler(current_pose, np.floor(t), dt)


def rk4(current_pose, t, dt):
    next_pose = current_pose.copy()
    theta = current_pose[YAW]
    u = 0.25
    x = current_pose[X]
    y = current_pose[Y]

    k1 = dt * np.cos(t)
    k2 = dt * np.cos(t + dt/2)
    k3 = dt * np.cos(t + dt/2)
    k4 = dt * np.cos(t + dt)
    dtheta = 1/6 * (k1 + 2*k2 + 2*k3 + k4)
    next_pose[YAW] = theta + dtheta

    k1_x = dt * u * np.cos(theta)
    k2_x = dt * u * np.cos(theta + k1/2)
    k3_x = dt * u * np.cos(theta + k2/2)
    k4_x = dt * u * np.cos(theta + k3)
    dx = 1/6 * (k1_x + 2*k2_x + 2*k3_x + k4_x)
    next_pose[X] = x + dx

    k1_y = dt * u * np.sin(theta)
    k2_y = dt * u * np.sin(theta + k1/2)
    k3_y = dt * u * np.sin(theta + k2/2)
    k4_y = dt * u * np.sin(theta + k3)
    dy = 1/6 * (k1_y + 2*k2_y + 2*k3_y + k4_y)
    next_pose[Y] = y + dy

    return next_pose


def rk4_floor_t(current_pose, t, dt):
    next_pose = current_pose.copy()
    theta = current_pose[YAW]
    u = 0.25
    x = current_pose[X]
    y = current_pose[Y]

    k1 = dt * np.cos(np.floor(t))
    k2 = dt * np.cos(np.floor(t + dt/2))
    k3 = dt * np.cos(np.floor(t + dt/2))
    k4 = dt * np.cos(np.floor(t + dt))
    dtheta = 1/6 * (k1 + 2*k2 + 2*k3 + k4)
    next_pose[YAW] = theta + dtheta

    k1_x = dt * u * np.cos(theta)
    k2_x = dt * u * np.cos(theta + k1/2)
    k3_x = dt * u * np.cos(theta + k2/2)
    k4_x = dt * u * np.cos(theta + k3)
    dx = 1/6 * (k1_x + 2*k2_x + 2*k3_x + k4_x)
    next_pose[X] = x + dx

    k1_y = dt * u * np.sin(theta)
    k2_y = dt * u * np.sin(theta + k1/2)
    k3_y = dt * u * np.sin(theta + k2/2)
    k4_y = dt * u * np.sin(theta + k3)
    dy = 1/6 * (k1_y + 2*k2_y + 2*k3_y + k4_y)
    next_pose[Y] = y + dy

    return next_pose


def main(args):
    print('Using method {}'.format(args.method))
    integration_method = globals()[args.method]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.ion()  # Interactive mode.
    plt.grid('on')
    plt.axis('equal')
    plt.xlim([-0.5, 2])
    plt.ylim([-0.75, 1.25])
    plt.show()
    colors = colors_from('jet', len(args.dt))

    # Show all dt.
    for color, dt in zip(colors, args.dt):
        print('Using dt = {}'.format(dt))

        # Initial robot pose (x, y and theta).
        robot_pose = np.array([0., 0., 0.], dtype=np.float32)
        robot_drawer = RobotDrawer(
            ax, robot_pose, color=color, label='dt = %.3f [s]' % dt)
        if args.animate:
            fig.canvas.draw()
            fig.canvas.flush_events()

        # Simulate for 10 seconds.
        last_time_drawn = 0.
        last_time_drawn_real = time.time()
        for t in np.arange(0., 10., dt):
            robot_pose = integration_method(robot_pose, t, dt)

            plt.title('time = %.3f [s] with dt = %.3f [s]' % (t + dt, dt))
            robot_drawer.update(robot_pose)

            # Do not draw too many frames.
            time_drawn = t
            if args.animate and (time_drawn - last_time_drawn > REFRESH_RATE):
                # Try to draw in real-time.
                time_drawn_real = time.time()
                delta_time_real = time_drawn_real - last_time_drawn_real
                if delta_time_real < REFRESH_RATE:
                    time.sleep(REFRESH_RATE - delta_time_real)
                last_time_drawn_real = time_drawn_real
                last_time_drawn = time_drawn
                fig.canvas.draw()
                fig.canvas.flush_events()
        robot_drawer.done()

    plt.ioff()
    plt.title('Trajectories')
    plt.legend(loc='lower right')
    plt.show(block=True)


# Simple class to draw and animate a robot.
class RobotDrawer(object):

    def __init__(self, ax, pose, radius=.05, label=None, color='g'):
        self._pose = pose.copy()
        self._radius = radius
        self._history_x = [pose[X]]
        self._history_y = [pose[Y]]
        self._outside = ax.plot([], [], 'b', lw=2)[0]
        self._front = ax.plot([], [], 'b', lw=2)[0]
        self._path = ax.plot([], [], c=color, lw=2, label=label)[0]
        self.draw()

    def update(self, pose):
        self._pose = pose.copy()
        self._history_x.append(pose[X])
        self._history_y.append(pose[Y])
        self.draw()

    def draw(self):
        a = np.linspace(0., 2 * np.pi, 20)
        x = np.cos(a) * self._radius + self._pose[X]
        y = np.sin(a) * self._radius + self._pose[Y]
        self._outside.set_data(x, y)
        r = np.array([0., self._radius])
        x = np.cos(self._pose[YAW]) * r + self._pose[X]
        y = np.sin(self._pose[YAW]) * r + self._pose[Y]
        self._front.set_data(x, y)
        self._path.set_data(self._history_x, self._history_y)

    def done(self):
        self._outside.set_data([], [])
        self._front.set_data([], [])


def colors_from(cmap_name, ncolors):
    cm = plt.get_cmap(cmap_name)
    cm_norm = matplotlib.colors.Normalize(vmin=0, vmax=ncolors - 1)
    scalar_map = matplotlib.cm.ScalarMappable(norm=cm_norm, cmap=cm)
    return [scalar_map.to_rgba(i) for i in range(ncolors)]


def positive_floats(string):
    values = tuple(float(v) for v in string.split(','))
    for v in values:
        if v <= 0.:
            raise argparse.ArgumentTypeError(
                '{} is not strictly positive.'.format(v))
    return values


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Launches a battery of experiments in parallel')
    parser.add_argument('--method', action='store', default='euler',
                        help='Integration method.', choices=['euler', 'rk4', 'euler_floor_t', 'rk4_floor_t'])
    parser.add_argument('--dt', type=positive_floats,
                        action='store', default=(0.05,), help='Integration step.')
    parser.add_argument('--animate', action='store_true',
                        default=False, help='Whether to animate.')
    args = parser.parse_args()
    main(args)
