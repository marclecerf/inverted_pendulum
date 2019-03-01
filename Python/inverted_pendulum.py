from scipy.integrate import ode
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

class Pendulum(object):
    def __init__(self, ax, length, theta):
        self.length = length
        self.ax = ax
        self.line = Line2D([], [], marker='o')
        self.ax.add_line(self.line)
        self.theta = theta
    @property
    def theta(self):
        return self._theta
    @theta.setter
    def theta(self, theta):
        self._theta = theta
        x = length * np.sin(theta)
        y = -1 * length * np.cos(theta)
        self.line.set_xdata([0, x])
        self.line.set_ydata([0, y])

class PendulumPhysics(object):
    def __init__(self, length, theta0):
        self.length = length
        self.g = 9.81
        self.r = ode(self._f).set_integrator('dopri5')
        self.r.set_initial_value([theta0, 0.], 0.)
        self.t = 0.
    def _f(self, t, x):
        theta = x[0]
        omega = x[1]
        return [omega, -self.g/self.length * np.sin(theta)]
    def step(self, dt):
        self.t += dt
        self.r.integrate(self.t)[0]
    @property
    def theta(self):
        return self.r.y[0]
    @property
    def period(self):
        return 2 * np.pi * np.sqrt(self.length/self.g)

if __name__ == "__main__":
    fig, ax = plt.subplots()
    plt.axis('equal')
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    length = 5.  # m
    theta = np.radians(10.) 
    pendulum = Pendulum(ax, length, theta)
    physics = PendulumPhysics(length, theta)
    dt = 0.05  # s
    print('Expected period ~ %5.1f s' % physics.period)
    def update(ii):
        physics.step(dt)
        pendulum.theta = physics.theta
        return [pendulum.line]
    anim = FuncAnimation(fig, update, frames=np.arange(100),
                         interval=int(dt * 1000), blit=True)
    plt.show()

