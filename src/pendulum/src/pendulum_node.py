import rospy
import numpy as np
from scipy.integrate import odeint
from std_msgs.msg import Float64

class Pendulum(object):
    def __init__(self):

        self.true_state_publisher = rospy.Publisher('pendulum_true_state', Float64, queue_size=10)
        rate = rospy.Rate(10)

        # Initial true state
        self.x0 = np.array([np.pi/3, 0.5])

    # System dynamics (continuous, non-linear) in state-space representation (https://en.wikipedia.org/wiki/State-space_representation)
    def stateSpaceModel(x,t):
        """
            Dynamics may be described as a system of first-order
            differential equations: 
            dx(t)/dt = f(t, x(t))
        """
        g=9.81
        l=1
        dxdt=np.array([x[1], -(g/l)*np.sin(x[0])])
        return dxdt
    
    def simulate_true_dynamics(self):
        """
            Simulate the true dynamics of the pendulum
        """
        # Discretization time step (frequency of measurements)
        deltaTime=0.01

        # Simulation duration in timesteps
        simulationSteps=400
        totalSimulationTimeVector=np.arange(0, simulationSteps*deltaTime, deltaTime)

        x_t_true=odeint(self.stateSpaceModel, self.x0, totalSimulationTimeVector)

        self.true_state_publisher.publish(x_t_true)

