#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.integrate import odeint
from std_msgs.msg import Float64

class Pendulum(object):
    def __init__(self):

        self.true_state_publisher = rospy.Publisher('pendulum_true_state', Float64, queue_size=10)
        self.rate = rospy.Rate(10)

        # Initial true state
        self.x0 = np.array([np.pi/3, 0.5])

    # System dynamics (continuous, non-linear) in state-space representation (https://en.wikipedia.org/wiki/State-space_representation)
    def stateSpaceModel(self, x, t):
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

        for timestep in range(simulationSteps):
            x_t_true_msg = Float64()
            x_t_true_msg.data = x_t_true[timestep,1]

            self.true_state_publisher.publish(x_t_true_msg)
            self.rate.sleep()
        
        rospy.signal_shutdown("Simulation finished")
        

if __name__ == '__main__':
    rospy.init_node('pendulum_node', anonymous=True)
    pendulum = Pendulum()
    pendulum.simulate_true_dynamics()
    rospy.spin()