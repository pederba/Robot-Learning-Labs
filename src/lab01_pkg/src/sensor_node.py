#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64

class Sensor(object):
    def __init__(self):

        self.true_state_subscriber = rospy.Subscriber('pendulum_true_state', Float64, self.trueStateCallback)
        self.measurement_publisher = rospy.Publisher('pendulum_measurement', Float64, queue_size=10)

    def trueStateCallback(self, msg):
        x_t_true = msg.data

        x_t_measurement = Float64()
        x_t_measurement.data = self.measure(x_t_true)
        self.measurement_publisher.publish(x_t_measurement)

    def measure(self, x_true):
        """
            Simulate the measurement of the pendulum
        """
        measurement_noise_var = 0.05  # Actual measurement noise variance (uknown to the user)
        z_t = x_true + np.sqrt(measurement_noise_var)*np.random.randn()
        return z_t

if __name__ == '__main__':
    rospy.init_node('sensor_node', anonymous=True)
    sensor = Sensor()
    rospy.spin()
