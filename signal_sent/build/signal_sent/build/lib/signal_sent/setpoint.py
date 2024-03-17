import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
#from std_msgs.msg import Float32
import numpy as np 
from scipy import signal 

class setpoint(Node):
    def __init__(self):
        super().__init__('setpoint')
        
        #1kHz
        self.signal_timer_period = 0.01
        self.time = 0
        self.signal = 0

        self.declare_parameter('signal_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('default.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('square.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('constant.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sawtooth.amplitude', rclpy.Parameter.Type.DOUBLE)
       
        self.signal_msg = Float64()
        #self.signal_msg = Float32()

        #self.signal_pub = self.create_publisher(Float32, '/setpoint', 10)
        self.signal_pub = self.create_publisher(Float64, '/setpoint', 10)

        self.signal_timer = self.create_timer(self.signal_timer_period, self.signal_timer_callback)

    def signal_timer_callback(self):

        self.time += self.signal_timer_period
        self.current_signal = self.get_parameter('signal_type').get_parameter_value().integer_value
        if self.current_signal == 0:
            self.a = self.get_parameter('default.amplitude').get_parameter_value().double_value
            if self.a > 1:
                self.a = 1
            elif self.a < -1:
                self.a = -1
            omega = 2 * np.pi * 0.1
            wave = self.a * np.sin(omega * self.time + 0) + 0

        elif self.current_signal == 1:
            self.a = self.get_parameter('square.amplitude').get_parameter_value().double_value
            if self.a > 1:
                self.a = 1
            elif self.a < -1:
                self.a = -1
            omega = 2 * np.pi * 2.0
            wave = self.a * signal.square(omega * self.time + 0) + 0

        elif self.current_signal == 2:
            self.a = self.get_parameter('sawtooth.amplitude').get_parameter_value().double_value
            if self.a > 1:
                self.a = 1
            elif self.a < -1:
                self.a = -1
            omega = 2 * np.pi * 0.1
            wave = self.a * signal.sawtooth(omega * self.time + 0) + 0

        elif self.current_signal == 3:
            self.a = self.get_parameter('constant.amplitude').get_parameter_value().double_value
            if self.a > 1:
                self.a = 1
            elif self.a < -1:
                self.a = -1
            wave = self.a 

        self.signal_msg.data = wave
        self.signal_pub.publish(self.signal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = setpoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()