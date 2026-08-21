import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PID_Controller(Node):

    def __init__(self):
        super().__init__("pid_controller")
        
        # Depth (Y axis) control variables
        self.setpoint = -20
        self.depth = 0
        
        # X axis control variables
        self.setpoint_x = 20
        self.x = 0
        
        # Publishers for thrust_depth (vertical) and thrust_x (horizontal)
        self.pub = self.create_publisher(Float64, 'thrust_depth', 10)
        self.pub_x = self.create_publisher(Float64, 'thrust_x', 10)
        
        # Subscribers for setpoints and current states
        self.sub_setpoint = self.create_subscription(
                Float64,
                'setpoint_depth',
                self.setpoint_callback,
                10)
        self.sub_depth = self.create_subscription(
                Float64,
                'depth',
                self.depth_callback,
                10)
                
        #TODO: write subscriptions for 'setpoint_x' and 'x'
        self.sub_setpoint_x = self.create_subscription(
                Float64,
                'setpoint_x',
                self.setpoint_x_callback,
                10)
        self.sub_x = self.create_subscription(
                Float64,
                'x',
                self.x_callback,
                10)

        #TODO: utilise and tune these gains
        # Gains for depth (Y axis)
        self.KP_depth = 2.0
        self.KI_depth = 0
        self.KD_depth = 1.2
        
        # Gains for X axis (horizontal)
        self.KP_x = 2.5
        self.KI_x = 0.0
        self.KD_x = 1.0
        
        self.bias = -2.0 # Is a bias necessary?

        # Variables for integral and derivative error (Y axis)
        self.error_sum_depth = 0
        self.last_error_depth = 0
        
        # Variables for integral and derivative error (X axis)
        self.error_sum_x = 0
        self.last_error_x = 0

        self.timer_period = 0.05 # change PID frequency?
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    # Callbacks for Y axis
    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def depth_callback(self, msg):
        self.depth = msg.data 
    
    #TODO: callbacks for 'setpoint_x' and 'x'

    def setpoint_x_callback(self, msg):
        self.setpoint_x = msg.data

    def x_callback(self, msg):
        self.x = msg.data

    #TODO: complete this function
    def timer_callback(self):
        # PID for depth
        thrust = Float64()
        error_depth = self.setpoint - self.depth
        self.error_sum_depth += error_depth * self.timer_period
        derivative_depth = (error_depth - self.last_error_depth) / self.timer_period
        thrust.data = float(self.KP_depth * error_depth + self.KI_depth * self.error_sum_depth + self.KD_depth * derivative_depth) * -1 + self.bias
        self.last_error_depth = error_depth

        # PID for X axis
        thrust_x = Float64()
        error_x = self.setpoint_x - self.x
        self.error_sum_x += error_x * self.timer_period
        derivative_x = (error_x - self.last_error_x) / self.timer_period
        thrust_x.data = float(self.KP_x * error_x + self.KI_x * self.error_sum_x + self.KD_x * derivative_x)
        self.last_error_x = error_x

        # We should set vertical thrust to 0 if moving upwards, letting buoyancy do the work?

        # Packaging Data
        # thrust.data = 0.0
        

        # thrust_x.data = 0.0
        
        # Publishing Data
        self.pub.publish(thrust)
        self.pub_x.publish(thrust_x)

        self.get_logger().info(f"Publishing thrust: {thrust.data}, thrust_x: {thrust_x.data}")

def main(args=None):
    rclpy.init(args=args)
    controller = PID_Controller()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
