import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PID_Controller(Node):

    def __init__(self):
        super().__init__("pid_controller")
        
        # Depth (Y axis) control variables
        self.setpoint = 0
        self.depth = 0
        
        # X axis control variables
        self.setpoint_x = 0
        self.x = 0
        
        # Publishers for thrust (vertical) and thrust_x (horizontal)
        self.pub = self.create_publisher(Float64, 'thrust', 10)
        self.pub_x = self.create_publisher(Float64, 'thrust_x', 10)
        
        # Subscribers for setpoints and current states
        self.sub_setpoint = self.create_subscription(
                Float64,
                'setpoint',
                self.setpoint_callback,
                10)
        self.sub_depth = self.create_subscription(
                Float64,
                'depth',
                self.depth_callback,
                10)
                
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
        
        # Gains for depth (Y axis)
        self.KP_depth = 0.0
        self.KI_depth = 0.0
        self.KD_depth = 0.0
        
        # Gains for X axis (horizontal)
        self.KP_x = 0.0
        self.KI_x = 0
        self.KD_x = 0.0
        
        self.bias = 0.0 # Is a bias necessary?

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
    
    # Callbacks for X axis
    def setpoint_x_callback(self, msg):
        self.setpoint_x = msg.data
    
    def x_callback(self, msg):
        self.x = msg.data



    #TODO: complete this function
    def timer_callback(self):
        # PID for depth

        
        # PID for X axis



        # We should set vertical thrust to 0 if moving upwards, letting buoyancy do the work?

        # Packaging Data
        thrust = Float64()
        thrust.data = 0.0
        
        thrust_x = Float64()
        thrust_x.data = 0.0
        
        # Publishing Data
        self.pub.publish(thrust)
        self.pub_x.publish(thrust_x)

        # self.get_logger().info(f"Publishing thrust: {thrust.data}, thrust_x: {thrust_x.data}")

def main(args=None):
    rclpy.init(args=args)
    controller = PID_Controller()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
