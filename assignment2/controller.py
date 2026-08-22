import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PID_Controller(Node):

    def __init__(self):
        super().__init__("pid_controller")

        # Path Planning params
        self.min_x = 0.5
        self.max_x = 7.8
        self.min_depth = 0.5
        self.max_depth = 14.5

        self.declare_parameter("mode", "manual") # manual or sweep
        self.declare_parameter("n_sweeps", 3) # number of sweeps for sweep mode
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.n_sweeps = self.get_parameter("n_sweeps").get_parameter_value().integer_value

        self.waypoints_x, self.waypoints_depth = self.gen_n_sweeps_waypoints(self.n_sweeps)
        self.wp_index = 0

        # Depth (Y axis) control variables
        self.setpoint = 0.0
        self.depth = 0.0

        # Store manually clicked setpoints for depth and x
        self.click_depth = 0.0
        self.click_x = 0.0
        
        # X axis control variables
        self.setpoint_x = 0.0
        self.x = 0.0
        
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
        self.KD_depth = 2.0
        
        # Gains for X axis (horizontal)
        self.KP_x = 2.5
        self.KI_x = 0.0
        self.KD_x = 1.5
        
        self.bias = -2.0 # Is a bias necessary?

        # Variables for integral and derivative error (Y axis)
        self.error_sum_depth = 0
        self.last_error_depth = 0
        
        # Variables for integral and derivative error (X axis)
        self.error_sum_x = 0
        self.last_error_x = 0

        self.timer_period = 0.05 # change PID frequency?
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def gen_n_sweeps_waypoints(self, n_sweeps):
        waypoints_x = []
        waypoints_depth = []
        depth_addition = (self.max_depth - self.min_depth) / n_sweeps
        for i in range(n_sweeps):
            if (i % 2 == 0):
                waypoints_x.append(self.min_x)
                waypoints_x.append(self.max_x)
            else:
                waypoints_x.append(self.max_x)
                waypoints_x.append(self.min_x)
            
            waypoints_depth.append(self.min_depth + i * depth_addition)
            waypoints_depth.append(self.min_depth + i * depth_addition)

        return waypoints_x, waypoints_depth

    # Callbacks for Y axis
    def setpoint_callback(self, msg):
        self.click_depth = msg.data

    def depth_callback(self, msg):
        self.depth = msg.data 
    
    #TODO: callbacks for 'setpoint_x' and 'x'

    def setpoint_x_callback(self, msg):
        self.click_x = msg.data

    def x_callback(self, msg):
        self.x = msg.data

    #TODO: complete this function
    def timer_callback(self):
        if self.mode == "sweep":
            self.setpoint_x = self.waypoints_x[self.wp_index]
            self.setpoint = self.waypoints_depth[self.wp_index]
                # if setpoint is reached, move to next waypoint
            if abs(self.x - self.setpoint_x) < 0.1 and abs(self.depth - self.setpoint) < 0.1:
                self.wp_index += 1
                if self.wp_index >= len(self.waypoints_x):
                    self.wp_index = 0
        else:
            self.setpoint = self.click_depth
            self.setpoint_x = self.click_x
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
