import pygame
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

# global variables
time_interval = 0.05
display_offset = 40
display_scale = 30
tolerance = 0.2
display_size = (250, 560)
max_depth = 15
drag_coefficient = 3
gravity = 9.81
fluid_density = 41.6666666667 * (gravity + 2) / gravity
start_time = time.time()
blue = (41, 128, 185)
green = (46, 204, 113)
red = (231, 76, 60)
screen = None
surface = None

class Motion:
    """Class representing the 2D motion of the vehicle"""
    def __init__(
            self,
            y=0, vy=0,
            x=0, vx=0,
            drag_coeff=3,
            mass=1,
            fluid_density=997,
            gravity=9.81,
            height=0.6,
            width=0.2,
            dt=0.05,
            env_depth=10,
            env_width=8.0,
            thrust_limit=2):
        self.y = y
        self.vy = vy
        self.x = x
        self.vx = vx
        self.drag_coeff = drag_coeff
        self.mass = mass
        self.fluid_density = fluid_density
        self.g = gravity
        self.width = width
        self.height = height
        self.dt = dt
        self.env_constraints = (-5, env_depth)
        self.env_constraints_x = (0, env_width)
        self.thrust_limit = thrust_limit

    @staticmethod
    def constrain(val, low, high):
        return min(high, max(low, val))

    def volume_submerged(self):
        return (self.height + min(self.y, 0)) * self.width ** 2

    def update(self, vertical_thrust=0, horizontal_thrust=0):
        thrust = self.constrain(vertical_thrust, -self.thrust_limit, self.thrust_limit)
        thrust_x = self.constrain(horizontal_thrust, -self.thrust_limit, self.thrust_limit)
        # Forces for vertical (y) axis
        Fb = -self.fluid_density * self.g * self.volume_submerged()
        Fg = self.mass * self.g
        Fd = -.5 * (-1 if self.vy < 0 else 1 if self.vy > 1 else 0) * self.drag_coeff * self.width ** 2 * self.vy ** 2
        F = Fb + Fg + Fd - thrust * self.mass + random.normalvariate(0, 0.1)
        nv = self.vy + (F / self.mass * self.dt)
        ny = self.constrain(self.y + nv * self.dt, *self.env_constraints)
        nv *= (self.env_constraints[0] < ny) * (ny < self.env_constraints[1])
        self.vy = nv
        self.y = ny
        # Forces for horizontal (x) axis
        Fd_x = -.5 * (-1 if self.vx < 0 else 1 if self.vx > 1 else 0) * self.drag_coeff * self.height ** 2 * self.vx ** 2
        Fx = Fd_x + thrust_x * self.mass
        nvx = self.vx + (Fx / self.mass * self.dt)
        nx = self.constrain(self.x + nvx * self.dt, *self.env_constraints_x)
        nvx *= (self.env_constraints_x[0] < nx) * (nx < self.env_constraints_x[1])
        self.vx = nvx
        self.x = nx

    def get_pos(self):
        return self.y, self.x

class Vehicle(Node):
    """ROS Node class representing the vehicle simulation"""
    def __init__(self):
        super().__init__("vehicle_simulation")
        self.setpoint = 5
        self.setpoint_x = 0
        self.thrust = 0
        self.thrust_x = 0
        self.vehicle_width = 0.2
        self.vehicle_height = 0.6
        self.vehicle_mass = 1
        self.motion = Motion(
                y=0,
                vy=0,
                x=0,
                vx=0,
                drag_coeff=drag_coefficient,
                mass=self.vehicle_mass,
                fluid_density=fluid_density,
                gravity=gravity,
                height=self.vehicle_height,
                width=self.vehicle_width,
                dt=time_interval,
                env_depth=max_depth,
                env_width=display_size[0] / display_scale,
                thrust_limit=10)
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_x = self.create_publisher(Float64, 'x', 10)
        self.pub_setpoint = self.create_publisher(Float64, 'setpoint', 10)
        self.pub_setpoint_x = self.create_publisher(Float64, 'setpoint_x', 10)
        self.sub_thrust = self.create_subscription(Float64, 'thrust', self.vertical_callback, 10)
        self.sub_thrust_x = self.create_subscription(Float64, 'thrust_x', self.horizontal_callback, 10)
        self.timer = self.create_timer(time_interval, self.timer_callback)
        self.count_within_threshold = 0

    def vertical_callback(self, msg):
        self.thrust = min(4, max(-4, msg.data))

    def horizontal_callback(self, msg):
        self.thrust_x = min(4, max(-4, msg.data))

    def reached_goal(self):
        return (abs(self.depth - self.setpoint) < tolerance and abs(self.x - self.setpoint_x) < tolerance)

    def update(self):
        self.depth, self.x = self.motion.get_pos()
        # publish telemetry
        depth = Float64()
        depth.data = float(self.depth)
        xmsg = Float64()
        xmsg.data = float(self.x)
        setpoint = Float64()
        setpoint.data = float(self.setpoint)
        setpoint_x = Float64()
        setpoint_x.data = float(self.setpoint_x)
        self.pub_depth.publish(depth)
        self.pub_x.publish(xmsg)
        self.pub_setpoint.publish(setpoint)
        self.pub_setpoint_x.publish(setpoint_x)
        if not self.reached_goal():
            self.count_within_threshold = 0
        else:
            self.count_within_threshold += 1
        if self.count_within_threshold == 40:
            self.get_logger().info(
                    f"Reached x = {self.setpoint_x:.2f}m, depth = {self.setpoint:.2f}m in " +
                    f"{time.time()-start_time:.2f} seconds")
        self.motion.update(self.thrust, self.thrust_x)

    def get_y(self, depth):
        """Get the y pos of a given depth in the window"""
        return depth * display_scale + display_offset

    def get_x(self, x):
        """Get the x pos of a given x coordinate in the window"""
        return x * display_scale

    def update_setpoint(self, depth, x=None):
        self.setpoint = depth
        if x is not None:
            self.setpoint_x = x

    def timer_callback(self):
        self.update()
        setpoint_line_colour = green if self.reached_goal() else red
        screen.fill((0, 0, 0))
        w, h = surface.get_width(), surface.get_height()
        # water
        pygame.draw.rect(
                screen,
                blue,
                (0,
                self.get_y(0),
                w,
                self.get_y(max_depth + self.vehicle_height) - display_offset),
                0)
        # water surface
        pygame.draw.rect(
                screen,
                (255, 255, 255),
                (0, 0, w, self.get_y(0)),
                0)
        # vehicle rectangle (depth, x)
        vehicle_rect = pygame.Rect(
            self.get_x(self.x),
            self.get_y(self.depth),
            self.vehicle_width * display_scale,
            self.vehicle_height * display_scale
        )
        pygame.draw.rect(
                screen,
                (230, 126, 34),
                vehicle_rect,
                0)
        # tiny x marker, centered horizontally and vertically
        marker_w, marker_h = 8, 8  # Small block
        marker_rect = pygame.Rect(
            self.get_x(self.x) + (self.vehicle_width * display_scale) // 2 - marker_w // 2,
            self.get_y(self.depth) + (self.vehicle_height * display_scale) // 2 - marker_h // 2,
            marker_w, marker_h
        )
        pygame.draw.rect(screen, (255, 255, 0), marker_rect, 0)

        # setpoint line (horizontal)
        pygame.draw.lines(
                screen,
                setpoint_line_colour,
                False,
                ((0, self.get_y(self.setpoint)),
                (w, self.get_y(self.setpoint))),
                3)
        # setpoint marker (vertical): small block
        marker_width_pixels = 6
        marker_height_pixels = 18
        marker_x_center = self.get_x(self.setpoint_x)
        marker_y_center = self.get_y(self.setpoint)
        pygame.draw.rect(
            screen,
            setpoint_line_colour,
            (marker_x_center - marker_width_pixels // 2,
             marker_y_center - marker_height_pixels // 2,
             marker_width_pixels,
             marker_height_pixels),
            0
        )

        # Render telemetry values at side, in black
        font = pygame.font.SysFont(None, 24)
        depth_text = font.render(f"Depth: {self.depth:.2f} m", True, (0, 0, 0))
        x_text = font.render(f"X: {self.x:.2f} m", True, (0, 0, 0))
        screen.blit(depth_text, (w - depth_text.get_width() - 10, 10))
        screen.blit(x_text, (w - x_text.get_width() - 10, 40))

        pygame.display.update()

        # user input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()  # tuple: (x, y)
                depth = Motion.constrain((pos[1] - display_offset) / display_scale, 0, max_depth)
                xpoint = Motion.constrain(pos[0] / display_scale, 0, w / display_scale)
                self.update_setpoint(depth, xpoint)
                global start_time
                start_time = time.time()
                self.get_logger().info(f"New setpoint: x={self.setpoint_x:.2f}m, depth={self.setpoint:.2f}m")

def main(args=None):
    global screen
    global surface
    rclpy.init(args=args)
    pygame.init()
    screen = pygame.display.set_mode(display_size)
    surface = pygame.display.get_surface()
    pygame.display.set_caption("Vehicle Simulation")
    vehicle = Vehicle()
    try:
        global start_time
        start_time = time.time()
        rclpy.spin(vehicle)
    except KeyboardInterrupt:
        pass
    pygame.quit()

if __name__ == "__main__":
    main()
