import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class PublisherNode(Node):

    def __init__(self, scenario, const_vel_duration, acceleration, velocity):
        super().__init__('tb_openLoop')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer interval
        sleep_time = 0.2
        self.timer = self.create_timer(sleep_time, self.timer_callback)

        # Scenario
        self.scenario = scenario

        # Motion parameters
        self.const_vel_duration = const_vel_duration
        self.velocity = velocity
        self.acceleration = acceleration

        self.start_time = time.time()

    def timer_callback(self):
        cmd = Twist()
        self.elapsed_time = time.time() - self.start_time

        # Scenario 1: Constant Velocity
        if self.scenario == 1:
            if self.elapsed_time < self.const_vel_duration:
                cmd.linear.x = self.velocity
                self.publisher_.publish(cmd)
                self.get_logger().info(f'Constant velocity: {cmd.linear.x:.2f} m/s')
            else:
                cmd.linear.x = 0.0
                self.publisher_.publish(cmd)
                self.get_logger().info('Stopped')
                rclpy.shutdown()

        # Scenario 2: Acceleration -> Constant -> Deceleration
        elif self.scenario == 2:

            self.deceleration = -(self.acceleration)
            self.acceleration_duration = self.velocity / self.acceleration
            self.deceleration_duration = -(self.velocity / self.deceleration)

            self.total_time = self.acceleration_duration + self.const_vel_duration + self.deceleration_duration

            # Acceleration Phase
            if self.elapsed_time <= self.acceleration_duration:
                cmd.linear.x = min(self.acceleration * self.elapsed_time, self.velocity)
                self.publisher_.publish(cmd)
                self.get_logger().info(f'Accelerating: {cmd.linear.x:.2f} m/s')

            # Constant Velocity Phase
            elif self.elapsed_time <= (self.acceleration_duration + self.const_vel_duration):
                cmd.linear.x = self.velocity
                self.publisher_.publish(cmd)
                self.get_logger().info(f'Constant velocity: {cmd.linear.x:.2f} m/s')

            # Deceleration Phase
            elif self.elapsed_time <= self.total_time:
                deceleration_time = self.elapsed_time - (self.acceleration_duration + self.const_vel_duration)
                cmd.linear.x = max(self.velocity + (self.deceleration * deceleration_time), 0.0)
                self.publisher_.publish(cmd)
                self.get_logger().info(f'Decelerating: {cmd.linear.x:.2f} m/s')

            # Stop the Robot
            else:
                cmd.linear.x = 0.0
                self.publisher_.publish(cmd)
                self.get_logger().info('Stopped')
                rclpy.shutdown()

def main(args=None):
    # User input for selecting scenario and parameters
    scenario = int(input("Enter 1 for Scenario 1 (Constant Velocity) or 2 for Scenario 2 (Acceleration-Constant-Deceleration): "))

    if scenario == 1:
        # Parameters for Scenario 1
        const_vel_duration = float(input("Enter run duration for constant velocity (in seconds): "))
        velocity = float(input("Enter constant velocity (in m/s): "))
        acceleration = 0
        deceleration = 0

    elif scenario == 2:
        # Parameters for Scenario 2
        acceleration = float(input("Enter acceleration (in m/s²): "))
        velocity = float(input("Enter maximum velocity (in m/s): "))
        const_vel_duration = float(input("Enter constant velocity phase duration (in seconds): "))

    else:
        print("Invalid scenario selection. Exiting.")
        return

    rclpy.init(args=args)
    publisher_node = PublisherNode(scenario, const_vel_duration, acceleration, velocity)
    rclpy.spin(publisher_node)

if __name__ == '__main__':
    main()
