''' This node is for the autonomous navigation that comes back to start position once it hits a 10 minute time frame'''

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

class DWANavigator(Node):
    def __init__(self):
        super().__init__('dwa_navigator')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/rosbot_base_controller/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.start_pose = None
        self.robot_pose = None
        self.start_time = time.time()
        self.navigating = True
        self.laser_ranges = []
        self.stuck_time = None

        self.MAX_LINEAR_VEL = 0.25
        self.MAX_ANGULAR_VEL = 0.60
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5  # 30 cm
        self.STUCK_DURATION = 60  # 5 seconds
        self.RECOVERY_DISTANCE = 0.2  # Distance to move back during recovery
        self.TURN_ANGLE = math.pi / 2  # 90 degrees
        self.FREE_SPACE_THRESHOLD = 0.5  # 50 cm free space required to move forward
        self.NAVIGATION_TIME = 600  # 10 minute

        self.state = 'MOVING_FORWARD'

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
        self.get_logger().debug(f"Odom callback: Robot pose set to {self.robot_pose}")

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if not self.navigating:
            return

        min_distance = min([x if x != float('inf') else self.OBSTACLE_DISTANCE_THRESHOLD * 2 for x in self.laser_ranges])
        self.get_logger().debug(f"Scan callback: Min distance to obstacle {min_distance}m")

        if min_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
            self.get_logger().debug(f"Obstacle detected at distance {min_distance}m")
            self.avoid_obstacle()
        else:
            self.state = 'MOVING_FORWARD'
            self.move_forward()

        self.check_navigation_time()

    def avoid_obstacle(self):
        if self.state == 'MOVING_FORWARD':
            self.move_backward()
            self.state = 'MOVING_BACKWARD'
        elif self.state == 'MOVING_BACKWARD':
            left_distance = sum(self.laser_ranges[30:90]) / len(self.laser_ranges[30:90])
            right_distance = sum(self.laser_ranges[-90:-30]) / len(self.laser_ranges[-90:-30])

            if left_distance > right_distance and left_distance > self.FREE_SPACE_THRESHOLD:
                self.turn_left()
                self.state = 'TURNING_LEFT'
            elif right_distance > self.FREE_SPACE_THRESHOLD:
                self.turn_right()
                self.state = 'TURNING_RIGHT'
            else:
                self.move_backward()

        elif self.state == 'TURNING_LEFT' or self.state == 'TURNING_RIGHT':
            self.move_forward()
            self.state = 'MOVING_FORWARD'

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.MAX_LINEAR_VEL
        twist_msg.angular.z = 0.0
        self.vel_publisher.publish(twist_msg)
        self.get_logger().debug("Move forward")

    def move_backward(self):
        twist_msg = Twist()
        twist_msg.linear.x = -self.MAX_LINEAR_VEL
        twist_msg.angular.z = 0.0
        self.vel_publisher.publish(twist_msg)
        self.get_logger().debug("Move backward")
        time.sleep(1)  # Move back for 1 second
        self.vel_publisher.publish(Twist())  # Stop moving

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.angular.z = -self.MAX_ANGULAR_VEL
        self.publish_turn(twist_msg, "right")

    def turn_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = self.MAX_ANGULAR_VEL
        self.publish_turn(twist_msg, "left")

    def publish_turn(self, twist_msg, direction):
        turn_duration = self.TURN_ANGLE / self.MAX_ANGULAR_VEL
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.vel_publisher.publish(twist_msg)
            time.sleep(0.1)
        self.vel_publisher.publish(Twist())  # Stop turning
        self.get_logger().debug(f"Turn {direction} completed")

    def check_navigation_time(self):
        current_time = time.time()
        if current_time - self.start_time > self.NAVIGATION_TIME:
            self.navigating = False
            self.get_logger().info("Navigation time is up. Returning to the starting position.")
            self.return_to_start()

    def return_to_start(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose = self.start_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Local and global costmap parameters
        local_costmap_params = {
            'update_frequency': 5.0,
            'publish_frequency': 2.0,
            'global_frame': 'odom',
            'robot_base_frame': 'base_link',
            'width': 3,
            'height': 3,
            'resolution': 0.03,
            'rolling_window': True,
            'always_send_full_costmap': True,
            'footprint': "[[0.17, 0.17], [0.17, -0.17], [-0.17, -0.17], [-0.17, 0.17]]",
            'footprint_padding': 0.03,
            'plugins': ["static_layer", "obstacle_layer", "inflation_layer"]
        }

        global_costmap_params = {
            'update_frequency': 1.0,
            'publish_frequency': 1.0,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'resolution': 0.05,
            'always_send_full_costmap': True,
            'track_unknown_space': True,
            'footprint': "[[0.17, 0.17], [0.17, -0.17], [-0.17, -0.17], [-0.17, 0.17]]",
            'footprint_padding': 0.03,
            'plugins': ["static_layer", "obstacle_layer", "inflation_layer"]
        }

        # Behavior server parameters
        behavior_server_params = {
            'local_costmap_topic': 'local_costmap/costmap_raw',
            'local_footprint_topic': 'local_costmap/published_footprint',
            'global_costmap_topic': 'global_costmap/costmap_raw',
            'global_footprint_topic': 'global_costmap/published_footprint',
            'cycle_frequency': 10.0,
            'behavior_plugins': ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"],
            'spin': {'plugin': "nav2_behaviors/Spin"},
            'backup': {'plugin': "nav2_behaviors/BackUp"},
            'drive_on_heading': {'plugin': "nav2_behaviors/DriveOnHeading"},
            'wait': {'plugin': "nav2_behaviors/Wait"},
            'assisted_teleop': {'plugin': "nav2_behaviors/AssistedTeleop"},
            'local_frame': 'odom',
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'transform_timeout': 0.1,
            'simulate_ahead_time': 2.0,
            'max_rotational_vel': 1.0,
            'min_rotational_vel': 0.4,
            'rotational_acc_lim': 3.2

        }

        # Planner server parameters
        planner_server_params = {
            'expected_planner_frequency': 1.0,
            'planner_plugins': ["GridBased"],
            'GridBased': {'plugin': "nav2_navfn_planner/NavfnPlanner", 'tolerance': 0.2, 'use_astar': False, 'allow_unknown': True}
        }

        # Controller server parameters
        controller_server_params = {
            'controller_frequency': 1.2,
            'min_x_velocity_threshold': 0.03,
            'min_y_velocity_threshold': 0.03,
            'min_theta_velocity_threshold': 0.3,
            'failure_tolerance': 2.0,
            'progress_checker_plugin': "progress_checker",
            'progress_checker': {'plugin': "nav2_controller::SimpleProgressChecker", 'required_movement_radius': 0.5, 'movement_time_allowance': 10.0},
            'goal_checker_plugin': "goal_checker",
            'goal_checker': {'plugin': "nav2_controller::SimpleGoalChecker", 'xy_goal_tolerance': 0.1, 'yaw_goal_tolerance': 0.2, 'stateful': True},
            'controller_plugins': ["FollowPath"],
            'FollowPath': {
                'plugin': "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                'desired_linear_vel': 0.4,
                'lookahead_dist': 0.6,
                'min_lookahead_dist': 0.4,
                'max_lookahead_dist': 0.9,
                'lookahead_time': 2.0,
                'transform_tolerance': 0.1,
                'use_velocity_scaled_lookahead_dist': True,
                'min_approach_linear_velocity': 0.1,
                'approach_velocity_scaling_dist': 0.5,
                'use_collision_detection': True,
                'max_allowed_time_to_collision_up_to_carrot': 1.0,
                'use_regulated_linear_velocity_scaling': True,
                'use_cost_regulated_linear_velocity_scaling': False,
                'regulated_linear_scaling_min_radius': 0.7,
                'regulated_linear_scaling_min_speed': 0.2,
                'allow_reversing': False,
                'use_rotate_to_heading': True,
                'rotate_to_heading_min_angle': 0.785,
                'max_angular_accel': 5.0,
                'rotate_to_heading_angular_vel': 1.8,
                'max_robot_pose_search_dist': 10.0,
                'use_interpolation': True
            }
        }

        # Send the goal to the navigation stack
        self.navigation_client.wait_for_server()
        future = self.navigation_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed')

def main(args=None):
    rclpy.init(args=args)
    dwa_navigator = DWANavigator()
    rclpy.spin(dwa_navigator)
    dwa_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()