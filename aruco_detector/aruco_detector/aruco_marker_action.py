import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import os
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
# from threading import Thread, Lock

from math import atan2


class DWANavigator(Node):

    PEN_LENGTH = 0.3
    def __init__(self):
        super().__init__('dwa_navigator')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/rosbot_base_controller/odom', self.odom_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.object_subscriber = self.create_subscription(Float32MultiArray, '/objects', self.object_callback, 10)

        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.start_pose = None
        self.robot_pose = None
        self.start_time = time.time()
        self.navigating = True
        self.laser_ranges = []
        self.stuck_time = None
        self.recovery_start_time = None
        
        self.MAX_LINEAR_VEL = 0.25
        self.MAX_ANGULAR_VEL = 0.60
        self.OBSTACLE_DISTANCE_THRESHOLD = 0.5
        self.NAVIGATION_TIME = 60  # 5 minutes
        self.STUCK_DURATION = 5  # 5 seconds
        self.RECOVERY_DISTANCE = 0.2  # Distance to move back during recovery
        self.TURN_ANGLE = math.pi / 2  # 90 degrees
        self. PEN_LENGTH = 0.3

        # Iteration
        self.aruco_markers = []
        self.marker_id = None
        self.aruco_markers_found = 0
        self.target_aruco_markers = 4
        self.starting_position = None
        self.visited_markers = set()


    def spin_360(self):
        """ This method makes the robot spin 360 degrees. """
        self.get_logger().info("Spinning 360 degrees...")

        # Calculate the duration needed to complete a 360-degree turn
        angular_speed = 0.5  # Reduced angular speed for more controlled rotation
        duration = 2 * 3.14159 / angular_speed

        # Create a Twist message to command the spin
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed

        # Spin for the calculated duration with higher precision timing
        start_time = self.get_clock().now().nanoseconds
        end_time = start_time + int(duration * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            # Check if any obstacles are too close during rotation
            if self.laser_ranges:
                # Assuming front ranges are in the 0 to 30 and 330 to 360 degree range
                front_range = self.laser_ranges[:30] + self.laser_ranges[-30:]
                min_front_distance = min(front_range)

                if min_front_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    self.get_logger().warn("Obstacle too close during spin, stopping.")
                    msg.angular.z = 0.0
                    self.vel_publisher.publish(msg)
                    return

            self.vel_publisher.publish(msg)
            time.sleep(0.1)

        # Stop the robot after spinning
        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)
        self.get_logger().info("Spinning complete.")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = msg.pose.pose

    def turn_right(self):
        """ This method makes the robot turn right 90 degrees. """
        self.get_logger().info("Turning right 90 degrees...")

        angular_speed = -0.5  # Reduced angular speed for more controlled turn
        duration = 3.14159 / 2 / abs(angular_speed)

        # Create a Twist message to command the turn
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed

        # Turn for the calculated duration with higher precision timing
        start_time = self.get_clock().now().nanoseconds
        end_time = start_time + int(duration * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            # Check if any obstacles are too close during turn
            if self.laser_ranges:
                # Assuming front ranges are in the 0 to 30 and 330 to 360 degree range
                front_range = self.laser_ranges[:30] + self.laser_ranges[-30:]
                min_front_distance = min(front_range)

                if min_front_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    self.get_logger().warn("Obstacle too close during right turn, stopping.")
                    msg.angular.z = 0.0
                    self.vel_publisher.publish(msg)
                    return

            self.vel_publisher.publish(msg)
            time.sleep(0.1)

        # Stop the robot after turning
        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)
        self.get_logger().info("Right turn complete.")




    def object_callback(self, msg):
        """ This method decides which move to be taken after object detection. """
        if msg.data:
            marker_id = int(msg.data[0])
            self.get_logger().info(f"Received marker id: {marker_id}")

            if marker_id not in self.visited_markers:
                self.visited_markers.add(marker_id)
                self.aruco_markers.append(marker_id)
                self.aruco_markers_found += 1
                self.get_logger().info(f"Total markers received: {self.aruco_markers_found}")
                self.get_logger().info(f"Markers received are: {self.aruco_markers}")

                if marker_id == 5:
                    self.get_logger().info("Marker 5 detected, spinning 360 degrees and pausing for 10 seconds")
                    time.sleep(1)
                    self.spin_360()
                    time.sleep(1)

                elif marker_id == 2:
                    self.get_logger().info("Marker 2 detected, turn right twice")
                    time.sleep(1)
                    self.turn_right()
                    time.sleep(1)
                    self.turn_right()
                    time.sleep(1)

                elif marker_id == 4:
                    self.get_logger().info("Marker 4 detected, sleeping for 10 seconds")
                    time.sleep(10)

                elif marker_id == 3:
                    self.get_logger().info("Marker 3 detected, moving back 1 meter")
                    time.sleep(1)
                    self.move_back()

                elif marker_id == 6:
                    self.get_logger().info("Marker 6 detected, pointing to marker")
                    time.sleep(1)
                    self.move_forward_to_wall()
                    time.sleep(10)

                elif marker_id == 8:
                    self.get_logger().info("Marker 8 detected, circle twice")
                    time.sleep(1)
                    self.handle_marker_8()
                    time.sleep(10)

            else:
                self.get_logger().info(f"Marker {marker_id} detected again, no action taken.")
    
    def is_obstacle_too_close(self):
            """ This method checks if any obstacle is too close based on the laser scan data. """
            if self.laser_ranges:
                front_range = self.laser_ranges[:30] + self.laser_ranges[-30:]
                min_front_distance = min(front_range)

                if min_front_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    return True

            return False

    def handle_marker_8(self):
        if not self.is_obstacle_too_close():
            self.get_logger().info("No obstacles detected, executing marker 6 actions")
            time.sleep(2)
            self.move_in_circles(2)
            self.marker_6_pending = False
        else:
            self.get_logger().warn("Obstacle too close, postponing marker 6 actions")
            self.marker_6_pending = True
         

    def move_in_circles(self, n):
        """ Move the robot in circles n times """
        self.get_logger().info(f"Moving in circles {n} times...")
        for _ in range(n):
            for _ in range(2):  # Two half circles to make a full circle
                angular_speed = 0.5
                linear_speed = 0.2
                duration = 3.14159 / angular_speed  # Half circle duration

                msg = Twist()
                msg.linear.x = linear_speed
                msg.angular.z = angular_speed

                start_time = self.get_clock().now().nanoseconds
                end_time = start_time + int(duration * 1e9)
                while self.get_clock().now().nanoseconds < end_time:
                    if self.is_obstacle_too_close():
                        self.get_logger().warn("Obstacle detected, stopping circle motion")
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        self.vel_publisher.publish(msg)
                        return

                    self.vel_publisher.publish(msg)
                    time.sleep(0.1)

                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.vel_publisher.publish(msg)

        self.get_logger().info("Circle motion complete.")


    
    def move_forward_to_wall(self):
        """ This method makes the robot move forward until it is close to a wall. """
        self.get_logger().info("Moving forward until close to a wall...")

        linear_speed = 0.1  # Reduced linear speed for more controlled forward movement
        safety_distance = 0.1  # Desired distance to keep from the wall (in meters)

        # Create a Twist message to command the forward movement
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = 0.0

        while True:
            # Continuously check for laser range data availability
            if self.laser_ranges:
                # Assuming front ranges are in the 0 to 30 and 330 to 360 degree range
                front_range = self.laser_ranges[:30] + self.laser_ranges[-30:]
                min_front_distance = min(front_range) - 0.3 # Subtract 0.3 meters

                if min_front_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    self.get_logger().warn("Obstacle too close, stopping.")
                    msg.linear.x = 0.0
                    self.vel_publisher.publish(msg)
                    return

                # Calculate the distance to move
                distance_to_move = min_front_distance - safety_distance

                if distance_to_move <= 0:
                    self.get_logger().warn("Already at or too close to the desired safety distance from the wall.")
                    msg.linear.x = 0.0
                    self.vel_publisher.publish(msg)
                    return

                duration = distance_to_move / linear_speed
                end_time = self.get_clock().now().nanoseconds + int(duration * 1e9)

                self.get_logger().info(f"Moving forward {distance_to_move:.2f} meters...")

                while self.get_clock().now().nanoseconds < end_time:
                    # Re-check the distance to ensure safety during the movement
                    if self.laser_ranges:
                        front_range = self.laser_ranges[:30] + self.laser_ranges[-30:]
                        min_front_distance = min(front_range) - 0.3  # Subtract 0.3 meters

                        if min_front_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                            self.get_logger().warn("Obstacle too close during forward movement, stopping.")
                            msg.linear.x = 0.0
                            self.vel_publisher.publish(msg)
                            return

                    self.vel_publisher.publish(msg)
                    time.sleep(0.1)

                # Stop the robot after moving forward
                msg.linear.x = 0.0
                self.vel_publisher.publish(msg)
                return
            else:
                self.get_logger().warn("No laser range data available. Retrying...")
                time.sleep(0.1)


    
    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if not self.navigating:
            return

        laser_filtered = [x if x != float('inf') else self.OBSTACLE_DISTANCE_THRESHOLD * 2 for x in self.laser_ranges]
        min_distance = min(laser_filtered)
        
        if min_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
            if self.stuck_time is None:
                self.stuck_time = time.time()
            elif time.time() - self.stuck_time > self.STUCK_DURATION:
                self.recover_from_stuck()
            else:
                self.avoid_obstacle(min_distance)
        else:
            self.stuck_time = None
            self.move_forward()
            
        self.navigate()

    def navigate(self):
        current_time = time.time()
        if current_time - self.start_time > self.NAVIGATION_TIME:
            self.navigating = False
            self.return_to_start()

    def move_back(self):
        """ This method makes the robot move back one meter while avoiding obstacles. """
        self.get_logger().info("Attempting to move back 1 meter...")

        backward_speed = -0.2
        distance = 1.0
        duration = distance / abs(backward_speed)

        # Check if there's enough space to move back
        if self.laser_ranges:
            # Assuming rear ranges are in the 135 to 225 degree range
            rear_range = self.laser_ranges[135:225]
            min_rear_distance = min(rear_range)

            if min_rear_distance < distance:
                self.get_logger().warn("Not enough distance to move back one meter.")
                self.recovery_start_time = None
                self.stuck_time = None
                self.move_forward()
                return

        # Create a Twist message to command the backward movement
        msg = Twist()
        msg.linear.x = backward_speed
        msg.angular.z = 0.0

        # Move back for the calculated duration while avoiding obstacles
        start_time = self.get_clock().now().nanoseconds
        end_time = start_time + int(duration * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            # Check if any obstacles are too close during backward movement
            if self.laser_ranges:
                # Get the minimum distance behind the robot
                rear_range = self.laser_ranges[135:225]
                min_rear_distance = min(rear_range)

                if min_rear_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    self.get_logger().warn("Obstacle too close during backward movement, stopping.")
                    break

            self.vel_publisher.publish(msg)
            time.sleep(0.1)

        # Stop the robot after moving back
        msg.linear.x = 0.0
        self.vel_publisher.publish(msg)

        # Resume navigation
        self.get_logger().info("Resuming navigation after backward movement.")
        self.move_forward()




    def avoid_obstacle(self, min_distance):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0

        # Determine the direction of free space
        left_range = self.laser_ranges[30:90]
        right_range = self.laser_ranges[-90:-30]
        left_distance = sum(left_range) / len(left_range)
        right_distance = sum(right_range) / len(right_range)

        if left_distance > right_distance:
            # Turn left
            twist_msg.angular.z = self.MAX_ANGULAR_VEL
            self.get_logger().info("Turning left to avoid obstacle")
        else:
            # Turn right
            twist_msg.angular.z = -self.MAX_ANGULAR_VEL
            self.get_logger().info("Turning right to avoid obstacle")

        # Turn for a specific angle
        turn_duration = self.TURN_ANGLE / self.MAX_ANGULAR_VEL
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.vel_publisher.publish(twist_msg)
            time.sleep(0.1)

            # Check if there's enough space to continue turning
            if self.laser_ranges:
                front_range = self.laser_ranges[:30] + self.laser_ranges[-30:]
                min_front_distance = min(front_range)
                if min_front_distance > self.OBSTACLE_DISTANCE_THRESHOLD:
                    # Stop turning and move forward
                    twist_msg.angular.z = 0.0
                    twist_msg.linear.x = self.MAX_LINEAR_VEL
                    self.vel_publisher.publish(twist_msg)
                    return

        # Stop the robot after turning
        twist_msg.angular.z = 0.0
        self.vel_publisher.publish(twist_msg)

        # Move forward after avoiding the obstacle
        self.move_forward()

    def recover_from_stuck(self):
        if self.recovery_start_time is None:
            self.recovery_start_time = time.time()
        elif time.time() - self.recovery_start_time > 5:  # Limit recovery to 5 seconds
            self.get_logger().info("Recovery process timed out")
            self.recovery_start_time = None
            self.stuck_time = None
            self.navigating = False
            return

        # Check if there is enough space to move forward
        forward_range = self.laser_ranges[0:30] + self.laser_ranges[-30:]
        if all(x > self.OBSTACLE_DISTANCE_THRESHOLD for x in forward_range):
            # Move forward if there is enough space
            twist_msg = Twist()
            twist_msg.linear.x = self.MAX_LINEAR_VEL
            twist_msg.angular.z = 0.0
            self.vel_publisher.publish(twist_msg)
            time.sleep(self.RECOVERY_DISTANCE / self.MAX_LINEAR_VEL)
            twist_msg.linear.x = 0.0
            self.vel_publisher.publish(twist_msg)
            self.recovery_start_time = None
            self.stuck_time = None
            self.move_forward()
            return

        # Check if the robot is close to a wall
        left_side = sum(self.laser_ranges[:45]) / 45
        right_side = sum(self.laser_ranges[-45:]) / 45
        if left_side < self.OBSTACLE_DISTANCE_THRESHOLD or right_side < self.OBSTACLE_DISTANCE_THRESHOLD:
            # Turn away from the wall
            if left_side < right_side:
                # Turn right
                twist_msg = Twist()
                twist_msg.angular.z = -self.MAX_ANGULAR_VEL
            else:
                # Turn left
                twist_msg = Twist()
                twist_msg.angular.z = self.MAX_ANGULAR_VEL

            # Turn for a specific angle
            turn_duration = self.TURN_ANGLE / self.MAX_ANGULAR_VEL
            start_time = time.time()
            while time.time() - start_time < turn_duration:
                self.vel_publisher.publish(twist_msg)
                time.sleep(0.1)

            # Stop the robot after turning
            twist_msg.angular.z = 0.0
            self.vel_publisher.publish(twist_msg)

            # Move forward after avoiding the obstacle
            self.move_forward()
            return

        # Move backward while avoiding obstacles
        twist_msg = Twist()
        twist_msg.linear.x = -self.MAX_LINEAR_VEL
        twist_msg.angular.z = 0.0

        start_time = time.time()
        while time.time() - start_time < self.RECOVERY_DISTANCE / self.MAX_LINEAR_VEL:
            # Check obstacle distances during recovery
            if self.laser_ranges:
                min_distance = min(self.laser_ranges)
                if min_distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    # Adjust movement to avoid obstacles
                    if self.laser_ranges[0] < self.OBSTACLE_DISTANCE_THRESHOLD:
                        twist_msg.angular.z = -self.MAX_ANGULAR_VEL  # Turn right
                    elif self.laser_ranges[-1] < self.OBSTACLE_DISTANCE_THRESHOLD:
                        twist_msg.angular.z = self.MAX_ANGULAR_VEL  # Turn left
                else:
                    # Move forward if no obstacles are detected after turning
                    twist_msg.linear.x = self.MAX_LINEAR_VEL
                    twist_msg.angular.z = 0.0
            else:
                # Move forward if no obstacle information is available
                twist_msg.linear.x = self.MAX_LINEAR_VEL
                twist_msg.angular.z = 0.0
                
            self.vel_publisher.publish(twist_msg)
            self.get_logger().info("Recovering from stuck position...")
            time.sleep(0.1)

        # Stop the robot after recovery
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.vel_publisher.publish(twist_msg)

        self.recovery_start_time = None
        self.stuck_time = None
        self.move_forward()

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.MAX_LINEAR_VEL
        twist_msg.angular.z = 0.0
        self.vel_publisher.publish(twist_msg)

    
    def return_to_start(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose = self.start_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

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

        planner_server_params = {
            'expected_planner_frequency': 1.0,
            'planner_plugins': ["GridBased"],
            'GridBased': {'plugin': "nav2_navfn_planner/NavfnPlanner", 'tolerance': 0.2, 'use_astar': False, 'allow_unknown': True}
        }

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