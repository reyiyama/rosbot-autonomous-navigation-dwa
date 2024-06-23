import rclpy
import rclpy.duration
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Path

class ArucoMarkers(Node):
    def __init__(self):
        super().__init__('aruco_markers')
        
        # Initialize variables
        self.laser_reading = None
        self.get_logger().info('ArucoMarkers Node started')
        
        # Marker publisher
        self.publisher = self.create_publisher(Marker, '/aruco_markers', 10)
        self.get_logger().info('Marker Publisher created')
        
        # Path publisher
        self.path_publisher_ = self.create_publisher(Path, '/path', 10)

        # Object subscriber
        self.object_subscriber = self.create_subscription(Float32MultiArray, '/objects', self.marker_callback, 10)
        
        # Laser subscriber
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data)
        
        # Transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for path publisher
        self.timer = self.create_timer(1.0, self.path_publisher_callback)
    
        # Detected markers list
        self.detected_markers = []
        
        # Waypoints list for path
        self.waypoints = []
        
    def marker_callback(self, msg):
        if msg.data:
            """ Handle ArUco marker detection """
            marker_id = int(msg.data[0]) 
            self.get_logger().info(f'ArUco Marker {marker_id} detected')

            # Check if marker is already detected
            if marker_id not in self.detected_markers:

                # Get laser data
                laser_msg = self.laser_reading
                while laser_msg is None:
                    self.get_logger().info(f"laser msg recieved none")
                    # Loop until valid laser data is received
                    laser_msg = self.laser_reading

                if laser_msg:
                    if laser_msg.ranges[0] == float('inf'):
                        laser_msg = None

                # Check if laser data is valid
                if laser_msg and laser_msg.ranges[0] != float('inf'):
                    self.get_logger().info("Updated")
                    self.get_logger().info(f"{laser_msg}")
                    # Transform laser data
                    transform_success = self.transform_laser(laser_msg, marker_id)
                    if transform_success:
                        # Add marker to detected markers list
                        self.detected_markers.append(marker_id) 
                        self.get_logger().info(f'Recieved ArUco Markers are: {self.detected_markers}')

                    else:
                        self.get_logger().info('Transform failed...')
                else:
                    self.get_logger().info('Laser data invalid...')
            else:
                self.get_logger().info(f'ArUco Marker {marker_id} already detected')

    def laser_callback(self, msg):
        """ Callback for laser data """
        # Store laser data
        self.laser_reading = msg

    def path_publisher_callback(self):
        """ Publish path callback """
        map_pose = self.transform_baselink()
        if map_pose:
            # Publish path
            self.publish_path(map_pose)

    def transform_baselink(self):
        """ Transform from base_link frame to map frame """
        try:
            base_link_pose = geometry_msgs.msg.Pose()
            base_link_pose.position.x = 0.0
            base_link_pose.position.y = 0.0
            base_link_pose.position.z = 0.0
            base_link_pose.orientation.w = 1.0
            
            # Configure frames
            dest = 'map'
            src = 'base_link'
            time = self.get_clock().now().to_msg() 

            # Timeout for transform data
            timeout = rclpy.duration.Duration(seconds=0.05)
            
            # Lookup a transform
            transform = self.tf_buffer.lookup_transform(dest, src, time, timeout=timeout)
            
            # Apply a transform to a stamped pose
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = time
            pose_stamped.header.frame_id = src
            pose_stamped.pose = base_link_pose
            map_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            if map_pose:
                self.get_logger(f"Base_link transform successful : {map_pose}")
                return map_pose
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform {src} to {dest}: {ex}')

    def transform_laser(self, msg, obj_id):
        """ Transform from laser frame to map frame """
        try:
            laser_time = Time.from_msg(msg.header.stamp)
            measurement = msg.ranges[0]
            time = laser_time.to_msg()
            src = 'laser'
            dest = 'map'
            
            # Laser pose of 0,0,0,0 in its own frame
            laser_pose = geometry_msgs.msg.Pose()
            laser_pose.position.x = -1 * measurement
            laser_pose.position.y = 0.0
            laser_pose.position.z = 0.0
            laser_pose.orientation.w = 1.0
            
            # Timeout for transform data
            timeout = rclpy.duration.Duration(seconds=0.05)
            
            # Lookup a transform
            transform = self.tf_buffer.lookup_transform(dest, src, time, timeout=timeout)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = time
            pose_stamped.header.frame_id = src
            pose_stamped.pose = laser_pose
            map_pose = tf2_geometry_msgs.do_transform_pose(laser_pose, transform)
            if map_pose:
                # Publish the marker
                self.publish_marker(map_pose, obj_id)
                return True
            else:
                return False
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform {src} to {dest}: {ex}')

    def publish_marker(self, pose, obj_id):
        """ Publish method for ArUco markers """
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.id = obj_id
        msg.pose.position.x = pose.position.x
        msg.pose.position.y = pose.position.y
        msg.pose.position.z = pose.position.z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        self.publisher.publish(msg)
        self.get_logger().info(f'ArUco Marker {obj_id} published')

    def publish_path(self, pose):
        """ Publish path """
        time = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = time
        
        poseMap = geometry_msgs.msg.PoseStamped()
        poseMap.header.frame_id = 'map'
        poseMap.header.stamp = time
        poseMap.pose.position.x = pose.position.x
        poseMap.pose.position.y = pose.position.y
        poseMap.pose.position.z = pose.position.z
        poseMap.pose.orientation.x = pose.orientation.x
        poseMap.pose.orientation.y = pose.orientation.y
        poseMap.pose.orientation.z = pose.orientation.z
        poseMap.pose.orientation.w = pose.orientation.w
        
        self.waypoints.append(poseMap)
        
        # Add waypoints to path
        for each_pose in self.waypoints:
            path.poses.append(each_pose)
            
        self.path_publisher_.publish(path)
        self.get_logger().info('Path published')

def main(args=None):
    rclpy.init(args=args)
    aruco_markers = ArucoMarkers()
    rclpy.spin(aruco_markers)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
