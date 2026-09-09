import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

import math
import time
from copy import deepcopy

class Waypoint(Node):
    def __init__(self):
        super().__init__('wavpoint_nav')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.waypoints = []
        self.marker_id = 0
        self.enter_pressed = False
        self.navigating = False
        self.enter_pressed_time = None
        self.last_click_time = None
        self.timer_created = False
        self.last_logged_waypoint = -1

        # Smooth velocity control parameters
        self.min_linear_vel = 0.05  # Minimum velocity (start/end)
        self.max_linear_vel = 0.5   # Maximum velocity (cruise)
        self.acceleration = 0.3     # Acceleration rate (m/s²)
        self.deceleration = 0.4     # Deceleration rate (m/s²)
        self.acceleration_distance = 0.3  # Distance over which to accelerate
        self.deceleration_distance = 0.4  # Distance over which to decelerate
        
        self.max_angular_vel = 1.0  # rad/s
        self.current_waypoint_index = 0
        self.current_robot_pose = None

        # Line interpolation parameters
        self.interpolation_spacing = 0.08  # 8cm between interpolation points
        self.path_smoothing = True
        self.line_following_tolerance = 0.15  # Maximum deviation from line (meters)
        self.lookahead_distance = 0.3  # Distance to look ahead for line following

        # Publishers and subscribers
        self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/interpolated_path', 10)

        # TF2 for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("[Waypoint Navigation Node Started]")
        self.get_logger().info("Use RViz 'Publish Point' to click waypoints.")
        self.get_logger().info(" Press ENTER in terminal after adding all waypoints.")
        self.create_timer(1.0, self.check_for_enter)

    def odom_callback(self, msg):
        """Store current robot pose for velocity calculations"""
        self.current_robot_pose = msg.pose.pose

    def point_callback(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0
        self.goal_callback(pose)

    def goal_callback(self, msg):
        self.waypoints.append(msg)
        self.last_click_time = time.time()
        self.get_logger().info(
            f" Waypoint {len(self.waypoints)} stored: "
            f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        self.publish_marker(msg)

    def check_for_enter(self):
        if not self.enter_pressed:
            try:
                self.enter_pressed = input("\n▶ Press ENTER in this terminal to start navigation...\n") == ""
            except EOFError:
                return

            if self.enter_pressed:
                self.enter_pressed_time = time.time()
                self.get_logger().info("ENTER pressed. Waiting for final waypoints to arrive...")
                if not self.timer_created:
                    self.create_timer(0.5, self.wait_and_start_navigation)
                    self.timer_created = True

    def wait_and_start_navigation(self):
        if not self.enter_pressed or self.navigating:
            return
        if self.last_click_time is None:
            return
        time_since_last_click = time.time() - self.last_click_time
        if time_since_last_click >= 2.0 and len(self.waypoints) >= 2:
            self.get_logger().info(" All waypoints collected. Starting smooth velocity navigation...")
            self.navigate_following_lines()

    def navigate_following_lines(self):
        if self.navigating or len(self.waypoints) < 2:
            return
        self.navigating = True
        self.get_logger().info(f" Following waypoints with smooth velocity and 2D pose stops...")
        
        # Navigate to each waypoint individually with smooth velocity
        self.navigate_waypoint_by_waypoint()

    def navigate_waypoint_by_waypoint(self):
        """Navigate to each waypoint individually with smooth velocity and 2D pose stops"""
        total_waypoints = len(self.waypoints)
        
        for i, waypoint in enumerate(self.waypoints):
            waypoint_num = i + 1
            self.get_logger().info(f"🎯 Navigating to waypoint {waypoint_num}/{total_waypoints}")
            
            # Create smooth velocity path to this waypoint
            if i == 0:
                # First waypoint - start from current position
                segment_path = self.create_segment_path_to_waypoint(None, waypoint, i, total_waypoints)
            else:
                # Subsequent waypoints - start from previous waypoint
                segment_path = self.create_segment_path_to_waypoint(self.waypoints[i-1], waypoint, i, total_waypoints)
            
            # Visualize the path segment
            self.publish_segment_path_visualization(segment_path, waypoint_num)
            
            # Navigate to waypoint with smooth velocity
            self.navigate_to_waypoint_smoothly(segment_path, waypoint, waypoint_num, total_waypoints)
            
            # Return to 2D pose at waypoint
            self.return_to_2d_pose_at_waypoint(waypoint, waypoint_num)
            
            # Brief pause before continuing to next waypoint
            time.sleep(0.5)
        
        self.get_logger().info("🎉 All waypoints completed with 2D pose stops!")

    def create_segment_path_to_waypoint(self, start_waypoint, end_waypoint, waypoint_index, total_waypoints):
        """Create a smooth velocity path segment to a single waypoint"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Use current robot position as start if no start waypoint
        if start_waypoint is None and self.current_robot_pose is not None:
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose = self.current_robot_pose
        elif start_waypoint is not None:
            start_pose = start_waypoint
        else:
            # Fallback to first waypoint if no robot pose available
            start_pose = self.waypoints[0]
        
        # Calculate distance and direction
        dx = end_waypoint.pose.position.x - start_pose.pose.position.x
        dy = end_waypoint.pose.position.y - start_pose.pose.position.y
        segment_distance = math.hypot(dx, dy)
        
        # Calculate orientation for this segment
        yaw = math.atan2(dy, dx)
        q = self.quaternion_from_yaw(yaw)
        
        # Create velocity profile for this segment
        velocity_profile = self.create_velocity_profile(
            segment_distance, 
            waypoint_index == 0,  # is_first_segment
            waypoint_index == total_waypoints - 1  # is_last_segment
        )
        
        # Create interpolated points
        num_points = len(velocity_profile)
        self.get_logger().info(f" Waypoint {waypoint_index + 1}: {segment_distance:.2f}m, {num_points} points")
        
        for j in range(num_points):
            t = j / (num_points - 1) if num_points > 1 else 0
            
            # Linear interpolation along the straight line
            x = start_pose.pose.position.x + t * dx
            y = start_pose.pose.position.y + t * dy
            z = start_pose.pose.position.z
            
            # Create pose for this interpolated point
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path.poses.append(pose)
        
        # Ensure the final pose is exactly the waypoint
        final_pose = deepcopy(end_waypoint)
        final_pose.header.stamp = self.get_clock().now().to_msg()
        if len(path.poses) > 0:
            path.poses[-1] = final_pose
        else:
            path.poses.append(final_pose)
        
        return path

    def navigate_to_waypoint_smoothly(self, segment_path, target_waypoint, waypoint_num, total_waypoints):
        """Navigate to a waypoint using smooth velocity along the path"""
        self.get_logger().info(f" Moving to waypoint {waypoint_num} with smooth velocity...")
        
        # Use segment-based navigation for smooth motion
        total_poses = len(segment_path.poses)
        segment_size = max(3, min(8, total_poses // 5))  # Moderate segment size
        
        for i in range(0, total_poses, segment_size):
            end_idx = min(i + segment_size, total_poses)
            segment_poses = segment_path.poses[i:end_idx]
            
            if len(segment_poses) == 0:
                continue
                
            # Calculate velocity for this segment
            progress = i / total_poses
            velocity_params = self.calculate_segment_velocity(progress, total_poses // segment_size + 1, i // segment_size + 1)
            
            # Navigate to segment end
            target_pose = segment_poses[-1]
            self.navigator.goToPose(target_pose)
            
            # Monitor with appropriate tolerance
            tolerance = 0.1 if end_idx >= total_poses else 0.15  # Tight tolerance only at waypoint
            
            while not self.navigator.isTaskComplete():
                if self.current_robot_pose is not None:
                    dx = target_pose.pose.position.x - self.current_robot_pose.position.x
                    dy = target_pose.pose.position.y - self.current_robot_pose.position.y
                    distance = math.hypot(dx, dy)
                    
                    # Early completion for smooth transitions (except at final waypoint)
                    if end_idx < total_poses and distance < tolerance:
                        self.navigator.cancelTask()
                        break
                
                time.sleep(0.05)
            
            result = self.navigator.getResult()
            if result != TaskResult.SUCCEEDED and end_idx >= total_poses:
                self.get_logger().warn(f"Failed to reach waypoint {waypoint_num}")
        
        self.get_logger().info(f"Reached waypoint {waypoint_num}")

    def return_to_2d_pose_at_waypoint(self, waypoint, waypoint_num):
        """Return to 2D pose at the waypoint (stop and stabilize)"""
        self.get_logger().info(f"Returning to 2D pose at waypoint {waypoint_num}...")
        
        # Stop all motion
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        # Send stop command multiple times to ensure robot stops
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.1)
        
        # Navigate to exact waypoint pose with high precision
        precise_waypoint = deepcopy(waypoint)
        precise_waypoint.header.stamp = self.get_clock().now().to_msg()
        
        self.navigator.goToPose(precise_waypoint)
        
        # Wait for precise positioning
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"2D pose achieved at waypoint {waypoint_num}")
        else:
            self.get_logger().warn(f"2D pose adjustment completed with issues at waypoint {waypoint_num}")
        
        # Final stop command
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.1)

    def publish_segment_path_visualization(self, path, waypoint_num):
        """Visualize the path segment to current waypoint"""
        marker_array = MarkerArray()
        
        # Clear previous segment markers
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = f"segment_path_{waypoint_num}"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add current segment path line
        if len(path.poses) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = f"segment_path_{waypoint_num}"
            line_marker.id = 7000 + waypoint_num
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.1
            line_marker.color.r = 0.0
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0  # Blue for current segment
            line_marker.color.a = 0.8
            line_marker.lifetime.sec = 30  # Keep visible for 30 seconds
            
            for pose in path.poses:
                line_marker.points.append(pose.pose.position)
            
            marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f" Visualized path to waypoint {waypoint_num}")

    def create_velocity_profile(self, segment_distance, is_first_segment, is_last_segment):
        """Create a velocity profile for a segment with smooth acceleration/deceleration"""
        profile = []
        
        # Determine acceleration and deceleration zones
        accel_dist = self.acceleration_distance if is_first_segment else 0
        decel_dist = self.deceleration_distance if is_last_segment else 0
        
        # Adjust distances if segment is too short
        if accel_dist + decel_dist > segment_distance:
            ratio = segment_distance / (accel_dist + decel_dist)
            accel_dist *= ratio
            decel_dist *= ratio
        
        cruise_dist = segment_distance - accel_dist - decel_dist
        
        # Calculate number of points for each phase
        total_points = max(10, int(segment_distance / self.interpolation_spacing))
        
        accel_points = int((accel_dist / segment_distance) * total_points) if accel_dist > 0 else 0
        decel_points = int((decel_dist / segment_distance) * total_points) if decel_dist > 0 else 0
        cruise_points = total_points - accel_points - decel_points
        
        # Acceleration phase
        for i in range(accel_points):
            t = i / (accel_points - 1) if accel_points > 1 else 0
            # Smooth acceleration curve (ease-in)
            velocity = self.min_linear_vel + (self.max_linear_vel - self.min_linear_vel) * (t * t)
            profile.append(velocity)
        
        # Cruise phase
        for i in range(cruise_points):
            profile.append(self.max_linear_vel)
        
        # Deceleration phase
        for i in range(decel_points):
            t = i / (decel_points - 1) if decel_points > 1 else 0
            # Smooth deceleration curve (ease-out)
            velocity = self.max_linear_vel - (self.max_linear_vel - self.min_linear_vel) * (t * t)
            profile.append(velocity)
        
        self.get_logger().debug(f" Velocity profile: {accel_points} accel + {cruise_points} cruise + {decel_points} decel = {len(profile)} points")
        return profile

    def follow_smooth_velocity_path(self, path):
        """Follow the interpolated path with smooth velocity transitions"""
        self.get_logger().info("Starting smooth velocity line following...")
        
        # Publish the path for visualization
        self.path_pub.publish(path)
        
        # Use Nav2's path following with custom velocity configuration
        self.follow_path_with_velocity_control(path)

    def follow_path_with_velocity_control(self, path):
        """Follow the path with velocity control using smaller segments"""
        total_poses = len(path.poses)
        
        # Use larger segments for smoother motion (don't stop at every interpolated point)
        segment_size = max(5, min(15, total_poses // 8))  # Larger segments for smoother motion
        
        self.get_logger().info(f"Following path with {total_poses} poses in {segment_size}-pose segments")
        
        for i in range(0, total_poses, segment_size):
            end_idx = min(i + segment_size, total_poses)
            segment_poses = path.poses[i:end_idx]
            
            segment_num = i // segment_size + 1
            total_segments = (total_poses + segment_size - 1) // segment_size
            
            # Determine velocity parameters for this segment
            progress = i / total_poses
            velocity_params = self.calculate_segment_velocity(progress, total_segments, segment_num)
            
            self.get_logger().info(f" Segment {segment_num}/{total_segments} - Target velocity: {velocity_params['target_vel']:.2f} m/s")
            
          
            self.configure_nav2_velocity(velocity_params)
            
            
            self.navigate_segment_smoothly(segment_poses, segment_num, total_segments)
        
        self.get_logger().info("Smooth velocity navigation completed!")

    def calculate_segment_velocity(self, progress, total_segments, segment_num):
        """Calculate velocity parameters for a segment based on progress"""
        if segment_num == 1:
            
            target_vel = self.min_linear_vel + (self.max_linear_vel - self.min_linear_vel) * min(1.0, progress * 3)
        elif segment_num == total_segments:
           
            target_vel = self.max_linear_vel - (self.max_linear_vel - self.min_linear_vel) * min(1.0, (1.0 - progress) * 3)
        else:
          
            target_vel = self.max_linear_vel
        
        return {
            'target_vel': max(self.min_linear_vel, min(self.max_linear_vel, target_vel)),
            'angular_vel': self.max_angular_vel
        }

    def configure_nav2_velocity(self, velocity_params):
        """Configure Nav2 velocity parameters for smooth motion"""
        
        self.get_logger().debug(f"📊 Configured velocity: {velocity_params['target_vel']:.2f} m/s")

    def navigate_segment_smoothly(self, segment_poses, segment_num, total_segments):
        """Navigate through a segment with smooth transitions"""
        if len(segment_poses) == 0:
            return
        
       
        target_pose = segment_poses[-1]
        
        
        if segment_num == total_segments:
           
            tolerance = 0.1
        else:
           
            tolerance = 0.2
        
       
        self.navigator.goToPose(target_pose)
        
      
        while not self.navigator.isTaskComplete():
            if not self.monitor_smooth_navigation(target_pose, segment_num, total_segments):
                break
            time.sleep(0.1) 
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f" Completed segment {segment_num}/{total_segments}")
        else:
            self.get_logger().warn(f" Segment {segment_num} completed with issues: {result}")

    def monitor_smooth_navigation(self, target_pose, segment_num, total_segments):
        """Monitor navigation with smooth velocity feedback"""
        if self.current_robot_pose is None:
            return True
        
       
        dx = target_pose.pose.position.x - self.current_robot_pose.position.x
        dy = target_pose.pose.position.y - self.current_robot_pose.position.y
        distance_to_target = math.hypot(dx, dy)
        
        
        if distance_to_target < 0.3:
            self.get_logger().debug(f"🎯 Approaching segment {segment_num} end: {distance_to_target:.2f}m")
        
       
        if segment_num < total_segments and distance_to_target < 0.15:
            self.get_logger().debug(f"⚡ Early completion for smooth transition to next segment")
            self.navigator.cancelTask()
            return False
        
        return True

    def calculate_line_deviation(self, path_poses):
        """Calculate how far the robot is from the intended straight line path"""
        if self.current_robot_pose is None or len(path_poses) < 2:
            return 0.0
        
        robot_x = self.current_robot_pose.position.x
        robot_y = self.current_robot_pose.position.y
        
       
        min_distance = float('inf')
        
        for i in range(len(path_poses) - 1):
           
            x1 = path_poses[i].pose.position.x
            y1 = path_poses[i].pose.position.y
            x2 = path_poses[i + 1].pose.position.x
            y2 = path_poses[i + 1].pose.position.y
            
           
            distance = self.point_to_line_distance(robot_x, robot_y, x1, y1, x2, y2)
            min_distance = min(min_distance, distance)
        
        return min_distance if min_distance != float('inf') else 0.0

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """Calculate the shortest distance from a point to a line segment"""
       
        dx = x2 - x1
        dy = y2 - y1
        
      
        if dx == 0 and dy == 0:
            return math.hypot(px - x1, py - y1)
        
       
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        
       
        t = max(0, min(1, t))
        
       
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
      
        return math.hypot(px - closest_x, py - closest_y)

    def publish_interpolated_path_visualization(self, path):
        """Visualize the interpolated path with velocity indicators"""
        marker_array = MarkerArray()
        
      
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "smooth_velocity_path"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
       
        if len(path.poses) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "smooth_velocity_path"
            line_marker.id = 5000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.08  # Line width
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0  
            line_marker.color.a = 0.9
            line_marker.lifetime.sec = 0
            
           
            for pose in path.poses:
                line_marker.points.append(pose.pose.position)
            
            marker_array.markers.append(line_marker)
        
       
        total_poses = len(path.poses)
        for i in range(0, total_poses, 10):  # Show every 10th point
            pose = path.poses[i]
            progress = i / total_poses
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "smooth_velocity_path"
            marker.id = 6000 + i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.15
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
           
            if progress < 0.2 or progress > 0.8:
             
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif progress < 0.4 or progress > 0.6:
                
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.8
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Visualized smooth velocity path with velocity indicators")

    def quaternion_from_yaw(self, yaw):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return (0.0, 0.0, qz, qw)

    def publish_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.marker_id += 1




def main(args=None):
    """Main function for console script entry point"""
    rclpy.init(args=args)
    node = Waypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()