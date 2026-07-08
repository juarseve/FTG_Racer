# MIT License
# Copyright (c) 2020 Hongrui Zheng

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # EGO PARAMS
        self.declare_parameter('ego_namespace', '')  
        self.declare_parameter('ego_odom_topic', '')  
        self.declare_parameter('ego_opp_odom_topic', '')  
        self.declare_parameter('ego_scan_topic', '')  
        self.declare_parameter('ego_drive_topic', '')  

        # OPP 1 PARAMS
        self.declare_parameter('opp_namespace', '')  
        self.declare_parameter('opp_odom_topic', '')  
        self.declare_parameter('opp_ego_odom_topic', '')  
        self.declare_parameter('opp_scan_topic', '')  
        self.declare_parameter('opp_drive_topic', '')  

        # OPP 2 PARAMS
        self.declare_parameter('opp2_namespace', 'opp2')  
        self.declare_parameter('opp2_odom_topic', 'odom')  
        self.declare_parameter('opp2_ego_odom_topic', 'opp2_ego_odom')  
        self.declare_parameter('ego_opp2_odom_topic', 'ego_opp2_odom') 
        self.declare_parameter('opp2_scan_topic', 'scan')  
        self.declare_parameter('opp2_drive_topic', 'drive')  

        # SIM PARAMS
        self.declare_parameter('scan_distance_to_base_link', 0.0)  
        self.declare_parameter('scan_fov', 0.0)  
        self.declare_parameter('scan_beams', 0)  
        self.declare_parameter('map_path', '')  
        self.declare_parameter('map_img_ext', '')  
        self.declare_parameter('num_agent', 0)  

        # POSES
        self.declare_parameter('sx', 0.0)  
        self.declare_parameter('sy', 0.0)  
        self.declare_parameter('stheta', 0.0)  

        self.declare_parameter('sx1', 0.0)  
        self.declare_parameter('sy1', 0.0)  
        self.declare_parameter('stheta1', 0.0)  

        self.declare_parameter('sx2', 0.0)  
        self.declare_parameter('sy2', 0.0)  
        self.declare_parameter('stheta2', 0.0)  

        self.declare_parameter('kb_teleop', False)  

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 3:
            raise ValueError('num_agents should be 1, 2, or 3.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents)

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        self.has_opp = False
        self.has_opp2 = False

        if num_agents >= 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

        if num_agents == 3:
            self.has_opp2 = True
            self.opp2_namespace = self.get_parameter('opp2_namespace').value
            sx2 = self.get_parameter('sx2').value
            sy2 = self.get_parameter('sy2').value
            stheta2 = self.get_parameter('stheta2').value
            self.opp2_pose = [sx2, sy2, stheta2]
            self.opp2_speed = [0.0, 0.0, 0.0]
            self.opp2_requested_speed = 0.0
            self.opp2_steer = 0.0
            opp2_scan_topic = self.get_parameter('opp2_scan_topic').value
            opp2_odom_topic = self.opp2_namespace + '/' + self.get_parameter('opp2_odom_topic').value
            opp2_drive_topic = self.get_parameter('opp2_drive_topic').value
            
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1], [sx2, sy2, stheta2]]))
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])
            self.opp2_scan = list(self.obs['scans'][2])
        elif num_agents == 2:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])
        else:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
            self.ego_scan = list(self.obs['scans'][0])

        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        self.timer = self.create_timer(0.004, self.timer_callback)
        self.br = TransformBroadcaster(self)

        # Publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        
        if self.has_opp:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_drive_published = False
            
        if self.has_opp2:
            self.opp2_scan_pub = self.create_publisher(LaserScan, opp2_scan_topic, 10)
            self.opp2_odom_pub = self.create_publisher(Odometry, opp2_odom_topic, 10)
            self.opp2_drive_published = False

        # Subscribers
        self.ego_drive_sub = self.create_subscription(AckermannDriveStamped, ego_drive_topic, self.drive_callback, 10)
        self.ego_reset_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.ego_reset_callback, 10)
        
        if self.has_opp:
            self.opp_drive_sub = self.create_subscription(AckermannDriveStamped, opp_drive_topic, self.opp_drive_callback, 10)
            self.opp_reset_sub = self.create_subscription(PoseStamped, '/goal_pose', self.opp_reset_callback, 10)
            
        if self.has_opp2:
            self.opp2_drive_sub = self.create_subscription(AckermannDriveStamped, opp2_drive_topic, self.opp2_drive_callback, 10)

    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True
        
    def opp2_drive_callback(self, drive_msg):
        self.opp2_requested_speed = drive_msg.drive.speed
        self.opp2_steer = drive_msg.drive.steering_angle
        self.opp2_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx, ry, rqx, rqy, rqz, rqw = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        
        if self.has_opp2:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta], self.opp_pose, self.opp2_pose]))
        elif self.has_opp:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta], self.opp_pose]))
        else:
            self.obs, _ , self.done, _ = self.env.reset(np.array([[rx, ry, rtheta]]))

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx, ry, rqx, rqy, rqz, rqw = pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            if self.has_opp2:
                self.obs, _ , self.done, _ = self.env.reset(np.array([self.ego_pose, [rx, ry, rtheta], self.opp2_pose]))
            else:
                self.obs, _ , self.done, _ = self.env.reset(np.array([self.ego_pose, [rx, ry, rtheta]]))

    def drive_timer_callback(self):
        if self.ego_drive_published and not self.has_opp:
            self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed]]))
        elif self.ego_drive_published and self.has_opp and not self.has_opp2 and self.opp_drive_published:
            self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed]]))
        elif self.ego_drive_published and self.has_opp and self.has_opp2 and self.opp_drive_published and self.opp2_drive_published:
            self.obs, _, self.done, _ = self.env.step(np.array([[self.ego_steer, self.ego_requested_speed], [self.opp_steer, self.opp_requested_speed], [self.opp2_steer, self.opp2_requested_speed]]))
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)
            
        if self.has_opp2:
            opp2_scan = LaserScan()
            opp2_scan.header.stamp = ts
            opp2_scan.header.frame_id = self.opp2_namespace + '/laser'
            opp2_scan.angle_min = self.angle_min
            opp2_scan.angle_max = self.angle_max
            opp2_scan.angle_increment = self.angle_inc
            opp2_scan.range_min = 0.
            opp2_scan.range_max = 30.
            opp2_scan.ranges = self.opp2_scan
            self.opp2_scan_pub.publish(opp2_scan)

        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = self.obs['poses_theta'][1]
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]
            
        if self.has_opp2:
            self.opp2_scan = list(self.obs['scans'][2])
            self.opp2_pose[0] = self.obs['poses_x'][2]
            self.opp2_pose[1] = self.obs['poses_y'][2]
            self.opp2_pose[2] = self.obs['poses_theta'][2]
            self.opp2_speed[0] = self.obs['linear_vels_x'][2]
            self.opp2_speed[1] = self.obs['linear_vels_y'][2]
            self.opp2_speed[2] = self.obs['ang_vels_z'][2]

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            self.opp_odom_pub.publish(opp_odom)
            
        if self.has_opp2:
            opp2_odom = Odometry()
            opp2_odom.header.stamp = ts
            opp2_odom.header.frame_id = 'map'
            opp2_odom.child_frame_id = self.opp2_namespace + '/base_link'
            opp2_odom.pose.pose.position.x = self.opp2_pose[0]
            opp2_odom.pose.pose.position.y = self.opp2_pose[1]
            opp2_quat = euler.euler2quat(0., 0., self.opp2_pose[2], axes='sxyz')
            opp2_odom.pose.pose.orientation.x = opp2_quat[1]
            opp2_odom.pose.pose.orientation.y = opp2_quat[2]
            opp2_odom.pose.pose.orientation.z = opp2_quat[3]
            opp2_odom.pose.pose.orientation.w = opp2_quat[0]
            self.opp2_odom_pub.publish(opp2_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x, ego_t.translation.y, ego_t.translation.z = self.ego_pose[0], self.ego_pose[1], 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x, ego_t.rotation.y, ego_t.rotation.z, ego_t.rotation.w = ego_quat[1], ego_quat[2], ego_quat[3], ego_quat[0]
        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp, ego_ts.header.frame_id, ego_ts.child_frame_id = ts, 'map', self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x, opp_t.translation.y, opp_t.translation.z = self.opp_pose[0], self.opp_pose[1], 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x, opp_t.rotation.y, opp_t.rotation.z, opp_t.rotation.w = opp_quat[1], opp_quat[2], opp_quat[3], opp_quat[0]
            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp, opp_ts.header.frame_id, opp_ts.child_frame_id = ts, 'map', self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)
            
        if self.has_opp2:
            opp2_t = Transform()
            opp2_t.translation.x, opp2_t.translation.y, opp2_t.translation.z = self.opp2_pose[0], self.opp2_pose[1], 0.0
            opp2_quat = euler.euler2quat(0.0, 0.0, self.opp2_pose[2], axes='sxyz')
            opp2_t.rotation.x, opp2_t.rotation.y, opp2_t.rotation.z, opp2_t.rotation.w = opp2_quat[1], opp2_quat[2], opp2_quat[3], opp2_quat[0]
            opp2_ts = TransformStamped()
            opp2_ts.transform = opp2_t
            opp2_ts.header.stamp, opp2_ts.header.frame_id, opp2_ts.child_frame_id = ts, 'map', self.opp2_namespace + '/base_link'
            self.br.sendTransform(opp2_ts)

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x, ego_wheel_ts.transform.rotation.y, ego_wheel_ts.transform.rotation.z, ego_wheel_ts.transform.rotation.w = ego_wheel_quat[1], ego_wheel_quat[2], ego_wheel_quat[3], ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x, opp_wheel_ts.transform.rotation.y, opp_wheel_ts.transform.rotation.z, opp_wheel_ts.transform.rotation.w = opp_wheel_quat[1], opp_wheel_quat[2], opp_wheel_quat[3], opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)
            
        if self.has_opp2:
            opp2_wheel_ts = TransformStamped()
            opp2_wheel_quat = euler.euler2quat(0., 0., self.opp2_steer, axes='sxyz')
            opp2_wheel_ts.transform.rotation.x, opp2_wheel_ts.transform.rotation.y, opp2_wheel_ts.transform.rotation.z, opp2_wheel_ts.transform.rotation.w = opp2_wheel_quat[1], opp2_wheel_quat[2], opp2_wheel_quat[3], opp2_wheel_quat[0]
            opp2_wheel_ts.header.stamp = ts
            opp2_wheel_ts.header.frame_id = self.opp2_namespace + '/front_left_hinge'
            opp2_wheel_ts.child_frame_id = self.opp2_namespace + '/front_left_wheel'
            self.br.sendTransform(opp2_wheel_ts)
            opp2_wheel_ts.header.frame_id = self.opp2_namespace + '/front_right_hinge'
            opp2_wheel_ts.child_frame_id = self.opp2_namespace + '/front_right_wheel'
            self.br.sendTransform(opp2_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)
            
        if self.has_opp2:
            opp2_scan_ts = TransformStamped()
            opp2_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp2_scan_ts.transform.rotation.w = 1.
            opp2_scan_ts.header.stamp = ts
            opp2_scan_ts.header.frame_id = self.opp2_namespace + '/base_link'
            opp2_scan_ts.child_frame_id = self.opp2_namespace + '/laser'
            self.br.sendTransform(opp2_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
