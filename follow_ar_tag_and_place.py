#!/usr/bin/env python3

import math
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_commander import PlanningSceneInterface
import time
import copy
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def translate(value, leftMin, leftMax, rightMin, rightMax):
    value = min(max(value, leftMin), leftMax)

    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

class MoveArm():
    def __init__(self):
        self.position = None
        self.orientation = None
        self.data = None  # Store data as an instance variable
        moveit_commander.roscpp_initialize(sys.argv)
        scene = PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        self.arm_group=moveit_commander.MoveGroupCommander("arm")
        self.gripper_group=moveit_commander.MoveGroupCommander("gripper")
    
        self.arm_group.set_planning_time(10)
        self.grasp_pose=PoseStamped()
        self.joint1_pub = rospy.Publisher('/gripper_finger1_joint/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/gripper_finger2_joint/command', Float64, queue_size=10)
        self.grasp_pose=PoseStamped()

    def control_joints(self, joint_positions):
        # Publish joint positions
        self.joint1_pub.publish(joint_positions[0])
        self.joint2_pub.publish(joint_positions[1])

    def movearm(self):
        time.sleep(5)

        # print(self.arm_group.get_current_pose())

        self.grasp_pose.header.frame_id = "base_footprint"
        self.grasp_pose.pose.position.x = 0.3290330861089922
        self.grasp_pose.pose.position.y = -0.01086978096035385
        self.grasp_pose.pose.position.z = 0.30036546454443824

        self.grasp_pose.pose.orientation.x = 0.011299476542224629
        self.grasp_pose.pose.orientation.y = 0.4909875622814981
        self.grasp_pose.pose.orientation.z = -0.020041837152399145
        self.grasp_pose.pose.orientation.w = 0.8708627103500876
        self.arm_group.set_pose_target(self.grasp_pose)
            
        plan = self.arm_group.plan()

        self.arm_group.execute(plan[1])
        time.sleep(2)

        joint_positions = [0.016, 0.016] 
        self.control_joints(joint_positions)
        time.sleep(2)

        joint_positions = [0.01, 0.01] 
        self.control_joints(joint_positions)
        time.sleep(2)

        self.arm_group.set_named_target("resting")
        self.arm_group.go()

class ARTagFollower:
    def __init__(self):
        self.last_marker_ts = rospy.Time()
        self.last_marker_position = None
        self.marker_angle = 0.0
        self.marker_distance = 0.0

        self.last_odom_ts = None
        self.odom_position = Vector3()
        self.odom_yaw = 0.0

        self.twist_cmd = Twist()

        self.get_parameters()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.ar_pose_sub = rospy.Subscriber(
            "ar_pose_marker", AlvarMarkers, self.callback_ar_pose, queue_size=1
        )
        self.wheel_odom_sub = rospy.Subscriber(
            "wheel_odom_with_covariance",
            Odometry,
            self.callback_wheel_odom,
            queue_size=1,
        )

    def get_parameters(self):
        self.follow_id = rospy.get_param("~follow_id", 5)
        self.marker_timeout = rospy.Duration(rospy.get_param("~marker_timeout", 0.5))
        self.min_ang_vel = rospy.get_param("~min_ang_vel", 0.1)
        self.max_ang_vel = rospy.get_param("~max_ang_vel", 1.0)
        self.angle_min = rospy.get_param("~angle_min", 0.1)
        self.angle_max = rospy.get_param("~angle_max", 0.7)
        self.min_lin_vel_forward = rospy.get_param("~min_lin_vel_forward", 0.05)
        self.max_lin_vel_forward = rospy.get_param("~max_lin_vel_forward", 0.2)
        self.distance_min_forward = rospy.get_param("~distance_min_forward", 0.5)
        self.distance_max_forward = rospy.get_param("~distance_max_forward", 2.0)
        self.min_lin_vel_reverse = rospy.get_param("~min_lin_vel_reverse", 0.05)
        self.max_lin_vel_reverse = rospy.get_param("~max_lin_vel_reverse", 0.2)
        self.distance_min_reverse = rospy.get_param("~distance_min_reverse", 0.5)
        self.distance_max_reverse = rospy.get_param("~distance_max_reverse", 2.0)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_cmd()
            self.cmd_vel_pub.publish(self.twist_cmd)
            rate.sleep()

    def update_cmd(self):
        # Check for a timeout
        if self.last_marker_ts + self.marker_timeout < rospy.get_rostime():
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = 0.0
            return

        # Get the absolute angle to the marker
        angle = math.fabs(self.marker_angle)

        # Get the direction multiplier
        dir = -1.0 if self.marker_angle < 0.0 else 1.0

        # Calculate angular command
        if angle < self.angle_min:
            ang_cmd = 0.0
        else:
            ang_cmd = translate(
                angle,
                self.angle_min,
                self.angle_max,
                self.min_ang_vel,
                self.max_ang_vel,
            )

        # Calculate linear command
        if self.marker_distance >= self.distance_min_forward:
            lin_cmd = translate(
                self.marker_distance,
                self.distance_min_forward,
                self.distance_max_forward,
                self.min_lin_vel_forward,
                self.max_lin_vel_forward,
            )
        elif self.marker_distance <= self.distance_max_reverse:
            lin_cmd = -translate(
                self.marker_distance,
                self.distance_min_reverse,
                self.distance_max_reverse,
                self.max_lin_vel_reverse,
                self.min_lin_vel_reverse,
            )
        else:
            lin_cmd = 0.0
            move_arm = MoveArm()
            move_arm.movearm()
            return

        self.twist_cmd.angular.z = dir * ang_cmd
        self.twist_cmd.linear.x = lin_cmd

    def update_marker_angle_distance(self):
        if self.last_marker_position:
            position_x = self.last_marker_position.x - self.odom_position.x
            position_y = self.last_marker_position.y - self.odom_position.y
            self.marker_angle = math.atan(position_y / position_x) - self.odom_yaw
            self.marker_distance = math.sqrt(
                position_x * position_x + position_y * position_y
            )

    def callback_ar_pose(self, msg):
        for marker in msg.markers:
            if marker.id != self.follow_id:
                continue
            if marker.header.stamp < self.last_marker_ts:
                rospy.logwarn_throttle(
                    3.0, "Got marker position with an older timestamp"
                )
                continue

            self.last_marker_ts = marker.header.stamp
            self.last_marker_position = marker.pose.pose.position
            self.odom_position = Vector3()
            self.odom_yaw = 0.0

            self.update_marker_angle_distance()

    def callback_wheel_odom(self, msg):
        if self.last_odom_ts:
            start_ts = max(self.last_odom_ts, self.last_marker_ts)

            end_ts = msg.header.stamp
            if end_ts < start_ts:
                rospy.logwarn(
                    "Reveived odometry has timestamp older than last marker position"
                )

            step_duration = (end_ts - start_ts).to_sec()

            # Integrate the velocity using rectangular rule
            self.odom_yaw += msg.twist.twist.angular.z * step_duration
            self.odom_position.x += (
                msg.twist.twist.linear.x * math.cos(self.odom_yaw) * step_duration
            )
            self.odom_position.y += (
                msg.twist.twist.linear.x * math.sin(self.odom_yaw) * step_duration
            )

            self.update_marker_angle_distance()

        self.last_odom_ts = msg.header.stamp

if __name__ == "__main__":
    rospy.init_node("follow_ar_tag")
    ar_tag_follower = ARTagFollower()
    ar_tag_follower.run()
