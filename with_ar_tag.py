#!/usr/bin/env python
import sys
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
import moveit_commander
from moveit_commander import PlanningSceneInterface
import time
import copy
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class MoveArm():
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        self.position = None
        self.orientation = None
        self.data = None  # Store data as an instance variable
        moveit_commander.roscpp_initialize(sys.argv)
        scene = PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        self.arm_group=moveit_commander.MoveGroupCommander("arm")
        self.gripper_group=moveit_commander.MoveGroupCommander("gripper")
        current_state = robot.get_current_state()
        current_pose = self.arm_group.get_current_pose().pose

        # Print the current pose
        # print("Current End Effector Pose:")
        # print(current_pose)
        
        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
        self.joint1_pub = rospy.Publisher('/gripper_finger1_joint/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/gripper_finger2_joint/command', Float64, queue_size=10)
        self.arm_group.set_planning_time(10)
        self.grasp_pose=PoseStamped()
        self.gripper_pose=PoseStamped()
        # Initialize ROS node
        self.movearm()
        rospy.spin()
        
        
    def callback(self, data):
        self.data = data  # Store data
        for marker in data.markers:
            self.position = marker.pose.pose.position
            self.orientation = marker.pose.pose.orientation
            self.frame_id = marker.header.frame_id

            # print("Marker ID:", marker.id)
            # print("Position :", self.position)
            # print("Orientation :", self.orientation)

    def control_joints(self, joint_positions):
        # Publish joint positions
        self.joint1_pub.publish(joint_positions[0])
        self.joint2_pub.publish(joint_positions[1])


    def movearm(self):
        time.sleep(3)
        if self.position is not None:

            joint_positions = [0.016, 0.016] 
            self.control_joints(joint_positions)

            print("AR TAG Position :", self.position)
            print()
            self.grasp_pose.header.frame_id = "base_footprint"
            # position is from marker
            self.grasp_pose.pose.position.x = self.position.x
            self.grasp_pose.pose.position.y = self.position.y + 0.0129
            self.grasp_pose.pose.position.z = self.position.z + 0.0247

            self.grasp_pose.pose.orientation.x = -0.03443147641271983	#self.orientation.x
            self.grasp_pose.pose.orientation.y = 0.7476661826604609		#self.orientation.y
            self.grasp_pose.pose.orientation.z = 0.03050846109213684	#self.orientation.z
            self.grasp_pose.pose.orientation.w = 0.662479423484054 		#self.orientation.w
            self.arm_group.set_pose_target(self.grasp_pose)
            print("ROBOTIC ARM Position :", self.grasp_pose)

            plan = self.arm_group.plan()

            self.arm_group.execute(plan[1])
            time.sleep(2)

            joint_positions = [0.01, 0.01] 
            self.control_joints(joint_positions)
            time.sleep(2)

            self.arm_group.set_named_target("resting")
            self.arm_group.go()

if __name__ == '__main__':
    move_arm = MoveArm()
