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
        print("Current End Effector Pose:")
        print(current_pose)
        self.grasp_pose=PoseStamped()
        self.movearm()
        rospy.spin()
        
    def callback(self, data):
        self.data = data  # Store data
        for marker in data.markers:
            self.position = marker.pose.pose.position
            self.orientation = marker.pose.pose.orientation
            self.frame_id = marker.header.frame_id
            self.pose = marker.pose.pose
            
            #It displays the marker's detatils continouesly 
            #print("Marker ID:", marker.id)
            #print("Position :", self.position)
            #print("Orientation :", self.orientation)

    def movearm(self):
        time.sleep(5)
        print()
        print("AR-Tag Pose:")
        print(self.pose)
        # print("AR-Tag Orientation :", self.orientation)
        if self.position is not None:

            self.grasp_pose.header.frame_id = "base_footprint"
            self.grasp_pose.pose.position.x = 0.2502012225613189
            self.grasp_pose.pose.position.y = -0.0032156971548337535
            self.grasp_pose.pose.position.z = 0.17652794280001272
            
            self.grasp_pose.pose.orientation.x = 0.007705798279333313
            self.grasp_pose.pose.orientation.y = 0.7534836526780807
            self.grasp_pose.pose.orientation.z = -0.006723028973121577
            self.grasp_pose.pose.orientation.w = 0.657387105670017
            self.arm_group.set_pose_target(self.grasp_pose)

            #plan = self.arm_group.plan()
            #self.arm_group.execute(plan[1])

if __name__ == '__main__':
    move_arm = MoveArm()
