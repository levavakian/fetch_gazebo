#!/usr/bin/env python

import signal
import copy
import actionlib
import rospy

from math import sin, cos
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Twist, Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from fetch_move_base_msgs.msg import *
from fetch_move_base_msgs.srv import *

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        self.finished = False

    def goto(self, x, y, theta, frame="map", timeout=rospy.Duration()):
        self.finished = False

        move_goal = MoveBaseGoal()
        target_pose = PoseStamped()

        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.orientation.z = sin(theta/2.0)
        target_pose.pose.orientation.w = cos(theta/2.0)
        target_pose.header.frame_id = frame
        target_pose.header.stamp = rospy.Time.now()

        move_goal.target_poses.append(target_pose)

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.finished = self.client.wait_for_result(timeout)

        stats = self.client.get_result()
        print self.client.get_goal_status_text()
        return stats

    def wait_for_server(self, timeout = rospy.Duration()):
        return self.client.wait_for_server(timeout)

class TimingTest(object):

    def __init__(self):
        self.succeeded = 0
        self.failed = 0
        self.avg_pdur = 0
        self.avg_cdur = 0
        self.avg_rdur = 0

    def run_trials(self, x0, y0, theta0, poses, collision_type):
        self.clear_costmaps()
        self.set_pos_gazebo(x0,y0,theta0)
        self.set_pos_ros(x0,y0,theta0)
        self.clear_costmaps()

        i = 0
        while i < len(poses):
            print "Target number: " + str(i)
            pose = poses[i] 

            cms = False
            while not cms:
                cms = self.collision_model_service(collision_type)

            move_base = MoveBaseClient()
            rospy.sleep(5)
            connected = move_base.wait_for_server(rospy.Duration(5.0))

            stats = move_base.goto(pose.x,pose.y,pose.z, timeout = rospy.Duration(120.0))

            if not connected:
                print "Status: did not connect"
                continue

            if not move_base.finished:
                print "Status: did not finish"
                continue

            if (move_base.client.get_state() != GoalStatus.SUCCEEDED) and (move_base.client.get_state() != GoalStatus.ABORTED):
                print "Status: indeterminate result"
                continue

            if move_base.client.get_state() == GoalStatus.ABORTED:
                self.failed+=1
                i+=1
                print "Status: failed"
                continue

            print "Status: succeeded"
            self.update_statistics(stats)
            print ""
            i += 1

        self.print_final_statistics()

    def set_pos_gazebo(self, x, y, theta, frame="world", model="freight"):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_pos = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            pos = ModelState()

            # Set the name and frame
            pos.model_name = model
            pos.reference_frame = frame

            # Set the position
            pos.pose.position.x = x
            pos.pose.position.y = y

            # Set the orientation 
            pos.pose.orientation.z = sin(theta/2)
            pos.pose.orientation.w = cos(theta/2)

            resp = set_pos(pos)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.sleep(1)

    def set_pos_ros(self, x, y, theta, frame="map"):
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = frame

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y

        pose.pose.pose.orientation.z = sin(theta/2)
        pose.pose.pose.orientation.w = cos(theta/2)
        pose.pose.covariance = [.25, 0, 0, 0, 0, 0, 
                                0, .25, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, 0, 
                                0, 0, 0, 0, 0, .0685]

        pub.publish(pose)

        self.spin_in_place()

        # This is awful but required for the latched topic to be read before we do other things
        rospy.sleep(1)

    def spin_in_place(self):
        twist = Twist()
        twist.angular.z = 1
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        t0 = rospy.get_rostime()

        spin_time = 7.5
        spin = True
        while spin:
            pub.publish(twist)
            t1 = rospy.get_rostime()
            if (t1-t0).to_sec() > spin_time:
                spin = False

    def update_statistics(self, stats):
        success = stats.statistics.summary.success
        pduration = stats.statistics.summary.planning_duration.to_sec()
        cduration = stats.statistics.summary.controller_duration.to_sec()
        rduration = stats.statistics.summary.recovery_duration.to_sec()
        print("Arrived at target: " + str(success))
        print("Planning duration: " + str(pduration) + "sec")
        print("Controls duration: " + str(cduration) + "sec")
        print("Recovery duration: " + str(rduration) + "sec")

        if success:
            self.avg_pdur += pduration
            self.avg_cdur += cduration
            self.avg_rdur += rduration

        if success:
            self.succeeded+=1

        if not success:
            self.failed+=1


    def print_final_statistics(self):
        print("Number of trials succeeded: " + str(self.succeeded))
        print("Number of trials failed: " + str(self.failed))

        if self.succeeded == 0:
            return

        print("Average planning duration: " + str(self.avg_pdur/self.succeeded))
        print("Average controls duration: " + str(self.avg_cdur/self.succeeded))
        print("Average recovery duration: " + str(self.avg_rdur/self.succeeded))

    def clear_costmaps(self):
        rospy.wait_for_service('move_base/clear_costmaps')
        cc = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        try:
            resp = cc()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def collision_model_service(self, collision_type):
            print("Setting collision model: " + collision_type)
            model = CollisionModelMsg()

            if collision_type == "standard":
                model.collision_model = "simple"
                model.footprint_type = "radius"
                model.robot_radius = .3
                model.inflation_factor = 2.5
            elif collision_type == "standard_large":
                model.collision_model = "simple"
                model.footprint_type = "radius"
                model.robot_radius = .6
                model.inflation_factor = 2.5
            elif collision_type == "long":
                shape = CollisionShapeMsg()
                shape.name = "rectangle"
                shape.p1 = Point2D(-0.5, 0.3)
                shape.p2 = Point2D(0.5, 0.3)
                shape.p3 = Point2D(-0.5, -0.3)

                model.collision_model = "primitives"
                model.footprint_type = "points"
                model.footprint = "[[-0.5, 0.3], [0.5, 0.3], [0.5, -0.3], [-0.5, -0.3]]"
                model.inflation_factor = 1.0
                model.components.append(shape)
            elif collision_type == "wide":
                shape = CollisionShapeMsg()
                shape.name = "rectangle"
                shape.p1 = Point2D(-0.3, 0.5)
                shape.p2 = Point2D(0.3, 0.5)
                shape.p3 = Point2D(-0.3, -0.5)

                model.collision_model = "primitives"
                model.footprint_type = "points"
                model.footprint = "[[-0.3, 0.5], [0.3, 0.5], [0.3, -0.5], [-0.3, -0.5]]"
                model.inflation_factor = 1.0
                model.components.append(shape)
            else:
                print("Did not recognize collision type, assuming standard")
                model.collision_model = "simple"
                model.footprint_type = "radius"
                model.robot_radius = .3
                model.inflation_factor = 2.5

            rospy.wait_for_service('/move_base/set_collision_model')
            try:
                cms = rospy.ServiceProxy('/move_base/set_collision_model', SetCollisionModel)
                cms(model)
                return True
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return False

if __name__ == "__main__":

  rospy.init_node("planner_timing_demo")
  while not rospy.Time.now():
      pass

  test = TimingTest()
  poses = [Vector3(-7.7, 6, .9),
           Vector3(100,100,0),
           Vector3(-7.7, -6, -.9),
           Vector3(-3, 3, .6),
           Vector3(-1.5, -4, 1.65), 
           Vector3(3, -3, -.6),
           Vector3(5, -4, 0),
           Vector3(5, 1, .2)]

  collision_type = rospy.get_param('collision_type')
  test.run_trials(4, 0, 0, poses, collision_type)
