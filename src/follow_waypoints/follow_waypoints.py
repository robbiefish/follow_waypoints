#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped, QuaternionStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from tf import TransformListener
import tf
import tf.transformations
import math
import rospkg
import csv
import time
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32

# change Pose to the correct frame 
def changePose(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()


#Path for saving and retreiving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
waypoints = []

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)

        # Get the global frame ID for the planner
        self.global_frame_id = rospy.get_param('/move_base/global_costmap/global_frame', 'map')

        # Subscribe to the plan so we can see how close to complete we are (this is a big hack, but seems to work)
        self._current_plan = None
        self.plan_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self._handle_plan)

        # Publish a polygon containing the waypoints
        self.waypoints_polygon_pub = rospy.Publisher('/waypoints_polygon', PolygonStamped)

    def execute(self, userdata):
        global waypoints

        # Massage the data a little bit before processing it
        polygon = PolygonStamped()
        polygon.header.frame_id = self.frame_id
        for waypoint in waypoints:
            if self.tf.canTransform(self.frame_id, self.global_frame_id, rospy.get_rostime()):
                # Make sure we are only sending yaw to the planner
                q = QuaternionStamped()
                q.header.frame_id = self.frame_id
                q.quaternion = waypoint.pose.pose.orientation
                q = self.tf.transformQuaternion(self.global_frame_id, q)
                r, p, y = tf.transformations.euler_from_quaternion((q.quaternion.x, q.quaternion.y, q.quaternion.z, q.quaternion.w))
                q_nums = tf.transformations.quaternion_from_euler(0, 0, y)
                q.quaternion.x = q_nums[0]
                q.quaternion.y = q_nums[1]
                q.quaternion.z = q_nums[2]
                q.quaternion.w = q_nums[3]
                q = self.tf.transformQuaternion(self.frame_id, q)
                waypoint.pose.pose.orientation = q.quaternion

                # Make sure that z is zeroed at the global frame
                p = PointStamped()
                p.header.frame_id = self.frame_id
                p.point = waypoint.pose.pose.position
                p = self.tf.transformPoint(self.global_frame_id, p)
                p.point.z = 0
                p = self.tf.transformPoint(self.frame_id, p)
                waypoint.pose.pose.position = p.point

            # Accumulate the waypoints into a polygon
            polygon.polygon.points.append(waypoint.pose.pose.position)

        # Publish the polygon
        polygon.header.stamp = rospy.get_rostime()
        self.waypoints_polygon_pub.publish(polygon)

        # Execute waypoints each in sequence
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break

            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)

            # Wait until we are close enough to the goal
            while not self.client.wait_for_result(rospy.Duration(0, 1e+8)):
                # If the current goal is short enough, we can move on
                if len(self._current_plan.poses) < 18:
                    break

        return 'success'

    def _handle_plan(self, data):
        self._current_plan = data

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Subscribe to pose message to get new waypoints
        self.addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True
            with open(output_file_path, 'w') as file:
                for current_pose in waypoints:
                    file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
            rospy.loginfo('poses written to '+ output_file_path)	
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        self.start_journey_bool = False

        # Start thread to listen start_jorney 
        # for loading the saved poses from follow_waypoints/saved_path/poses.csv
        def wait_for_start_journey():
            """thread worker function"""
            data_from_start_journey = rospy.wait_for_message('start_journey', Empty)
            rospy.loginfo('Recieved path READY start_journey')
            with open(output_file_path, 'r') as file:
                reader = csv.reader(file, delimiter = ',')
                for row in reader:
                    print (row)
                    current_pose = PoseWithCovarianceStamped() 
                    current_pose.pose.pose.position.x     =    float(row[0])
                    current_pose.pose.pose.position.y     =    float(row[1])
                    current_pose.pose.pose.position.z     =    float(row[2])
                    current_pose.pose.pose.orientation.x = float(row[3])
                    current_pose.pose.pose.orientation.y = float(row[4])
                    current_pose.pose.pose.orientation.z = float(row[5])
                    current_pose.pose.pose.orientation.w = float(row[6])
                    waypoints.append(current_pose)
                    self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            self.start_journey_bool = True
            
            
        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()

        topic = self.addpose_topic;
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        rospy.loginfo("OR")
        rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")


        # Wait for published waypoints or saved path  loaded
        while (not self.path_ready and not self.start_journey_bool):
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                continue  # no new waypoint within timeout, looping...
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(changePose(pose, "map"))
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()
