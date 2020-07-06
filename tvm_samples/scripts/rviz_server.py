#! /usr/bin/env python

import numpy as np

import rospy
from tf import transformations
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg \
    import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server \
    import InteractiveMarkerServer


class RvizServer(object):
    def __init__(self):
        self.world_frame_id = "world"

        self.setupInteractiveMarker()

        self.pose_pub = rospy.Publisher(
            "interactive_marker_pose", PoseStamped, queue_size=1)

    def setupInteractiveMarker(self):
        # make server
        self.im_server = InteractiveMarkerServer("interactive_marker_server")

        # add start
        self._addInteractiveMarker6Dof(
            "pose",
            Pose(position=Point(0.,0.,1.), orientation=Quaternion(0.,0.,0.,1.))
        )

        # apply to server
        self.im_server.applyChanges()

    def _addInteractiveMarker6Dof(self, name, initial_pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.world_frame_id
        int_marker.name = name
        int_marker.pose = initial_pose
        int_marker.scale = 0.3

        # rotate_x control
        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w \
            = transformations.quaternion_from_euler(0, 0, 0)
        int_marker.controls.append(control)

        # rotate_y control
        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w \
            = transformations.quaternion_from_euler(0, 0, np.pi/2)
        int_marker.controls.append(control)

        # rotate_z control
        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w \
            = transformations.quaternion_from_euler(0, np.pi/2, 0)
        int_marker.controls.append(control)

        # move_x control
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w \
            = transformations.quaternion_from_euler(0, 0, 0)
        int_marker.controls.append(control)

        # move_y control
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w \
            = transformations.quaternion_from_euler(0, 0, np.pi/2)
        int_marker.controls.append(control)

        # move_z control
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w \
            = transformations.quaternion_from_euler(0, np.pi/2, 0)
        int_marker.controls.append(control)

        self.im_server.insert(int_marker, self.interactivemarkerFeedback)

    def interactivemarkerFeedback(self, feedback):
        pose_st_msg = PoseStamped()
        pose_st_msg.header = feedback.header
        pose_st_msg.pose = feedback.pose
        self.pose_pub.publish(pose_st_msg)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    # setup
    rospy.init_node("rviz_server", anonymous=False)

    # instantiate visualizer
    server = RvizServer()

    server.spin()

