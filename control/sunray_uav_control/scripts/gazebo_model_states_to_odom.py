#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


def trim_leading_slash(s):
    if s.startswith("/"):
        return s[1:]
    return s


def resolve_uav_ns():
    if rospy.has_param("/uav_ns"):
        return trim_leading_slash(str(rospy.get_param("/uav_ns")))

    if rospy.has_param("/uav_name") and rospy.has_param("/uav_id"):
        name = str(rospy.get_param("/uav_name"))
        uid = str(rospy.get_param("/uav_id"))
        return trim_leading_slash(name + uid)
    return ""


class GazeboModelStatesToOdom:
    def __init__(self):
        self.uav_ns = resolve_uav_ns()
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))

        self.use_model_states = rospy.get_param("~use_model_states", False)
        default_model = self.uav_ns if self.uav_ns else "uav1"
        self.model_name = rospy.get_param("~model_name", default_model)
        default_input = ("/" + self.uav_ns + "/sunray/gazebo_pose") if self.uav_ns else "/sunray/gazebo_pose"
        self.input_topic = rospy.get_param("~input_topic", default_input)
        default_out = ("/" + self.uav_ns + "/sunray_odom_in") if self.uav_ns else "/sunray_odom_in"
        self.output_topic = rospy.get_param("~output_topic", default_out)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "body")

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        if self.use_model_states:
            self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb_model_states, queue_size=20)
            rospy.loginfo(
                "[gazebo_model_states_to_odom] mode=model_states ns=%s model=%s out=%s",
                self.uav_ns,
                self.model_name,
                self.output_topic,
            )
        else:
            self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb_odom, queue_size=20)
            rospy.loginfo(
                "[gazebo_model_states_to_odom] mode=odom ns=%s in=%s out=%s use_sim_time=%s",
                self.uav_ns,
                self.input_topic,
                self.output_topic,
                str(self.use_sim_time),
            )

    def cb_model_states(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(
                2.0,
                "[gazebo_model_states_to_odom] model '%s' not found in /gazebo/model_states",
                self.model_name,
            )
            return

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = msg.pose[idx]
        odom.twist.twist = msg.twist[idx]
        self.pub.publish(odom)

    def cb_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        if not self.use_sim_time:
            odom.header.stamp = rospy.Time.now()
        elif odom.header.stamp.to_sec() <= 0.0:
            odom.header.stamp = rospy.Time.now()
        odom.child_frame_id = msg.child_frame_id if msg.child_frame_id else self.child_frame_id
        if not odom.header.frame_id:
            odom.header.frame_id = self.frame_id
        elif odom.header.frame_id.startswith("/"):
            odom.header.frame_id = odom.header.frame_id[1:]
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("gazebo_model_states_to_odom")
    GazeboModelStatesToOdom()
    rospy.spin()
