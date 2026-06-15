#!/usr/bin/env python3
"""Minimal rospy example for TAKEOFF -> HOVER -> LAND."""

import sys

import rospy

from sunray_msgs.msg import UAVControlCMD, UAVControlState


class TakeoffLandDemo:
    def __init__(self):
        self.control_state = UAVControlState()
        self.use_private_agent_key = rospy.get_param("~use_private_agent_key", False)
        self.agent_name = self._get_agent_name()
        self.agent_id = self._get_agent_id()
        self.agent_key = "/{}{}".format(self.agent_name, self.agent_id)

        self.control_state_sub = rospy.Subscriber(
            self.agent_key + "/sunray/uav_control/control_state",
            UAVControlState,
            self._control_state_cb,
            queue_size=10,
        )
        self.control_cmd_pub = rospy.Publisher(
            self.agent_key + "/sunray/uav_control/control_cmd",
            UAVControlCMD,
            queue_size=1,
        )

    def _get_agent_name(self):
        if self.use_private_agent_key:
            return rospy.get_param("~agent_name", "uav")
        return rospy.get_param("/agent_name", "uav")

    def _get_agent_id(self):
        if self.use_private_agent_key:
            return int(rospy.get_param("~agent_id", 1))
        return int(rospy.get_param("/agent_id", 1))

    def _control_state_cb(self, msg):
        self.control_state = msg

    def _publish_cmd_once(self, control_cmd):
        msg = UAVControlCMD()
        msg.header.stamp = rospy.Time.now()
        msg.cmd_source = UAVControlCMD.EXAMPLE_DEMO
        msg.control_cmd = control_cmd
        self.control_cmd_pub.publish(msg)

    def _wait_for_state(self, target_state, state_name):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown() and self.control_state.control_state != target_state:
            rospy.loginfo_throttle(5.0, "waiting for %s...", state_name)
            rate.sleep()
        return not rospy.is_shutdown()

    def run(self):
        rospy.loginfo("takeoff_land.py uses agent_key=%s", self.agent_key)

        if not self._wait_for_state(UAVControlState.INIT, "INIT"):
            return

        rospy.loginfo("sending TAKEOFF command")
        self._publish_cmd_once(UAVControlCMD.TAKEOFF)

        if not self._wait_for_state(UAVControlState.HOVER, "HOVER"):
            return

        rospy.loginfo("uav takeoff succeeded, hovering for 5 s before LAND")
        rospy.sleep(5.0)

        rospy.loginfo("sending LAND command")
        self._publish_cmd_once(UAVControlCMD.LAND)
        rospy.loginfo("sent LAND command and [takeoff_land.py] demo finished")


def main():
    rospy.init_node("takeoff_land_py_node")
    try:
        TakeoffLandDemo().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:  # Keep beginner demo failures visible in roslaunch output.
        rospy.logerr("takeoff_land.py failed: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
