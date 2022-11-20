#!/usr/bin/env python

import threading
import random
import rospy
# Import constant name defined to structure the architecture.
from arch_skeleton import architecture_name_mapper as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from arch_skeleton.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE


# The node manager class. (HA I SERVER DEI DUE SERVIZI NECESSARI a posizione (per controller e planner)
# e pubblica variazioni batteria)!!
# This class defines ---> two services <----- to get and set the current
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    def __init__(self):
        # Initialise this node.
        rospy.init_node('robot_state_manager')
        # Initialise robot position.
        self._pose = None
        # Initialise battery level.
        self._battery_low = False
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [25.0, 45.0, 20.0,35.0])# [15.0, 35.0, 5.0,10.0] The delay between changes of battery levels, i.e., high/low.
                                                                                             # It should be a list `[min_time, max_time]`, and the battery level change
                                                                                             # will occur after a random number of seconds within such an interval.
            log_msg = (f'Random-based battery low notification active: the battery change state (i.e., low/high) with a '
                       f'delay in the range of [{self._random_battery_time[0]}, {self._random_battery_time[1]}) seconds.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose_callback)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose_callback)

        # Start publisher on a separate thread.
        th = threading.Thread(target=self._is_battery_low)
        th.start()


    # The `robot/set_pose` service implementation.
    # The `request` input parameter is the current robot pose to be set,
    # as given by the client. This server returns an empty `response`.
    def set_pose_callback(self, request):
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Uncomment to Log information.
            # self._print_info(f'Set current robot position through `{anm.SERVER_SET_POSE}` '
            #                  f'as ({self._pose.x}, {self._pose.y}).')
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty. Thus, it is not used.
    # The `response` returned to the client contains the current robot pose.
    def get_pose_callback(self, request):
        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        # else: uncomment to log info
        #     log_msg = f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
        #     self._print_info(log_msg)
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    # Publish changes of battery levels. This method runs on a separate thread.
    def _is_battery_low(self):
        # Define a `latched` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self._random_battery_notifier(publisher)
        # else:
            # Publish battery level changes through a keyboard-based interface.
            # self._manual_battery_notifier(publisher)

    # Publish when the battery change state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def _random_battery_notifier(self, publisher):
        discharge_delay = 0  # Initialised to 0 just for logging purposes.
        recharging_delay = 0
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                log_msg = f'Robot got low battery after {discharge_delay} seconds.' #devo ricaricare
                recharging_delay = random.uniform(self._random_battery_time[2], self._random_battery_time[3])
                self._print_info(log_msg)
                # Wait for simulate battery recharging
                rospy.sleep(recharging_delay)
            else:
                log_msg = f'Robot got a fully charged battery after {recharging_delay} seconds.'
                discharge_delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
                self._print_info(log_msg)
                # Wait for simulate battery usage.
                rospy.sleep(discharge_delay)
            # Change battery state.
            self._battery_low = not self._battery_low

            publisher.publish(Bool(self._battery_low))

    # Print logging only when random testing is active.
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def _print_info(self, msg):
        if self._randomness:
            rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
