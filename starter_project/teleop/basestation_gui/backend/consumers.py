import json
import traceback
from typing import Any, Type

import yaml
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
import tf2_ros
import numpy as np
from backend.drive_controls import send_joystick_twist
from backend.input import DeviceInputs
from mrover.msg import WheelCmd

rospy.init_node("teleoperation", disable_signals=True)


class GUIConsumer(JsonWebsocketConsumer):
    subscribers: list[rospy.Subscriber] = []

    def connect(self) -> None:
        self.accept()

        ########################################################################################
        # Use self.forward_ros_topic when you want to get data from a ROS topic to a GUI
        # without needing any modifications done on it. For instance, reading motor output.
        ########################################################################################
        self.forward_ros_topic("/wheel_cmd", WheelCmd, "wheel_cmd")

    def disconnect(self, close_code) -> None:
        for subscriber in self.subscribers:
            subscriber.unregister()

    def forward_ros_topic(self, topic_name: str, topic_type: Type, gui_msg_type: str) -> None:
        """
        Subscribes to a ROS topic and forwards messages to the GUI as JSON

        @param topic_name:      ROS topic name
        @param topic_type:      ROS message type
        @param gui_msg_type:    String to identify the message type in the GUI
        """

        def callback(ros_message: Any):
            # Formatting a ROS message as a string outputs YAML
            # Parse it back into a dictionary, so we can send it as JSON
            self.send_message_as_json({"type": gui_msg_type, **yaml.safe_load(str(ros_message))})

        self.subscribers.append(rospy.Subscriber(topic_name, topic_type, callback))

    def send_message_as_json(self, msg: dict):
        try:
            self.send(text_data=json.dumps(msg))
        except Exception as e:
            rospy.logwarn(f"Failed to send message: {e}")


    def receive(self, text_data=None, bytes_data=None, **kwargs) -> None:
        """
        Callback function when a message is received in the Websocket

        @param text_data:   Stringfied JSON message
        """

        if text_data is None:
            rospy.logwarn("Expecting text but received binary on GUI websocket...")
            return

        try:
            message = json.loads(text_data)
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to decode JSON: {e}")
            return

        try:
            match message:
                case {
                    "type": "joystick",
                    "axes": axes,
                    "buttons": buttons,
                }:
                    device_input = DeviceInputs(axes, buttons)
                    send_joystick_twist(device_input)
                case _:
                    rospy.logwarn(f"Unhandled message: {message}")

        except:
            rospy.logerr(f"Failed to handle message: {message}")
            rospy.logerr(traceback.format_exc())