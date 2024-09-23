from geometry_msgs.msg import Twist

from context import Context
from state_machine.state import State
from state import DoneState, FailState
import numpy as np


class TagSeekState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        DISTANCE_TOLERANCE = 0.995
        ANGULAR_TOLERANCE = 0.3
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)
        tag = context.env.get_fid_data()

        # TODO: if we don't have a tag: go to the FailState
        if tag is None:
            return FailState()
    
        tag_size = 0.2 * 0.2
        image_width = 640
        fov = np.pi / 3.0
        focal_length = image_width / (2.0 * np.tan(fov/2.0))
        distance = focal_length * tag_size / tag.closeness_metric 
        d_x_from_center = tag.x - image_width/2.0
        angular_offset = d_x_from_center/image_width * fov

        # TODO: if we are within angular and distance tolerances: go to DoneState
        if distance <= DISTANCE_TOLERANCE and angular_offset <= ANGULAR_TOLERANCE:
            return DoneState()

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        # Create the Twist command
        twist_cmd = Twist()
        
        if distance > DISTANCE_TOLERANCE:
            twist_cmd.linear.x = min(0.5, distance)  # Move forward with a speed proportional to the distance
        else:
            twist_cmd.linear.x = 0  # Stop moving forward when within tolerance

        # Set angular velocity proportional to angular offset (turn to face the tag)
        if abs(angular_offset) > ANGULAR_TOLERANCE:
            twist_cmd.angular.z = -angular_offset * 1.0  # Adjust the scale factor (1.0) for how fast the rover should turn
        else:
            twist_cmd.angular.z = 0  # Stop turning when within tolerance

        # TODO: send Twist command to rover
        context.rover.send_drive_command(Twist())

        # TODO: stay in the TagSeekState (with outcome "working")
        return self

