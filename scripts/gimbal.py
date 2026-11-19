#!/usr/bin/env python3
"""Point the Iris gimbal camera using Gazebo transport (no ROS).

The Iris model mounts a three-axis gimbal under the body
(models/gimbal_small_3d/model.sdf). Each axis is driven by a
gz::sim::systems::JointPositionController that listens on a plain Gazebo
transport topic:

    /gimbal/cmd_pitch   gz.msgs.Double   radians, positive = nose down
    /gimbal/cmd_roll    gz.msgs.Double   radians
    /gimbal/cmd_yaw     gz.msgs.Double   radians

That is why this project needs no ROS: the command is one protobuf message
on a topic that gz-transport already carries. `gz topic -l` lists the same
topics, and `gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p "data: 0.7"`
does the same thing from a shell.

Why the camera is pointed down
    The stock camera looks along the horizon. At 5 m altitude a level frame
    is roughly half sky and half runway compressed into a few rows near the
    horizon, where ground features have almost no parallax. Pitching down
    by PITCH_DOWN_RAD puts the ground plane across most of the image, which
    is what makes a feature-based VIO work in this world.
"""

import time

import gz.transport13 as gzt
from gz.msgs10.double_pb2 import Double

# The pitch joint's command topic, named by the model's
# JointPositionController. Only pitch is driven here; roll and yaw stay at
# whatever the model loaded with, and the VIO assumes exactly that.
PITCH_TOPIC = "/gimbal/cmd_pitch"

# 0.7 rad = 40 degrees. Measured on this model: the horizon sits near the
# top of the 640x480 frame, and the ground is visible from directly below
# the aircraft out to about 30 m ahead. 0 leaves the stock level view;
# larger than about 1.2 rad starts to see the underside of the airframe.
PITCH_DOWN_RAD = 0.7

# The controller subscribes after the model loads, and a single publish to
# a topic nobody is listening to yet is lost. Republishing for this long at
# PUBLISH_RATE_HZ guarantees delivery without needing a subscriber-count
# API.
PUBLISH_DURATION_S = 0.5
PUBLISH_RATE_HZ = 20.0


def set_pitch(radians, topic=PITCH_TOPIC,
              duration_s=PUBLISH_DURATION_S, rate_hz=PUBLISH_RATE_HZ):
    """Publish a pitch target and return after the gimbal has had time to move.

    radians is the joint target, not a rate: the joint position controller
    holds it until a new target arrives. Negative values tilt the camera up;
    positive values tilt it down (verified against captured frames).
    """
    node = gzt.Node()
    # advertise() returns the publisher object; Node itself has no
    # publish(). The publisher is kept alive for the whole call so the
    # message is not dropped when the local object goes out of scope.
    publisher = node.advertise(topic, Double)
    message = Double()
    message.data = radians
    sent = 0
    deadline = time.time() + duration_s
    period = 1.0 / rate_hz
    while time.time() < deadline:
        if publisher.publish(message):
            sent += 1
        time.sleep(period)
    if sent == 0:
        raise RuntimeError(
            f"Could not publish to {topic}. Is Gazebo running? "
            "scripts/run_sim.sh starts it."
        )
    # Wait for the joint controller to slew to the target. p_gain is 2 in
    # the gimbal model, so 0.7 rad is reached in well under a second.
    time.sleep(0.5)
    print(f"Gimbal pitch set to {radians:+.2f} rad on {topic} ({sent} messages)")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Point the Iris gimbal camera")
    parser.add_argument("radians", type=float, nargs="?", default=PITCH_DOWN_RAD,
                        help="pitch target in radians, positive = down")
    set_pitch(parser.parse_args().radians)
