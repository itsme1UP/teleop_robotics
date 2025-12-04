#!/usr/bin/env python3

# motion_mapper.py

def map_hand_to_robot(hand_msg, scale=0.5):
    """Maps Leap Motion hand X position to UR10e forward/backward"""
    # This is a simple mapping: hand x-axis value [-1,1] -> robot displacement
    # hand_msg.axes[0] is assumed to be X-axis of hand
    try:
        hand_x = hand_msg.axes[0]
        robot_x = hand_x * scale
        return robot_x
    except Exception as e:
        print('Error mapping hand:', e)
        return 0.0
