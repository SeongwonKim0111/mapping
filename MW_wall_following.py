#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Manhattan Wall Following Demo
#
# This is a simplified wall following state machine adapted for Manhattan world
# constraints (i.e. walls are assumed to be aligned with the cardinal directions).
# The drone flies forward along a cardinal axis while keeping a reference distance
# from the wall on its chosen side. When a wall is detected in the front (e.g. at a corner)
# the drone turns by 90 degrees (clockwise or anticlockwise depending on the wall-following side)
# so that its heading “snaps” to one of 0, ±90, 180 degrees.
#
# This code is inspired by the Crazyflie wall following demo code.
#
# Author: Your Name
# License: GNU GPL v3

import math
import time
from enum import Enum

class ManhattanWallFollower:
    """
    A simple Manhattan-constrained wall following state machine.
    
    The drone is assumed to fly in a Manhattan world where walls are aligned along
    the x and y axes. In this example the drone maintains a reference distance from the wall
    on a given side (either "LEFT" or "RIGHT"). When a front obstacle is detected, the drone 
    performs a 90° corner turn so that its heading is “snapped” to a cardinal direction.
    """
    
    class State(Enum):
        FORWARD = 0          # Fly forward while checking sensors.
        CORNER_TURN = 1      # Perform a 90° turn to avoid a front obstacle.
        FOLLOW_WALL = 2      # Fly forward along the wall.
        HOVER = 3            # Stop/hover.

    def __init__(self, reference_distance=0.5, max_forward_speed=0.3,
                 turn_rate=math.pi/8, wall_following_direction="LEFT", ranger_buffer=0.1):
        """
        Initialize the ManhattanWallFollower.
        
        :param reference_distance: desired distance from the wall (m)
        :param max_forward_speed: forward speed (m/s)
        :param turn_rate: constant turn rate during corner turns (rad/s)
        :param wall_following_direction: which side to follow the wall ("LEFT" or "RIGHT")
        :param ranger_buffer: sensor buffer (m) to decide when a wall is too close
        """
        self.reference_distance = reference_distance
        self.max_forward_speed = max_forward_speed
        self.turn_rate = turn_rate
        self.wall_following_direction = wall_following_direction.upper()
        self.ranger_buffer = ranger_buffer
        self.state = ManhattanWallFollower.State.FORWARD
        self.desired_heading = None  # Set during turns

    def wrap_to_pi(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def snap_to_90(self, angle):
        """
        Snap an angle (in radians) to the nearest multiple of 90°.
        """
        angle = self.wrap_to_pi(angle)
        # 90° in radians is pi/2.
        return round(angle / (math.pi / 2)) * (math.pi / 2)

    def update(self, front_range, side_range, current_heading, time_now):
        """
        Update the wall-following state machine given sensor inputs.
        
        :param front_range: measured distance in front (m)
        :param side_range: measured distance on the side (m) used for wall following
        :param current_heading: current yaw (rad)
        :param time_now: current time in seconds (unused here, but can be used for state timing)
        :return: (vx, vy, yaw_rate, state) where
            vx: forward velocity (m/s)
            vy: lateral (sideways) velocity (m/s) in body frame (positive = left)
            yaw_rate: rotational velocity (rad/s)
            state: current state (an integer code)
        """
        # Define thresholds:
        front_threshold = self.reference_distance + self.ranger_buffer

        # Default command is to hover (no motion)
        vx = 0.0
        vy = 0.0
        yaw_rate = 0.0

        # --- State Machine ---
        if self.state == ManhattanWallFollower.State.FORWARD:
            # Fly forward.
            vx = self.max_forward_speed
            # If a wall is detected ahead (front sensor reading falls below threshold),
            # then transition to CORNER_TURN.
            if front_range < front_threshold:
                self.state = ManhattanWallFollower.State.CORNER_TURN
                # For Manhattan constraints, we turn exactly 90°.
                # When following LEFT, a front obstacle implies a turn to the right.
                if self.wall_following_direction == "LEFT":
                    self.desired_heading = self.snap_to_90(current_heading - math.pi/2)
                else:
                    self.desired_heading = self.snap_to_90(current_heading + math.pi/2)
            # Otherwise, if the side sensor reading deviates significantly from the reference,
            # add a small lateral correction.
            else:
                error = side_range - self.reference_distance
                # For wall following, if following LEFT then positive error (wall too far)
                # means the drone should move left (positive vy).
                if abs(error) > self.ranger_buffer:
                    if self.wall_following_direction == "LEFT":
                        # Move left if wall is too far; move right if too close.
                        vy = 0.5 * (-error)
                    else:
                        vy = 0.5 * error
                else:
                    vy = 0.0

        elif self.state == ManhattanWallFollower.State.CORNER_TURN:
            # Perform a 90° turn.
            heading_error = self.wrap_to_pi(self.desired_heading - current_heading)
            if abs(heading_error) < 0.1:
                # Turn complete: transition to FOLLOW_WALL.
                self.state = ManhattanWallFollower.State.FOLLOW_WALL
                yaw_rate = 0.0
            else:
                # Turn with constant rate in the direction of the error.
                yaw_rate = self.turn_rate if heading_error > 0 else -self.turn_rate
                vx = 0.0
                vy = 0.0

        elif self.state == ManhattanWallFollower.State.FOLLOW_WALL:
            # In FOLLOW_WALL state, the drone flies forward along the wall.
            vx = self.max_forward_speed
            # Adjust lateral motion to maintain the reference distance.
            error = side_range - self.reference_distance
            # For LEFT wall following, if error is positive then the wall is farther away
            # so the drone should move left (positive vy); if error is negative, move right.
            if self.wall_following_direction == "LEFT":
                vy = 0.5 * (-error)
            else:
                vy = 0.5 * error

            # If a wall appears in front (indicating a corner), transition to CORNER_TURN.
            if front_range < front_threshold:
                self.state = ManhattanWallFollower.State.CORNER_TURN
                if self.wall_following_direction == "LEFT":
                    self.desired_heading = self.snap_to_90(current_heading - math.pi/2)
                else:
                    self.desired_heading = self.snap_to_90(current_heading + math.pi/2)

        elif self.state == ManhattanWallFollower.State.HOVER:
            vx = 0.0
            vy = 0.0
            yaw_rate = 0.0

        return vx, vy, yaw_rate, self.state
