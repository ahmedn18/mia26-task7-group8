"""Tuning constants shared by the action servers and the stages.

Distances come from maze_world.sdf in the simulation repo. Arena is
5.0 x 3.0 m, the robot starts at (0.5, 0.5) facing +x, gate 1 sits at
y = 1.0 (closed at startup) and gate 2 at y = 2.0 (open at startup).
"""

import math

# --- Gates -----------------------------------------------------------------

# One SetBool service moves both gates, in opposite directions:
#   data: true  -> gate 1 opens, gate 2 closes
#   data: false -> gate 1 closes, gate 2 opens
GATE_SERVICE = 'toggle_walls_1_2'

# The service returns success as soon as the command is published, not when
# the wall has finished moving. The wall travels 1.0 m at a 0.5 m/s joint
# limit, so a stage must not cross until this much has elapsed since its
# request. Measure it and trim -- if the robot only needs the wall to clear
# its roof this drops under a second and we win time at both gates.
GATE_OPEN_S = 2.0

# --- Stage legs (metres / radians) -----------------------------------------

STAGE1_APPROACH_M = 0.30
STAGE1_CROSS_M = 0.45

STAGE2_APPROACH_M = 0.55
STAGE2_CROSS_M = 0.70

# Stage 3 is split so the final goal is short. The simulation shuts itself
# down the moment the robot touches the pad, which cuts the in-flight goal
# off mid-action; keeping that goal small means only the last 0.30 m is
# exposed to it and everything before is verified normally.
STAGE3_APPROACH_M = 4.20   # up to the pad's west edge at x = 4.7
STAGE3_TOUCH_M = 0.30      # onto the pad centre at x = 5.0

# Turns, radians, positive is left.
STAGE1_TURN_RAD = math.pi / 2.0    # onto the +y corridor
STAGE3_TURN_RAD = -math.pi / 2.0   # onto the finish run

# --- Action server control loop --------------------------------------------

CONTROL_HZ = 20.0

# Abort a goal if /odom goes quiet for this long -- a dead proprioceptive
# sensor must not leave the robot driving blind.
ODOM_TIMEOUT_S = 2.0

# How long to wait for the very first /odom message before giving up.
ODOM_STARTUP_TIMEOUT_S = 15.0

# Abort a goal that has not completed in this long.
GOAL_TIMEOUT_S = 30.0

# Linear motion.
LINEAR_TOLERANCE_M = 0.01
MAX_LINEAR_SPEED = 0.20
MIN_LINEAR_SPEED = 0.04
LINEAR_KP = 1.0

# Rotation.
YAW_TOLERANCE_RAD = 0.02
MAX_YAW_SPEED = 0.60
MIN_YAW_SPEED = 0.10
YAW_KP = 1.5
