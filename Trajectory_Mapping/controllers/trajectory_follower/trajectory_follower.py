"""trajectory_controller controller.

TiagoLite in kitchen.wbt: GPS/compass trajectory following around the
dining table (clockwise then counter-clockwise), combined with
probabilistic grid mapping and, at the end, a configuration-space
computation (obstacle growth by convolution + 90% threshold).
"""

from controller import Supervisor
import numpy as np
from matplotlib import pyplot as plt

try:
    from scipy import signal

    def grow_obstacles(prob_map, kernel):
        return signal.convolve2d(prob_map, kernel, mode='same')
except ImportError:
    # Fallback if scipy isn't available in Webots' bundled Python.
    def grow_obstacles(prob_map, kernel):
        kh, kw = kernel.shape
        pad_h, pad_w = kh // 2, kw // 2
        padded = np.pad(prob_map, ((pad_h, pad_h), (pad_w, pad_w)))
        out = np.zeros_like(prob_map)
        for i in range(prob_map.shape[0]):
            for j in range(prob_map.shape[1]):
                out[i, j] = np.sum(padded[i:i + kh, j:j + kw] * kernel)
        return out

# ---------------------------------------------------------------
# Robot / device initialization
# ---------------------------------------------------------------
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('wheel_left_joint')
rightMotor = robot.getDevice('wheel_right_joint')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Lidar
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

lidar_resolution = lidar.getHorizontalResolution()
lidar_fov = lidar.getFov()
angles = np.linspace(lidar_fov / 2, -lidar_fov / 2, lidar_resolution)

# The Hokuyo sits behind a shield that blanks out the first/last ~80
# readings on each side (Hint 2) -> drop them from angles AND ranges.
UNUSED = 80
angles = angles[UNUSED:lidar_resolution - UNUSED]

# The lidar sensor offset from the centre, so we must add it back to each
# point in the robot frame before rotating/translating to world coords

LIDAR_OFFSET = 0.202

# #Add GPS and compass sensors for Odometry at translation [-0.203, 0, 0] inside lidarSlot
gps = robot.getDevice('gps')
gps.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)

# display = robot.getDevice('display')
display = robot.getDevice('Astra rgb')
display = robot.getDevice('Astra depth')


MAX_SPEED = 6.28  # rad/s

# ---------------------------------------------------------------
# Map
# ---------------------------------------------------------------
MAP_SIZE = 300

# World-space bounding box of the area to map. It cover the
# waypoint loop below (roughly x in [-1.65, 0.78], y in [-3.25, 0.2]) with a margin. 
X_MIN, X_MAX = -2, 1.2
Y_MIN, Y_MAX = -3.6, 0.6

probability_map = np.zeros((MAP_SIZE, MAP_SIZE))


def world2map(xw, yw):    
    # Convert from world (meters) to map (pixels)
    px = int((xw - X_MIN) / (X_MAX - X_MIN) * (MAP_SIZE - 1))
    py = int((Y_MAX - yw) / (Y_MAX - Y_MIN) * (MAP_SIZE - 1))

    # Clamp values to map boundaries
    px = min(max(px, 0), MAP_SIZE - 1)
    py = min(max(py, 0), MAP_SIZE - 1)
    return [px, py]

def draw_probability_pixel(px, py):
    # Convert 0..1 probability to a gray level carefully: multiply by
    # 255 and convert to int ONCE, then reuse that same int for all
    # three channels (otherwise you get washed-out or rainbow pixels).
    v = int(probability_map[px, py] * 255)
    v = min(max(v, 0), 255)
    color = (v << 16) + (v << 8) + v
    display.setColor(color)
    display.drawPixel(px, py)


# ---------------------------------------------------------------
# Waypoints (clockwise loop around the dining table)
# ---------------------------------------------------------------
WP = [(0.35, 0), (0.78, -0.45), (0.78, -1.25), (0.68, -1.75), (0.6, -2.75),
      (0.5, -2.95), (0.4, -3.1), (0.2, -3.1), (-0.8, -3.2), (-1.4, -3.25), 
      (-1.6, -2.7), (-1.6, -2.5), (-1.65, -1.2), (-1.65, -0.6), 
      (-1.65, -0.3), (-1.5, -0.1), (-1.4, 0.1), (-1.2, 0.2), (0, 0)]

index = 0
direction = 1                 # +1: forward through WP (CW), -1: backward (CCW)
WAYPOINT_THRESHOLD = 0.3      # ~30 cm, per assignment instructions
finished = False

marker = robot.getFromDef("marker").getField("translation")

# PID roportional Controller gains
P_ALPHA = 4.0 # proportional gain for heading error
P_RHO = 3.0 # proportional gain for distance error
ANGLE_GATE = 0.3   # rad (~17 deg): below this, start moving forward too

# Trajectory following using a simple proportional controller 
p1 = 1 # proportional gain for heading error
p2 = 10 # proportional gain for distance error
p3 = 0.5 # derivative gain for slowing down when close to the waypoint


# ---------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------
while robot.step(timestep) != -1:

    # --- Localization (GPS + compass) ---
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])

    marker.setSFVec3f([*WP[index], 0])

    # --- Errors to current waypoint ---
    rho = np.sqrt((xw - WP[index][0]) ** 2 + (yw - WP[index][1]) ** 2) # distance error 
    alpha = np.arctan2(WP[index][1] - yw, WP[index][0] - xw) - theta # heading (orientation) error
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # wrap to [-pi, pi]

    # --- Waypoint switching: forward through the list, then backward ---
    if not finished and rho < WAYPOINT_THRESHOLD:
        if direction == 1 and index == len(WP) - 1:
            direction = -1
        elif direction == -1 and index == 0:
            finished = True
        else:
            index += direction

    # --- Trajectory controller: turn to face waypoint, then drive ---
    if finished:
        leftSpeed = rightSpeed = 0.0
    else:
        forward = rho * P_RHO if abs(alpha) < ANGLE_GATE else 0.0
        turn = alpha * P_ALPHA 
        leftSpeed = -turn + forward
        rightSpeed = turn + forward
        leftSpeed = max(min(leftSpeed, MAX_SPEED), -MAX_SPEED)
        rightSpeed = max(min(rightSpeed, MAX_SPEED), -MAX_SPEED)
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # --- Homogeneous transform from robot to world coordinates ---
    w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw],
                       [np.sin(theta), np.cos(theta), yw],
                       [0, 0, 1]])

    # --- Lidar processing ---
    ranges = np.array(lidar.getRangeImage())
    ranges = ranges[UNUSED:lidar_resolution - UNUSED]
    ranges[np.isinf(ranges)] = 100  # push erroneous readings far away

    X_i = np.array([ranges * np.cos(angles) + LIDAR_OFFSET,
                     ranges * np.sin(angles),
                     np.ones_like(ranges)]) # lidar points in homogeneous coordinates (3xN)
    Data = w_T_r @ X_i  # lidar ranges images transform to world coordinates (3xN)

    # --- Draw robot trajectory ---
    # px, py = world2map(xw, yw)
    # display.setColor(0xFF0000)
    # display.drawPixel(px, py)

    # --- Probabilistic occupancy update + display, per hit point ---
    # for d in Data.T:
    #     px, py = world2map(d[0], d[1])
    #     probability_map[px, py] = min(probability_map[px, py] + 0.005, 1.0)
    #     draw_probability_pixel(px, py)

    # --- Once the loop (CW + CCW) is done: compute configuration space ---
    if finished:
        print(f"Trajectory complete at simulation time t = {robot.getTime():.1f} s")

        # kernel_size = 12  # ~ robot radius in map pixels; tune to your scale
        # kernel = np.ones((kernel_size, kernel_size))
        # cspace = grow_obstacles(probability_map, kernel)
        # cspace = cspace / cspace.max()
        # cspace = cspace > 0.9

        # plt.figure()
        # plt.imshow(cspace, cmap='gray')
        # plt.title(f"Configuration space at t = {robot.getTime():.1f} s")
        # plt.savefig('cspace_map.png')
        # plt.show()

        break