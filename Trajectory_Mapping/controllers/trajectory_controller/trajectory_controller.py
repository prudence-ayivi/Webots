"""trajectory_controller controller."""

from controller import Robot, Supervisor 
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize sensors and actuators
leftMotor = robot.getDevice('wheel_left_joint')
rightMotor = robot.getDevice('wheel_right_joint')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#Add lidar sensor
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()
# The lidar sensor offset from the centre of the robot
LIDAR_OFFSET = 0.202

# Angles for Lidar readings 
angles = np.linspace(4.19/2, -4.19/2, 667)  # From ~120 degrees to -120 degrees
angles = angles[80:len(angles)-80]

#Add GPS and compass sensors for Odometry
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')

MAX_SPEED = 6.28  # rad/s

# Wheel radius and distance between wheels
WHEEL_RADIUS = 0.10  # meters
WHEEL_DISTANCE = 0.455  # meters

# Initialize mapping structures
MAP_WIDTH = 250               # Width of occupancy grid in pixels 
MAP_HEIGHT = 300              # Height of occupancy grid in pixels

map = np.zeros((MAP_WIDTH, MAP_HEIGHT))
kernel = np.ones((26, 26))

# World-space bounding box of the area to map. It cover the
# waypoint loop below (roughly x in [-1.65, 0.8], y in [-3.2, 0.1]) with a margin. 
X_MIN, X_MAX = -2.2, 1.2
Y_MIN, Y_MAX = -3.6, 0.6

# Converts world (meters) coordinates (xw, yw) to map grid to map (pixels) coordinates (px, py).
def world2map(xw,yw):   
    px = int(52 * xw + 124.8) 
    py = int(-52 * yw + 93.834)

    # px = int((xw - X_MIN) / (X_MAX - X_MIN) * (MAP_WIDTH - 1))
    # py = int((Y_MAX - yw) / (Y_MAX - Y_MIN) * (MAP_HEIGHT - 1))
    
    # Clamp values to map boundaries
    px = min(max(px, 0), MAP_WIDTH - 1) 
    py = min(max(py, 0), MAP_HEIGHT - 1) 
    return [px, py]
           
# Waypoints for trajectory following (loop around the dining table)
WP = [(0.4, 0.1), (0.5, -0.45), (0.5, -1.75), (0.4, -2.9), 
      (-0.4, -3.1), (-0.8, -3.2), (-1.2, -3.2), (-1.6, -2.7), 
      (-1.6, -2.5), (-1.65, -1.2), (-1.65, -0.6), (-1.65, -0.3), 
      (-1.5, -0.1), (-1.4, 0.1), (-1.2, 0.2), (-0.6, 0)]

index = 0
direction = 1                 # +1: forward through WP (CW), -1: backward (CCW)
WAYPOINT_THRESHOLD = 0.3      # ~30 cm, per assignment instructions
finished = False

# Proportional Controller gains (4/5/0.3,0.4, 6/5/0.5 t= 91.4s)
P_ALPHA = 6.0 # proportional gain for heading error
P_RHO = 5.0 # proportional gain for distance error
ANGLE_GATE = 0.6  # rad derivative gain for slowing down when close to the waypoint 

marker = robot.getFromDef("marker").getField("translation")

# Main loop: 
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    # --- Localization (GPS + compass) ---
    xw = gps.getValues()[0]
    yw = gps.getValues()[1] 
    theta = np.arctan2(compass.getValues()[0], compass.getValues()[1]) 
    
    # Update the marker's position to the current waypoint
    marker.setSFVec3f([*WP[index], 0])

    # --- Calculate distance (rho) and heading (alpha) errors to current waypoint ---
    rho = np.sqrt((xw - WP[index][0]) ** 2 + (yw - WP[index][1]) ** 2) # distance error 
    alpha = np.arctan2(WP[index][1] - yw, WP[index][0] - xw) - theta # heading (orientation) error
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

    # Implement full proportional controller for trajectory following
    
    # --- Waypoint switching: forward through WP, then backward ---
    if not finished and rho < WAYPOINT_THRESHOLD:
        if direction == 1 and index == len(WP) - 1:
            direction = -1
        elif direction == -1 and index == 0:
            finished = True
        else:
            index += direction
 
    # --- Proportional controller: turn to face the waypoint, then drive ---
    if finished:
        leftSpeed = rightSpeed = 0.0
    else:
        forward = P_RHO * rho if abs(alpha) < ANGLE_GATE else 0.0
        turn = P_ALPHA * alpha
        leftSpeed = -turn + forward
        rightSpeed = turn + forward
        leftSpeed = max(min(leftSpeed, MAX_SPEED), -MAX_SPEED)
        rightSpeed = max(min(rightSpeed, MAX_SPEED), -MAX_SPEED)
 
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Transformation matrix from robot to world coordinates
    w_T_r = np.array([[np.cos(theta), -np.sin(theta), xw], 
                      [np.sin(theta),  np.cos(theta), yw], 
                      [0,0,1]])

    # Read and process lidar data
    ranges = np.array(lidar.getRangeImage())
    # Exclude the first and last 80 readings to avoid edge artifacts
    ranges = ranges[80:len(ranges)-80]
    ranges[ranges == np.inf] = 100 
    
    # Transform Lidar readings to the robot's coordinate system
    X_i= np.array([ranges * np.cos(angles) + LIDAR_OFFSET, ranges * np.sin(angles), np.ones_like(angles)]) 
    Data = w_T_r @ X_i 
    
    # Draw objects  

    # 2. Probabilistic Occupancy map : Store value on the map / 0.64, -2.46, 0.02 / robot : 0.67, 0.02, 0.095 / -0.6, 0.1, 
    for d in Data.transpose():
        px, py = world2map(d[0], d[1])
        map[px, py] += 0.01
        if (map[px, py]>1):
            map[px, py] = 1 
        v = int(map[px, py] * 255)
        color=(v*256**2+v*256+v)    # Grayscale color
        display.setColor(int(color)) 
        display.drawPixel(px,py)  

    # 1. Draw robot trajectory
        px, py = world2map(xw,yw) 
        display.setColor(0xFF0000)  # red color
        display.drawPixel(px,py)     
    
    # 3. C-space convolution map
    cmap = signal.convolve2d(map, kernel, mode='same') 
    cspace = cmap > 0.9  # Thresholding to create a binary occupancy grid


    if finished:
        print(f"Trajectory complete at simulation time t = {robot.getTime():.1f} s") 

        plt.figure()
        plt.imshow(cspace)
        plt.title(f"Configuration space map at t = {robot.getTime():.1f} s")
        plt.savefig('cspace_map.png')
        plt.show()    
    
    pass
