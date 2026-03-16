"""line_following controller with range finder and homogeneous transformation."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor 
import numpy as np
from matplotlib import pyplot as plt

MAX_SPEED = 6.28  # rad/s

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0  # convert timestep to seconds

# Wheel radius and distance between wheels
WHEEL_RADIUS = 0.0201  # meters
WHEEL_DISTANCE = 0.052  # meters

gs = []
for i in range (3):
        gs.append(robot.getDevice('gs' + str(i)))
        gs[-1].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#Add lidar sensor
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

# Odometry variables (initialization before the loop)
total_distance = 0.0  # Total distance traveled (initialize to 0)
orientation = 0.0  # Orientation in radians (initialize to 0)

# Initial pose of the robot
xw = 0.0       # Robot starts at x = 0
yw = 0.028     # Robot starts slightly ahead of the start line
omegaz = 1.5708   # Robot orientation in radians (90°)

angles = np.linspace(3.1415,-3.1415,360)

# Main loop: 
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    # Process sensor data here.
    g =[]
    for gsensor in gs :
        g.append(gsensor.getValue())
    
    #print(g)
    
    # initialize motor speeds at 50% of MAX_SPEED.
    phildot  = 0.5 * MAX_SPEED # left motor
    phirdot = 0.5 * MAX_SPEED # right motor
    
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 0 
    
    # read sensor data and tranform to robot coordinate system 
    x_r, y_r = [], []
    x_w, y_w = [], [] #tranform in world coordinate system

    #homogeneous transformation
    # w_R_r = np.array([[np.cos(omegaz), -np.sin(omegaz)], # rotation matrix from robot to world coordinates
    #                   [np.sin(omegaz),  np.cos(omegaz)]])
    
    w_T_r = np.array([[np.cos(omegaz), -np.sin(omegaz), xw], # transformation matrix from robot to world coordinates
                      [np.sin(omegaz),  np.cos(omegaz), yw], 
                      [0,0,1]])

    # X_w = np.array([[xw], [yw]]) # robot position in world coordinates
    # print(w_R_r)

    X_i= np.array([ranges * np.cos(angles), ranges * np.sin(angles), np.ones_like(ranges)]) # lidar points in homogeneous coordinates (3x360)
    Data = w_T_r @ X_i # homogeneous transformation from robot to world coordinates (3x360)

    # for i, angle in enumerate(angles):
    #     x_i = ranges[i]*np.cos(angle) #lidar value
    #     y_i = ranges[i]*np.sin(angle) #lidar value
    #     x_r.append(x_i)
    #     y_r.append(y_i)
        
    #     # Data = w_R_r @ np.array([[x_i], [y_i]]) + X_w # homogeneous transformation from robot to world coordinates
    #     Data = w_T_r @ np.array([[x_i], [y_i], [1]]) # homogeneous transformation from robot to world coordinates

    #     x_w.append(Data[0,0])
    #     y_w.append(Data[1,0])
    
    plt.ion()
    plt.plot(Data[0,:], Data[1,:] , '.') 
    plt.pause(0.01) 
    plt.show()
    
    # follow line 
    if (g[0]>0 and g[1] < 250 and g[2] > 500 ) : #drive straight
         phildot, phirdot = 0.35 * MAX_SPEED, 0.35 * MAX_SPEED 

    elif (g[2] < 550) : # turn right
         phildot, phirdot = 0.1 * MAX_SPEED, -0.1 * MAX_SPEED 
    
    elif (g[1] > 500) : # turn left
         phildot, phirdot = -0.1 * MAX_SPEED, 0.1 * MAX_SPEED 
    
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)

    # Odometry calculations
    v_l = WHEEL_RADIUS * phildot  # Linear speed of left wheel
    v_r = WHEEL_RADIUS * phirdot  # Linear speed of right wheel
    
    # Compute displacement Δx and change in orientation Δωz
    delta_x = (v_l + v_r) / 2 * dt
    delta_wz = (v_r - v_l) / WHEEL_DISTANCE * dt 
    # print(f"Displacement : {delta_x:.3f} m, Orientation: {delta_wz:.2f} rad")
    
    #  total distance and orientation
    total_distance += delta_x
    orientation += delta_wz
    
    # Convert orientation to degrees for readability
    orientation_deg = (orientation / 3.1415) * 180
    
    # Print odometry data
    #print(f"Distance traveled: {total_distance:.3f} m, Orientation: {orientation_deg:.2f} degrees")
    
    # Update orientation and position
    omegaz += delta_wz
    xw = xw + np.cos(omegaz) * delta_x
    yw = yw + np.sin(omegaz) * delta_x 
    
    error = np.sqrt(xw**2 + yw**2) # Euclidean error distance to (0,0)
    
    print(f"Localization: xw = {xw:.2f} m, yw = {yw:.2f} m, omegaz = {np.degrees(omegaz):.2f} °, Error = {error:.3f}")
    
    if (-303 <= np.degrees(omegaz) <= -300):
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        print("Robot has returned to the start line within acceptable error.")
        break
    
    
    pass
