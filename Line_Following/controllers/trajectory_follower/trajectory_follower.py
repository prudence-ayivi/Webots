""" trajectorye_follower controller"""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Supervisor 
import numpy as np
from matplotlib import pyplot as plt
# from scipy import signal

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0  # convert timestep to seconds

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

#Add GPS and compass sensors for Odometry
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

display = robot.getDevice('display')

# Odometry variables (initialization before the loop)
total_distance = 0.0  # Total distance traveled (initialize to 0)
orientation = 0.0  # Orientation in radians (initialize to 0)

MAX_SPEED = 6.28  # rad/s

# Wheel radius and distance between wheels
WHEEL_RADIUS = 0.0201  # meters
WHEEL_DISTANCE = 0.052  # meters

# Initial pose of the robot
xw = 0.0       # Robot starts at x = 0
yw = 0.028     # Robot starts slightly ahead of the start line
omegaz = 1.5708   # Robot orientation in radians (90°)

angles = np.linspace(3.1415,-3.1415,360)
# angles = np.linspace(1.5708/2,-1.5708/2,360)

map = np.zeros((300,300))

def world2map(xw,yw):   
   # Convert from world (meters) to map (pixels), world coordinates shifted to (0,0) as center
    px = int((xw + 0.5 - 0.305)*300)
    py = int(299-(yw + 0.25)*300)
    
    # Clamp values to map boundaries
    px = min(px,299) 
    py = min(py,299) 
    px = max(px,0)
    py = max(py,0)
    
    return [px,py]

cmap = False

# Waypoints for trajectory following
WP = [(0, 0.68), (0.44, 0.68), (0.66, 0.51), (0.35, 0.24), (0.63, 0), (0.63, -0.16), (0, -0.16), (0, 0)]
index = 0

display.setColor(0x00FF00)

marker = robot.getFromDef("marker").getField("translation")
#marker.setSFVec3f([0, 0, 0.2])

# Main loop: 
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    xw = gps.getValues()[0]
    yw = gps.getValues()[1] 
    omegaz= np.arctan2(compass.getValues()[0], compass.getValues()[1]) 
    #print(omegaz)

    marker.setSFVec3f([*WP[index], 0])

    rho = np.sqrt((xw - WP[index][0])**2 + (yw - WP[index][1])**2) # distance error to the current waypoint
    alpha = np.arctan2(WP[index][1] - yw, WP[index][0] - xw) - omegaz # heading (orientation) error to the current waypoint
    
    # Normalize alpha to the range [-pi, pi]
    if (alpha > np.pi):
        alpha -= 2*np.pi
    
    print(rho, alpha/3.1425*180)

    if (rho < 0.1) :
         index += 1
       
        # print(f"Reached waypoint {index} at position ({xw:.2f}, {yw:.2f})")
        # index += 1
        # if (index >= len(WP)) :
        #     print("All waypoints reached. Stopping the robot.")
        #     leftMotor.setVelocity(0.0)
        #     rightMotor.setVelocity(0.0)
        #     break
    
    # Process sensor data here.
    g =[]
    for gsensor in gs :
        g.append(gsensor.getValue())
        
    # initialize motor speeds at MAX_SPEED.
    leftSpeed = MAX_SPEED # left motor
    rightSpeed = MAX_SPEED # right motor
    
    #homogeneous transformation
    w_T_r = np.array([[np.cos(omegaz), -np.sin(omegaz), xw], # transformation matrix from robot to world coordinates
                      [np.sin(omegaz),  np.cos(omegaz), yw], 
                      [0,0,1]])

    # Read and process lidar data
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100 

    X_i= np.array([ranges * np.cos(angles), ranges * np.sin(angles), np.ones_like(ranges)]) # lidar points in homogeneous coordinates (3x360)
    Data = w_T_r @ X_i # transform ranges images to world coordinates (3x360)
    
     # Draw objects 
    for i in range(Data.shape[1]): 
        x_i = Data[0, i] 
        y_i = Data[1, i] 
        
        px, py = world2map(x_i, y_i) 
        color = 0xFFFFFF  
        display.setColor(int(color)) 
        display.drawPixel(px, py)
    
    # 1. Draw robot trajectory
    px, py = world2map(xw,yw)
    display.setColor(0xFF0000)
    display.drawPixel(px,py)    

    # 2. Probabilistic Mapping : Store value on the map
    for d in Data.transpose():
         px, py = world2map(d[0], d[1])
         map[px, py] += 0.005
         if (map[px, py]>1):
              map[px, py] = 1 
    v = int(map[px, py] * 255)
    color=(v*256**2+v*256+v)
    #color = 0xFFFFFF 
    display.setColor(int(color)) 
    display.drawPixel(px,py)  

    # plt.ion()
    # plt.plot(Data[0,:], Data[1,:] , '.') 
    # plt.pause(0.01) 
    # plt.show()

    
    # follow line 
    if (g[0]>0 and g[1] < 250 and g[2] > 500 ) : #drive straight
         leftSpeed, rightSpeed = 0.5 * MAX_SPEED, 0.5 * MAX_SPEED 

    elif (g[2] < 550) : # turn right
         leftSpeed, rightSpeed = 0.25 * MAX_SPEED, -0.05 * MAX_SPEED 
    
    elif (g[1] > 500) : # turn left
         leftSpeed, rightSpeed = -0.05 * MAX_SPEED, 0.25 * MAX_SPEED 
    
    # Odometry calculations
    v_l = WHEEL_RADIUS * leftSpeed  # Linear speed of left wheel
    v_r = WHEEL_RADIUS * rightSpeed  # Linear speed of right wheel
    
    # Compute displacement Δx and change in orientation Δωz
    delta_x = (v_l + v_r) / 2 * dt
    delta_wz = (v_r - v_l) / WHEEL_DISTANCE * dt 
    
    # Update orientation and position
    omegaz += delta_wz
    xw = xw + np.cos(omegaz) * delta_x
    yw = yw + np.sin(omegaz) * delta_x 
    
    error = np.sqrt(xw**2 + yw**2) # Euclidean error distance to (0,0)

    # Trajectory following using a simple proportional controller
    p1 = 1 # proportional gain for heading error
    p2 = 10 # proportional gain for distance error

    leftSpeed = - alpha*p1 + rho*p2
    rightSpeed = alpha*p1 + rho*p2

    leftSpeed = max(min(leftSpeed,6.28),-6.28)
    rightSpeed = max(min(rightSpeed,6.28),-6.28)

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    print(f"Localization: xw = {xw:.2f} m, yw = {yw:.2f} m, omegaz = {np.degrees(omegaz):.2f} °, Error = {error:.3f}")
    
    if (-303 <= np.degrees(omegaz) <= -300):
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        print("Robot has returned to the start line within acceptable error.")
        break
    
    
    pass
