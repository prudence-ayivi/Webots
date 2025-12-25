from controller import Robot, Motor

TIME_STEP = 64

MAX_SPEED = 6.28 # rad/s (approx. 2π rad/s)
WHEEL_RADIUS = 0.0205  # mm

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
#leftMotor.setPosition(6.28)
#rightMotor.setPosition(0)
#leftMotor.setPosition(6.28)
#rightMotor.setPosition(6.28)

# set up the motor speeds at 10% of the MAX_SPEED.
leftMotor.setVelocity(10.0 )
rightMotor.setVelocity(12.0)
#rightMotor.setVelocity(0.1 * MAX_SPEED)

# Calculate the distance per second
#dist_per_sec = 2 * 3.14159 * WHEEL_RADIUS  # meters traveled per second
time = 3  # seconds

# Total distance traveled in 3 seconds
#total_distance = dist_per_sec * time # meters

#print(f"The robot traveled {total_distance * 100:.2f} cm in {time} seconds.")  # Convert to cm

while robot.step(TIME_STEP) != -1:
    
   pass