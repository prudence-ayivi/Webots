from controller import Robot, LightSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
ls = []
lsNames = [
    'ls0', 'ls1', 'ls2', 'ls3',
    'ls4', 'ls5', 'ls6', 'ls7'
]

for i in range(8):
    ls.append(robot.getDevice(lsNames[i]))
    ls[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
#leftMotor.setVelocity(0.0)
#rightMotor.setVelocity(0.0) 


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    lsValues = []
    for i in range(8):
        lsValues.append(ls[i].getValue())
        
    leftMotor.setVelocity(lsValues[7]/1000.0)
    rightMotor.setVelocity(lsValues[0]/1000.0)

        
    #print (lsValues)

    # detect obstacles
    #right_obstacle = lsValues[0] > 80.0 or lsValues[1] > 80.0 or lsValues[2] > 80.0
    #left_obstacle = lsValues[5] > 80.0 or lsValues[6] > 80.0 or lsValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    #leftSpeed  = 0.5 * MAX_SPEED
    #rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    #if left_obstacle:
        # turn right
        #leftSpeed  = 0.5 * MAX_SPEED
        #rightSpeed = -0.5 * MAX_SPEED
    #elif right_obstacle:
        # turn left
        #leftSpeed  = -0.5 * MAX_SPEED
        #rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    #leftMotor.setVelocity(leftSpeed)
    #rightMotor.setVelocity(rightSpeed)
    pass