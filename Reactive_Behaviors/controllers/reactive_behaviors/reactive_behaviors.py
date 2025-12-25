"""reactive_behaviors controller."""

from controller import Robot, Motor, DistanceSensor

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28  # rad/s
THRESHOLD_DIST = 0.05  # threshold distance in meters
TURN_SPEED = 0.3 * MAX_SPEED  # speed during turning

# create the Robot instance.
robot = Robot()

# Initialize distance sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# State enumeration
STATE_DRIVE_FORWARD = 0
STATE_TURN_180 = 1
STATE_DRIVE_FORWARD_AFTER_TURN = 2
STATE_FIND_LEFT_OBJECT = 3
STATE_DRIVE_LEFT_OBJECT = 4
STATE_WAIT_FOREVER = 5

# Initialize state
state = STATE_DRIVE_FORWARD

#Convert the sensor's intensity value to an approximate distance in meters.
def get_distance(sensor_value):
    
    return 2.5 * (sensor_value / 4096)  

while robot.step(TIME_STEP) != -1: 
    if state == STATE_DRIVE_FORWARD:
        # Drive forward until within 0.05m of an object
        front_distance = get_distance(ps[0].getValue())
        print(f"Front Distance: {front_distance:.3f}")

        if front_distance >= THRESHOLD_DIST:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            state = STATE_TURN_180
        else:
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)

    elif state == STATE_TURN_180:
        # Turn 180 degrees by actuating wheels in opposite directions
        leftMotor.setVelocity(-TURN_SPEED) 
        rightMotor.setVelocity(TURN_SPEED)
        back_right_distance = get_distance(ps[4].getValue())
        print(f"Back Right Distance: {back_right_distance:.3f}")

        # Stop turning if both back sensors read <= 0.05
        if back_right_distance >= THRESHOLD_DIST:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            state = STATE_DRIVE_FORWARD_AFTER_TURN             
            print("Turn completed.")
            
    elif state == STATE_DRIVE_FORWARD_AFTER_TURN:
        # Drive forward again before searching for the left object
        front_distance = get_distance(ps[0].getValue())
        print(f"Front Distance (After Turn): {front_distance:.3f}")

        if front_distance >= THRESHOLD_DIST:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            state = STATE_FIND_LEFT_OBJECT
        else:
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
    elif state == STATE_FIND_LEFT_OBJECT:
        # Rotate clockwise until the left distance sensor reads > 0.05m
        leftMotor.setVelocity(TURN_SPEED)
        rightMotor.setVelocity(-TURN_SPEED)
        left_distance = get_distance(ps[5].getValue())
        print(f"Left Distance: {left_distance:.3f}")
        
        if left_distance >= THRESHOLD_DIST:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            state = STATE_DRIVE_LEFT_OBJECT
            
    elif state == STATE_DRIVE_LEFT_OBJECT:
        # Drive forward as long as the left sensor reads > 0.05m
        left_distance = get_distance(ps[5].getValue())
        print(f"Left Distance (Driving): {left_distance:.3f}")
        
        if left_distance >= THRESHOLD_DIST:
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
        else:
            leftMotor.setVelocity(0.0)
            rightMotor.setVelocity(0.0)
            state = STATE_WAIT_FOREVER

    elif state == STATE_WAIT_FOREVER:
        # Stop and do nothing        
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        print("Robot is waiting forever.")

    print(f"State: {state}, Front Distance: {front_distance:.3f}")

   
