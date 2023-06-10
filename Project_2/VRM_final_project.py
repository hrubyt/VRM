# Project VRM
# authors: Hruby Tomas, Ther Tomas, Rolny Jakub
# modified code from Erwin Coumans https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
import pybullet as p
import time
import math
import pybullet_data
import os
import numpy as np
import random

# constants for husky & kuka
SLEEP_TIME = 0.04
MIN_TURN_VELOCITY = 0.4
MAX_TURN_VELOCITY = 3
MAX_KUKA_VELOCITY = 0.5
POSITION_TOLERANCE = 2e-2
WAIT_TIME = 3
p.setTimeStep = SLEEP_TIME

# Key commands binding
automatic = ord('o')
stop = ord('q')
generate_target = ord('h')
turn_to_target = ord('j')
husky_forward = ord('k')
move_arm = ord('l')
remove_target = ord('u')

# Get path to fithub husky & kuka arm models
os.system("git clone https://github.com/bulletphysics/bullet3/tree/master/data/husky.git")
os.system("git clone https://github.com/bulletphysics/bullet3/tree/master/data/kuka_iiwa")

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)

p.setPhysicsEngineParameter(enableConeFriction=0)

# Generate environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.3])

# Generate and initialize models
husky = p.loadURDF("husky/husky.urdf", [0.290388, 0.329902, -0.310270], [0.002328, -0.000984, 0.996491, 0.083659])
for i in range(p.getNumJoints(husky)):
    print(p.getJointInfo(husky, i))
kukaId = p.loadURDF("kuka_iiwa/model_free_base.urdf", 0.193749, 0.345564, 0.120208, 0.002327,-0.000988, 0.996491, 0.083659)

joint_positions_default = [1, 0.5, 1, 1.5, 0.5, -1, 0.5]
KUKA_NUM_JOINTS = p.getNumJoints(kukaId)
print("KUKa join:",KUKA_NUM_JOINTS )
# Initialize settings of arm
for jointIndex in range(KUKA_NUM_JOINTS):
    p.resetJointState(kukaId, jointIndex, joint_positions_default[jointIndex])
    p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, targetPosition=joint_positions_default [jointIndex])

#put kuka on top of husky
cid = p.createConstraint(husky, -1, kukaId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., -.5], [0, 0, 0, 1])

#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.95, 2, 2.96, 2.29, 2.96, 2.09, 2.95] #3.05
#joint ranges for null space.
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi* 0.5*0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

kukaEndEffectorIndex = 6        # end effector
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
    exit()

# Turn on gravity and simulation time
p.setGravity(0, 0, -9.81)
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
wheels = [2, 3, 4, 5] # 2 front left wheel, 3 front right, 4 rear left, 5 rear right
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]
targetPositionsJoints = [0,0,0,0,0,0,0]
remove_idx = 0
ball = 0
# Quaternion for endeffector to point down
Orientation = p.getQuaternionFromEuler([math.pi, 0., 0,])
# Define a function to calculate the distance between two points
def distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1) ** 2 + (y2 - y1) ** 2)
# Limit the value of a variable to intervals.
# {-MAX_VELOCITY, -MIN_VELOCITY} u {MAX_VELOCITY, MIN_VELOCITY}
def limit_value(value):
    return np.sign(value)*min(max(abs(value), MIN_TURN_VELOCITY), MAX_TURN_VELOCITY)

def move_arm_to_target(robot, target):
    target_position, *_ = p.getBasePositionAndOrientation(target)
    target_position = list(target_position)
    target_position[2] = target_position[2]
    targetPositionsJoints = p.calculateInverseKinematics(robot,
                                                          kukaEndEffectorIndex,
                                                          target_position,
                                                          Orientation,
                                                          lowerLimits=ll,
                                                          upperLimits=ul,
                                                          jointRanges=jr,
                                                          restPoses=rp,
                                                          jointDamping=jd,
                                                          maxNumIterations=50)
    targetPositionsJoints = list(targetPositionsJoints)

    for jointIndex in range(KUKA_NUM_JOINTS):
        p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, targetPosition=targetPositionsJoints[jointIndex], maxVelocity=MAX_KUKA_VELOCITY)

    joints_positon_ok = 0
    # Get positions of all joints and compare them with target positions
    for i in range(len(targetPositionsJoints)):
        joint_position_actual,*_ = p.getJointState(kukaId, i)
        print("Actual:", joint_position_actual)
        if abs(abs(joint_position_actual) - abs(targetPositionsJoints[i])) < POSITION_TOLERANCE:
            joints_positon_ok = joints_positon_ok + 1

    print("Target: ", targetPositionsJoints)
    print("Joints ok: ", joints_positon_ok)
    for jointIndex in range(KUKA_NUM_JOINTS):
        p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, targetPosition=targetPositionsJoints[jointIndex], maxVelocity=MAX_KUKA_VELOCITY)
    time.sleep(1)
    if joints_positon_ok == KUKA_NUM_JOINTS:
        return True
    return False


# Initialize the wheel velocities to zero
velocity = 0.0 # rad/s
for joint in wheels:
    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL, targetVelocity=velocity)

# Set the linear speed of the Husky robot (m/s)
linear_speed = 1.0

# Initialize time_required
time_required = 0.0
timeNeeded = 0.0

# Run the simulation loop.
move_forward = False
turn = False
start_time = 0.0
startTime = 0.0

# Set the turning radius of the Husky
turningRadius = 0.5

# Set the wheel separation of the Husky
wheelSeparation = 0.7

# Set the direction of the turn (1 = right, -1 = left)
turnDirection = 1

# Define a function that turns the husky robot towards the object ballS
def turn_towards_ball():
    # Get the position and orientation of the husky robot and the object ball
    ball_pos = target_pos
    husky_pos, husky_orn = p.getBasePositionAndOrientation(husky)
    *_, husky_z_orn_E = p.getEulerFromQuaternion(husky_orn)
    target_z_orn_E = math.atan2(ball_pos[1] - husky_pos[1], ball_pos[0] - husky_pos[0]) 
    degree_diff = target_z_orn_E - husky_z_orn_E
    # Prevent husky's indecisiveness about which side to turn
    if abs(degree_diff) > 3.1:
        degree_diff= 3.1

    # Set the left and right wheel velocities proportional to the angle difference with upper and lower limits 
    turning_val = limit_value(2*degree_diff)
    left_vel =  -turning_val
    right_vel = +turning_val
    p.setJointMotorControl2(husky, 2, p.VELOCITY_CONTROL, targetVelocity=left_vel)
    p.setJointMotorControl2(husky, 3, p.VELOCITY_CONTROL, targetVelocity=right_vel)
    p.setJointMotorControl2(husky, 4, p.VELOCITY_CONTROL, targetVelocity=left_vel)
    p.setJointMotorControl2(husky, 5, p.VELOCITY_CONTROL, targetVelocity=right_vel)
    print("rychlost otaceni: ", turning_val)
    print("rozdil uhlu: ", degree_diff)

    if abs(degree_diff) < 0.05:
        # Stop turning
        p.setJointMotorControl2(husky, 2, p.VELOCITY_CONTROL, targetVelocity=0)
        p.setJointMotorControl2(husky, 3, p.VELOCITY_CONTROL, targetVelocity=0)
        p.setJointMotorControl2(husky, 4, p.VELOCITY_CONTROL, targetVelocity=0)
        p.setJointMotorControl2(husky, 5, p.VELOCITY_CONTROL, targetVelocity=0)
        return True
    return False

def move_husky_forward():
    # Get the position of the husky robot and the object ball 
    husky_pos,_ = p.getBasePositionAndOrientation(husky) 
    ball_pos,_ = p.getBasePositionAndOrientation(ball)

    # Calculate the distance between the husky and the ball
    dist= distance(husky_pos[0], husky_pos[1], ball_pos[0], ball_pos[1]) 
    print("vzdalenost je:", dist)
    vel = 2
    p.setJointMotorControl2(husky, 2, p.VELOCITY_CONTROL, targetVelocity=vel)
    p.setJointMotorControl2(husky, 3, p.VELOCITY_CONTROL, targetVelocity=vel)
    p.setJointMotorControl2(husky, 4, p.VELOCITY_CONTROL, targetVelocity=vel) 
    p.setJointMotorControl2(husky, 5, p.VELOCITY_CONTROL, targetVelocity=vel)

    if abs(dist) < 0.7:
        p.setJointMotorControl2(husky, 2, p.VELOCITY_CONTROL, targetVelocity=0) 
        p.setJointMotorControl2(husky, 3, p.VELOCITY_CONTROL, targetVelocity=0) 
        p.setJointMotorControl2(husky, 4, p.VELOCITY_CONTROL, targetVelocity=0) 
        p.setJointMotorControl2(husky, 5, p.VELOCITY_CONTROL, targetVelocity=0) 
        return True
    return False

def generate_random_target():
    husky_pos, _ = p.getBasePositionAndOrientation(husky)
    while True:
        rand_x = husky_pos[0] + random.uniform(-ball_radius, ball_radius) 
        rand_y= husky_pos[1] + random.uniform(-ball_radius, ball_radius) 
        rand_z = random.uniform(0.1, 0.7) # Set the height of the ball 
        if distance(husky_x_pos, husky_y_pos, rand_x, rand_y) > 2: 
            return rand_x, rand_y, rand_z
    
    
ball_radius = 3
state = 10
mode_auto = False

while True:
    keys = p.getKeyboardEvents()
    pos, orn  = p.getBasePositionAndOrientation(husky)
    husky_x_pos = pos[0]
    husky_y_pos = pos[1]

    if stop in keys:
        mode_auto = False
        p.removeBody(ball)
        # Return the arm to default position
        for jointIndex in range(KUKA_NUM_JOINTS):
            p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, targetPosition=joint_positions_default[jointIndex], maxVelocity=MAX_KUKA_VELOCITY)
        # Stop husky
        p.setJointMotorControl2(husky, 2, p.VELOCITY_CONTROL, targetVelocity=0) 
        p.setJointMotorControl2(husky, 3, p.VELOCITY_CONTROL, targetVelocity=0) 
        p.setJointMotorControl2(husky, 4, p.VELOCITY_CONTROL, targetVelocity=0) 
        p.setJointMotorControl2(husky, 5, p.VELOCITY_CONTROL, targetVelocity=0) 
        state = 10
    
    # WAIT
    if state == 10:
        if automatic in keys:
            mode_auto= True
            state = 20
        elif generate_target in keys and keys[generate_target] & p.KEY_IS_DOWN:
            print("novy bod")
            state = 20
        elif turn_to_target in keys and keys[turn_to_target] & p.KEY_IS_DOWN:
            print("otocit k bodu")
            state = 30
        elif husky_forward in keys:
            print("husky vpred")
            state = 40
        elif move_arm in keys:
            print("rameno k bodu")
            state = 50
        elif remove_target in keys:
            print("odstranit bod")
            state = 60

    # Generate target 
    elif state == 20:      
        #h
        # Generate target ball position
        rand_x, rand_y, rand_z = generate_random_target()
        # Show ball in simulation
        ball = p.loadURDF("ball.urdf", [rand_x, rand_y, rand_z], useFixedBase = 1)
        # Get the position and orientation of the ball
        pos, orn = p.getBasePositionAndOrientation(ball) 
        target_pos = pos
        print("pozice noveho bodu je:", target_pos) 
        if mode_auto:
            state = 30
        else:
            state = 10

    #Turn husky towards the target   
    elif state == 30:           
        #j
        if turn_towards_ball():
            if mode_auto:
                state = 40
            else:
                state = 10

    # Move husky towards the target
    elif state == 40:
        #k
        if move_husky_forward():
            time.sleep(1)
            timeout = time.time() + 5 # in case InverseKinematic or MotorControl don't work properly 
            if mode_auto:
                state = 50
            else:
                state = 10
        
    # Move arm to the target
    elif state == 50:
        #l
        if move_arm_to_target(kukaId, ball) or time.time() > timeout:
            time.sleep(1)
            if mode_auto:
                state = 60
            else:
                state = 10

    # Remove ball and move arm to default position
    elif state == 60:
        #u 
        try:
            p.removeBody(ball)
        except:
            print("No body to remove")

        for jointIndex in range(KUKA_NUM_JOINTS):
            p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, targetPosition=joint_positions_default[jointIndex], maxVelocity=MAX_KUKA_VELOCITY)
        time.sleep(1)
        if mode_auto:
            state = 20
        else:
            state = 10

    p.stepSimulation()
    time.sleep(SLEEP_TIME)
