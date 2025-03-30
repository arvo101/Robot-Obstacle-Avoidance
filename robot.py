import pybullet as p
import pybullet_data
import time
import math
from block_positions import blockPos

STEPS = 100000
SLOW_DOWN_VELOCITY = 15
VELOCITY_INCREMENT = 1.01
RAY_DISTANCE = -1.5
SENSOR_ANGLE = math.radians(30)
MAX_FORCE = 100

# Initialize PyBullet
physicsClient = p.connect(p.GUI)

# Set up the simulation environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.47]
startOrn = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("r2d2.urdf", startPos, startOrn)
blockOrn = p.getQuaternionFromEuler([0, 0, 0,])

# Load block to make a map of obstacles
for pos in blockPos:
    blockId = p.loadURDF("cube.urdf", pos, blockOrn, useFixedBase=True)

currentVelocity = 5
maximumVelocity = 20

# Wheel joint indices
left_front_wheel = 6
right_front_wheel = 2
left_back_wheel = 7
right_back_wheel = 3

debug_line_center = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[1, 0, 0], lineWidth=3)
debug_line_left = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[0, 1, 0], lineWidth=3)
debug_line_right = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=3)

def set_velocity(left, right):
    p.setJointMotorControl2(robotId, left_front_wheel, p.VELOCITY_CONTROL, targetVelocity=left, force=MAX_FORCE)
    p.setJointMotorControl2(robotId, right_front_wheel, p.VELOCITY_CONTROL, targetVelocity=right, force=MAX_FORCE)
    p.setJointMotorControl2(robotId, left_back_wheel, p.VELOCITY_CONTROL, targetVelocity=left, force=MAX_FORCE)
    p.setJointMotorControl2(robotId, right_back_wheel, p.VELOCITY_CONTROL, targetVelocity=right, force=MAX_FORCE)

def turn_right():
    set_velocity(currentVelocity * (-1), currentVelocity * 2)

def turn_left():
    set_velocity(currentVelocity * 2, currentVelocity * (-1))

def turn_around():
    set_velocity(-currentVelocity * 0.1, currentVelocity * 0.1)

def slow_down():
    set_velocity(SLOW_DOWN_VELOCITY, SLOW_DOWN_VELOCITY)

def start():
     global currentVelocity, maximumVelocity
     if currentVelocity < maximumVelocity:
        currentVelocity *= VELOCITY_INCREMENT

for i in range(STEPS):
    start()
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    euler = p.getEulerFromQuaternion(robotOrn)
    
    # Calculate ray end points based on robot position and orientation
    forward_vector = [-RAY_DISTANCE * math.sin(euler[2]), RAY_DISTANCE * math.cos(euler[2]), 0]
    left_vector = [-RAY_DISTANCE * math.sin(euler[2] + SENSOR_ANGLE), RAY_DISTANCE * math.cos(euler[2] + SENSOR_ANGLE), 0]
    right_vector = [-RAY_DISTANCE * math.sin(euler[2] - SENSOR_ANGLE), RAY_DISTANCE * math.cos(euler[2] - SENSOR_ANGLE), 0]
    
    center_ray_end = [robotPos[0] + forward_vector[0], robotPos[1] + forward_vector[1], robotPos[2]]
    left_ray_end = [robotPos[0] + left_vector[0], robotPos[1] + left_vector[1], robotPos[2]]
    right_ray_end = [robotPos[0] + right_vector[0], robotPos[1] + right_vector[1], robotPos[2]]

    # Perform ray tests
    center_ray_result = p.rayTest(robotPos, center_ray_end)
    left_ray_result = p.rayTest(robotPos, left_ray_end)
    right_ray_result = p.rayTest(robotPos, right_ray_end)

    center_hit = center_ray_result[0][0] not in [-1, planeId, robotId]
    left_hit = left_ray_result[0][0] not in [-1, planeId, robotId]
    right_hit = right_ray_result[0][0] not in [-1, planeId, robotId]

    # Decision making based on sensor readings
    if center_hit:
        if left_hit and right_hit:
            turn_left()
        elif left_hit:
            turn_right()
        elif right_hit:
            turn_left()
        else:
            slow_down()
    elif left_hit and right_hit:
        turn_right()
    elif left_hit:
        turn_right()
    elif right_hit:
        turn_left()
    else:
        set_velocity(currentVelocity, currentVelocity)

    p.addUserDebugLine(robotPos, center_ray_end, lineColorRGB=[1, 0, 0], lineWidth=3, replaceItemUniqueId=debug_line_center)
    p.addUserDebugLine(robotPos, left_ray_end, lineColorRGB=[0, 1, 0], lineWidth=3, replaceItemUniqueId=debug_line_left)
    p.addUserDebugLine(robotPos, right_ray_end, lineColorRGB=[0, 0, 1], lineWidth=3, replaceItemUniqueId=debug_line_right)
    
    p.stepSimulation()
    time.sleep(1 / 240.)

    p.resetDebugVisualizerCamera(
        cameraDistance=5,
        cameraYaw=180,
        cameraPitch=-60,
        cameraTargetPosition=robotPos
    )

p.disconnect()
