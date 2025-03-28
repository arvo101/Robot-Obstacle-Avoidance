import pybullet as p
import pybullet_data
import time
import math

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0.47]
startOrn = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("r2d2.urdf", startPos, startOrn)

blockPos = [0.6, -5, 0.5]
blockOrn = p.getQuaternionFromEuler([0, 0, 0])
blockId = p.loadURDF("cube.urdf", blockPos, blockOrn, useFixedBase=True)

maxForce = 100
currentVelocity = 5
maximumVelocity = 25

left_front_wheel = 6
right_front_wheel = 2
left_back_wheel = 7
right_back_wheel = 3

steps = 10000

ray_distance = -1.5
sensor_offset_center = 0.0
sensor_offset_left = 0.1
sensor_offset_right = 0.1

turning_factor = 0.0

debug_line_center = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[1, 0, 0], lineWidth=3)
debug_line_left = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[0, 1, 0], lineWidth=3)
debug_line_right = p.addUserDebugLine([0, 0, 0], [0, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=3)

for i in range(steps):
    if currentVelocity < maximumVelocity:
        currentVelocity *= 1.01
    print("Current Velocity: " + str(currentVelocity))

    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    euler = p.getEulerFromQuaternion(robotOrn)

    sensor_position_center = [robotPos[0] + sensor_offset_center, robotPos[1], robotPos[2]]
    sensor_position_left = [robotPos[0] + sensor_offset_left, robotPos[1], robotPos[2]]
    sensor_position_right = [robotPos[0] - sensor_offset_right, robotPos[1], robotPos[2]]

    center_ray_end = [sensor_position_center[0] - ray_distance * math.sin(euler[2]), 
                      sensor_position_center[1] + ray_distance * math.cos(euler[2]),
                      sensor_position_center[2]]
    
    left_ray_end = [sensor_position_left[0] + 0.5 - ray_distance * math.sin(euler[2]),
                    sensor_position_left[1] + ray_distance * math.cos(euler[2]),
                    sensor_position_left[2]]
    
    right_ray_end = [sensor_position_right[0] - 0.5 -ray_distance * math.sin(euler[2]),
                     sensor_position_right[1] + ray_distance * math.cos(euler[2]),
                     sensor_position_right[2]]


    center_ray_result = p.rayTest(sensor_position_center, center_ray_end)
    left_ray_result = p.rayTest(sensor_position_left, left_ray_end)
    right_ray_result = p.rayTest(sensor_position_right, right_ray_end)

    if (center_ray_result[0][0] != -1 and center_ray_result[0][0] != planeId and center_ray_result[0][0] != robotId) or \
    (left_ray_result[0][0] != -1 and left_ray_result[0][0] != planeId and left_ray_result[0][0] != robotId) or \
    (right_ray_result[0][0] != -1 and right_ray_result[0][0] != planeId and right_ray_result[0][0] != robotId):
        print(f"Obstacle located!")


    if currentVelocity == 0:
        print(f"Robot stopped. Obstacle located at {center_ray_result[0][3]}")

    if center_ray_result[0][0] != -1:
        turning_factor = 1
        currentVelocity = 15

    if left_ray_result[0][0] != -1:
        turning_factor = 1
        left_back_wheel_vel = currentVelocity * (1 - turning_factor)
        left_front_wheel_vel = currentVelocity * (1 - turning_factor)
    else:
        turning_factor = 0.0
        left_back_wheel_vel = currentVelocity
        left_front_wheel_vel = currentVelocity

    if right_ray_result[0][0] != -1:
        turning_factor = 1
        right_back_wheel_vel = currentVelocity * (1 - turning_factor)
        right_front_wheel_vel = currentVelocity * (1 - turning_factor)
    else:
        turning_factor = 0.0
        right_back_wheel_vel = currentVelocity
        right_front_wheel_vel = currentVelocity

    print("Turning factor: " + str(turning_factor))

    p.addUserDebugLine(sensor_position_center, center_ray_end, lineColorRGB=[1, 0, 0], lineWidth=3, replaceItemUniqueId=debug_line_center)
    p.addUserDebugLine(sensor_position_left, left_ray_end, lineColorRGB=[0, 1, 0], lineWidth=3, replaceItemUniqueId=debug_line_left)
    p.addUserDebugLine(sensor_position_right, right_ray_end, lineColorRGB=[0, 0, 1], lineWidth=3, replaceItemUniqueId=debug_line_right)

    p.setJointMotorControl2(robotId, left_front_wheel, p.VELOCITY_CONTROL, targetVelocity=left_front_wheel_vel, force=maxForce)
    p.setJointMotorControl2(robotId, right_front_wheel, p.VELOCITY_CONTROL, targetVelocity=right_front_wheel_vel, force=maxForce)
    p.setJointMotorControl2(robotId, left_back_wheel, p.VELOCITY_CONTROL, targetVelocity=left_back_wheel_vel, force=maxForce)
    p.setJointMotorControl2(robotId, right_back_wheel, p.VELOCITY_CONTROL, targetVelocity=right_back_wheel_vel, force=maxForce)

    p.stepSimulation()
    time.sleep(1 / 240.)

    p.resetDebugVisualizerCamera(
        cameraDistance=5,
        cameraYaw=120,
        cameraPitch=-45,
        cameraTargetPosition=robotPos
    )

p.disconnect()
