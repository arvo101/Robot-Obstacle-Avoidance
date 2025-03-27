import pybullet as p
import pybullet_data
import time
from PIL import Image
import numpy as np

physicsClient = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0.47]
startOrn = p.getQuaternionFromEuler([0, 0, 0])

robotId = p.loadURDF("r2d2.urdf", startPos, startOrn)

num_joints = p.getNumJoints(robotId)

maxForce = 100
targetVelocity = 10

left_wheel_joint_index = 6
right_wheel_joint_index = 2
left_rear_wheel_joint_index = 7
right_rear_wheel_joint_index = 3

steps = 2000

positions = []
orientations = []

for i in range(steps):
    p.setJointMotorControl2(robotId, left_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)
    p.setJointMotorControl2(robotId, right_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)
    p.setJointMotorControl2(robotId, left_rear_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)
    p.setJointMotorControl2(robotId, right_rear_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)

    p.stepSimulation()
    time.sleep(1 / 240.)

    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    positions.append(robotPos)
    orientations.append(robotOrn)

    if i % 100 == 0: 
        camera_position = [robotPos[0], robotPos[1], robotPos[2] + 5]
        target_position = [robotPos[0], robotPos[1], robotPos[2]]
        camera_up = [1, 0, 1]

        viewMatrix = p.computeViewMatrix(camera_position, target_position, camera_up)
        projectionMatrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100)

        width, height, rgbImg, _, _ = p.getCameraImage(320, 240, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)
        rgb_array = np.array(rgbImg, dtype=np.uint8).reshape((height, width, 4))

        img = Image.fromarray(rgb_array)
        img.save(f"camera_image_step_{i}.png")

print("Positions:", positions)
print("Orientations:", orientations)

p.disconnect()
