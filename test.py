import pybullet as p
import pybullet_data
import time
import numpy as np
from PIL import Image

physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0.47]
startOrn = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("r2d2.urdf", startPos, startOrn)

wheel_joints = [2, 3, 6, 7]  # Joint indices for wheels
wheel_velocity = 10.0

steps = 100
positions = []
orientations = []

for joint in wheel_joints:
    p.setJointMotorControl2(robotId, joint, p.VELOCITY_CONTROL, targetVelocity=wheel_velocity)

for i in range(steps):
    p.stepSimulation()
    time.sleep(1 / 240.)

    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    positions.append(robotPos)
    orientations.append(robotOrn)

    viewMatrix = p.computeViewMatrix([robotPos[0], robotPos[1], robotPos[2] + 5],
                                     [robotPos[0], robotPos[1], robotPos[2]],
                                     [1, 0, 0])
    
    projectionMatrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100)
    width, height, rgbImg, depthImg, segmentationImg = p.getCameraImage(320, 240, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)
    
    rgb_array = np.array(rgbImg, dtype=np.uint8).reshape((height, width, 4))
    img = Image.fromarray(rgb_array)
    img.save(f"camera_image_step_{i}.png")

print("Positions:", positions)
print("Orientations:", orientations)

p.disconnect()
