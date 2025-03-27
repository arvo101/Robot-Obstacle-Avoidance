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

angular_velocity = 1
max_force = 50

positions = []
orientations = []

wheel_joints = [2, 3, 6, 7]

for i in range(1000):
    for joint in wheel_joints:
        p.setJointMotorControl2(
            robotId, joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=angular_velocity, force=max_force
        )
    
    p.stepSimulation()
    time.sleep(1 / 240.)

    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
    positions.append(robotPos)
    orientations.append(robotOrn)

    camera_position = [robotPos[0], robotPos[1], robotPos[2] + 5]
    target_position = [robotPos[0], robotPos[1], robotPos[2]]
    camera_up = [1, 0, 0]

    viewMatrix = p.computeViewMatrix(camera_position, target_position, camera_up)
    projectionMatrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100)

    width, height, rgbImg, depthImg, segmentationImg = p.getCameraImage(320, 240,
                                                                        viewMatrix=viewMatrix,
                                                                        projectionMatrix=projectionMatrix)
    rgb_array = np.array(rgbImg, dtype=np.uint8).reshape((height, width, 4))

    img = Image.fromarray(rgb_array)
    img.save(f"camera_image_step_{i}.png")


p.disconnect()
