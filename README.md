# Robot Obstacle Avoidance - PyBullet

This project implements a robot with obstacle avoidance capabilities using PyBullet physics engine. The robot is equipped with three sensors that allow it to detect and avoid obstacles. The robot navigates by adjusting its movement based on sensor data, ensuring it avoids collisions with obstacles in its path.

## Features

- **Obstacle Avoidance**: The robot detects obstacles using three sensors placed in front, left, and right. It adjusts its velocity and direction to avoid collisions.
- **Sensor Ray Orientation**: The sensors rotate with the robot, ensuring that they always point in the direction the robot is facing.
- **Navigation Logic**: The robot adjusts its speed and direction based on the proximity of obstacles, providing smoother navigation.

## Installation

To run the project, you need to have the following installed:

- **Python 3**
- **PyBullet**: This is the physics engine used for simulating the robot and obstacles.
  - Install PyBullet using pip:
    ```bash
    pip install pybullet
    ```
- **Python Dependencies**: The script uses some standard Python libraries, which should already be available with Python:
  - `time`
  - `math`

## Running the Code

1. Clone the repository or download the files.
2. Install PyBullet (if not already installed):
   ```bash
   pip install pybullet
   ```
3. Run the script:
   ```bash
   python robot.py
   ```

The robot will start navigating in the simulated environment, avoiding obstacles based on the sensor input.

## Improvements

Some potential improvements you might consider implementing in the future:

- **Goal-Oriented Navigation**: Implement functionality to give the robot a specific target position to navigate to, rather than just avoiding obstacles.
- **Sensor Fusion**: Combine data from multiple sensors to improve decision-making and avoid false positives in obstacle detection.
- **Machine Learning for Navigation**: Use reinforcement learning to teach the robot how to navigate and avoid obstacles more efficiently over time.
- **Path Planning Algorithms**: Implement more sophisticated pathfinding algorithms like A* or D* to enable the robot to find optimal paths around obstacles.