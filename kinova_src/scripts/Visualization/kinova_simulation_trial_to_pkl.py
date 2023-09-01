import pybullet as p
import pybullet_data
import numpy as np
import csv
import pickle
from recorder.pyBulletSimRecorder import PyBulletRecorder

# Initialize PyBullet and set up simulation parameters
p.connect(p.GUI)  # or p.DIRECT for non-graphical simulation
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)
p.setGravity(0, 0, 0)  # Set gravity in the z direction

# Load robot URDF and get robot ID
robot_urdf_path = "/home/baiyuew/ROAHM/waitr-dev/urdfs/kinova/Kinova_Grasp_Cylinder_Edge.urdf" # Kinova_Grasp_Cylinder_Edge
robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # Roll, Pitch, Yaw
print("Loading robot from:", robot_urdf_path)
robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0], robot_start_orientation,useFixedBase=1)

# Get the number of joints in your robot
num_joints = p.getNumJoints(robot_id)

# Load trajectory from CSV
trajectory = []
with open("robot_trajectory.csv", "r") as csv_file:
    csv_reader = csv.reader(csv_file)
    num_joints = p.getNumJoints(robot_id)  # Get the number of joints in your robot
    for row in csv_reader:
        target_pos = np.array(row[:num_joints], dtype=float)  # Extract joint positions from CSV
        # Append zeros to match the number of joints
        zeros_to_append = np.zeros(num_joints - len(target_pos))
        trajectory.append(np.concatenate((target_pos, zeros_to_append)))

# Load obstacles from CSV
obstacles = []
with open("obstacles.csv", "r") as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        obstacles.append(np.array(row, dtype=float))

# Initialize the recorder
recorder = PyBulletRecorder()
# Load cube URDF for obstacles
cube_urdf_path = "cube.urdf"

# Add obstacles to the simulation
for obstacle in obstacles:
    obstacle_pos = obstacle[:3]
    obstacle_size = obstacle[6:9]  # Length, Width, Height
    obstacle_ori = p.getQuaternionFromEuler([0, 0, 0])  # Roll, Pitch, Yaw
    obstacle_id = p.loadURDF(cube_urdf_path, obstacle_pos, obstacle_ori) # , globalScaling=obstacle_size)
    recorder.register_object(obstacle_id,cube_urdf_path,obstacle_size)

# # Create and add obstacles to the simulation
# for obstacle in obstacles:
#     obstacle_pos = obstacle[:3]
#     obstacle_size = obstacle[6:9]  # Length, Width, Height
#     half_extents = np.array(obstacle_size) * 0.5  # Divide by 2 for half extents
#     obstacle_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
#     obstacle_position = np.array(obstacle_pos) + half_extents  # Correct position based on half extents
#     obstacle_orientation = p.getQuaternionFromEuler(obstacle[3:6])
#     p.createMultiBody(baseCollisionShapeIndex=obstacle_id, basePosition=obstacle_position, baseOrientation=obstacle_orientation)


# Start recording
recorder.register_object(robot_id, robot_urdf_path)

# Simulate the robot's motion along the trajectory
for target_pos in trajectory:

    index = 0
    for i in range(num_joints):
            jointInfo = p.getJointInfo(robot_id, i)
            # assign values to only revolute joints
            if jointInfo[3] > -1:
                p.resetJointState(robot_id, i, target_pos[index])
                index += 1
    recorder.add_keyframe()

    # if len(target_pos) == num_joints:  # Make sure the target positions match the number of joints
    #     p.stepSimulation()
    #     p.setJointMotorControlArray(
    #         bodyUniqueId=robot_id,
    #         jointIndices=list(range(num_joints)),  # Use the correct number of joint indices
    #         controlMode=p.POSITION_CONTROL,
    #         targetPositions=target_pos
    #     )
    #     recorder.add_keyframe()  # Capture the current frame
    # else:
    #     print(f"Target positions should have {num_joints} elements, but {len(target_pos)} provided.")

# Stop recording
# recorder.stopRecording()

# Save recorded data to a .pkl file
recorder.save('demo.pkl')

# Disconnect from the simulation
p.disconnect()
