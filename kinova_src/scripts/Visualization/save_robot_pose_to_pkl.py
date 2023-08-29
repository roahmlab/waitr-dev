import os
from pathlib import Path
import time
import math
from datetime import datetime
import json
import numpy as np
import pandas as pd

import pybullet as p
import pybullet_data

from pyBulletSimRecorder import PyBulletRecorder


def create_recorder(names, ids):
    '''Save assets for rendering in Blender'''

    assert len(names) == len(ids)

    recorder = PyBulletRecorder()

    for i in range(len(names)):
        recorder.register_object(ids[i], names[i])

    return recorder


if __name__ == '__main__':
    # setup pybullet
    clid = p.connect(p.SHARED_MEMORY)

    if (clid < 0):
        # options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")
        p.connect(p.GUI, )

    p.resetSimulation()
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.81)
    useRealTimeSimulation = 1
    p.setRealTimeSimulation(useRealTimeSimulation)

    # scene directories
    # TODO: specify scene name
    scene_name = 'contact_diagram_v1'

    # robot name (urdf path)
    # TODO: specify urdf path here
    base_robot_path = '../../urdfs/robots/kinova/meshes/' #
    robot_basename = base_robot_path + 'Kinova_Grasp_Cylinder_Edge'

    # pose
    # TODO: define pose here
    qpos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    if not os.path.exists(f"data/pkl_files/{scene_name}"):
        os.mkdir(f"data/pkl_files/{scene_name}")

    # create empty list for ids
    ids = []
    names = []

    # adjust robot to the pose
    robot_names, robot_ids = [], []
    # robot
    robot_name = robot_basename + '.urdf'
    robot_id = p.loadURDF(robot_name, useFixedBase=True)
    ids.append(robot_id)
    robot_ids.append(robot_id)
    robot_names.append(robot_name)

    # move the arm
    joint_indices = list(range(len(qpos)))
    for i_joint in range(p.getNumJoints(robot_id)):
        p.setCollisionFilterGroupMask(robot_id, i_joint, 0, 0)
    p.setJointMotorControlArray(bodyIndex=robot_id, jointIndices=joint_indices, controlMode=p.POSITION_CONTROL, targetPositions=qpos)
    p.setGravity(0, 0, -9.81)

    # record keyframes
    robot_recorder = create_recorder(robot_names, robot_ids)
    robot_recorder.add_keyframe()

    # dump simulation into pickle file
    robot_recorder.save(f'data/pkl_files/{scene_name}/robot.pkl')

    # remove bodies
    for id in ids:
        p.removeBody(id)
