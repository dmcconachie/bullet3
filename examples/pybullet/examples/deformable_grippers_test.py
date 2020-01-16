#!/usr/bin/env python3

import sys
# sys.path.append("/home/dmcconac/pybullet_cloth_testing/build-bullet3-Desktop-Debug/examples/pybullet")
sys.path.append("/home/dmcconac/pybullet_cloth_testing/build-bullet3-Desktop-Release/examples/pybullet")

import pybullet as p
import math, time
import difflib, sys
import argparse
import IPython
import numpy as np


def setupWorld():
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setPhysicsEngineParameter(deterministicOverlappingPairs=1,
                                sparseSdfVoxelSize=0.25)
    p.setGravity(0, 0, -10)

    planeOrn = [0, 0, 0, 1] #p.getQuaternionFromEuler([0.3,0,0])
    planeId = p.loadURDF("plane.urdf", [0, 0, -2], planeOrn)
    
    corners = [(1.5, 1.5, 0), (0.5, 1.5, 0), (1.5, 0.5, 0), (0.5, 0.5, 0)]
    numNodes = [40, 40]
    p.createClothPatchObjFile("/tmp/cloth_patch.obj", corners, numNodes)
    clothId = p.loadSoftBody("/tmp/cloth_patch.obj",
                             basePosition=[0, 0, 2.0],
                             scale=1.,
                             mass=1.,
                             useNeoHookean=0,
                             useBendingSprings=1,
                             useMassSpring=1,
                             springElasticStiffness=40,
                             springDampingStiffness=.1,
                             useSelfCollision=0,
                             frictionCoeff=.5,
                             useFaceContact=1)
    p.generateClothPatchDiagonalLinks(clothId, numNodes)
    _, meshData = p.getMeshData(clothId)

    gripper0Id = p.loadURDF("gripper.urdf", meshData[0], [0, 0, 0, 1])
    gripper1Id = p.loadURDF("gripper.urdf", meshData[numNodes[0]-1], [0, 0, 0, 1])
    # Note that the bodyFramePosition value is unused for DEFORMABLE_WORLD
    p.createSoftBodyAnchor(clothId, 0,             gripper0Id, -1)
    p.createSoftBodyAnchor(clothId, numNodes[0]-1, gripper1Id, -1)

    p.setSoftBodyParams(clothId,
                        # generateBendingConstraints=1,
                        # bendingConstraintsDistance=2,
                        linearStiffness=0.1,
                        angularStiffness=0.1)

    return clothId, [gripper0Id, gripper1Id]


if __name__ == "__main__":
    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    clothId, gripperIds = setupWorld()
    
    pos0, orn0 = p.getBasePositionAndOrientation(gripperIds[0])
    pos1, orn1 = p.getBasePositionAndOrientation(gripperIds[1])
    _, meshData = p.getMeshData(clothId)
    
    p.resetDebugVisualizerCamera(cameraDistance=2,
                                 cameraPitch=-35,
                                 cameraYaw=50,
                                 cameraTargetPosition=np.average(np.array([pos0, pos1]), 0))

    # setRealTimeSimulation is fundamentally broken; don't use it
    # - the faster the computation happens, the more likely the amount of elapsed 
    #   in a loop time goes under
    p.setRealTimeSimulation(1)
    while p.isConnected():
        pos0, orn0 = p.getBasePositionAndOrientation(gripperIds[0])
        pos1, orn1 = p.getBasePositionAndOrientation(gripperIds[1])
        p.resetBasePositionAndOrientation(gripperIds[0], np.add(pos0, [0, 0, 0.001]), orn0)
        p.resetBasePositionAndOrientation(gripperIds[1], np.add(pos1, [0, 0, 0.001]), orn1)
        # p.stepSimulation()
        _, meshData = p.getMeshData(clothId)
        print(meshData[30], meshData[30+29])
        time.sleep(1./60.)
