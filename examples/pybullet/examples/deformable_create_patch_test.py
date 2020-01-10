#!/usr/bin/env python3

import sys
sys.path.append("/home/dmcconac/pybullet_cloth_testing/build-bullet3-Desktop-Debug/examples/pybullet")

import pybullet as p
import math, time
import difflib, sys
import argparse
import IPython


def setupWorld():
    # p.resetSimulation(p.RESET_USE_DISCRETE_DYNAMICS_WORLD)
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    # p.resetSimulation()
    p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    p.setGravity(0,0,-10)

    planeOrn = [0,0,0,1] #p.getQuaternionFromEuler([0.3,0,0])
    planeId = p.loadURDF("plane.urdf", [0,0,-2], planeOrn)
    boxId = p.loadURDF("cube.urdf", [0,1,2], useMaximalCoordinates = True)
    # clothId = p.loadSoftBody("cloth_z_up.obj",
    #                          basePosition = [0,0,2],
    #                          scale=0.5,
    #                          mass=1.,
    #                          useNeoHookean=0,
    #                          useBendingSprings=1,
    #                          useMassSpring=1,
    #                          springElasticStiffness=40,
    #                          springDampingStiffness=.1,
    #                          useSelfCollision=0,
    #                          frictionCoeff=.5,
    #                          useFaceContact=1)
    numX = 40
    numY = 50
    p.createClothPatchObjFile("/tmp/cloth_patch.obj", [(0.5, 0.5, 0), (-0.5, 0.5, 0), (0.5, -0.5, 0), (-0.5, -0.5, 0)], [numX, numY])
    clothId = p.loadSoftBody("/tmp/cloth_patch.obj",
                             basePosition=[0,0,2],
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

    p.createSoftBodyAnchor(clothId, 0, boxId,-1, [0.5,-0.5,0])
    p.createSoftBodyAnchor(clothId, numX-1, boxId,-1, [-0.5,-0.5,0])
    p.createSoftBodyAnchor(clothId, numX*(numY - 1), -1, -1)
    p.createSoftBodyAnchor(clothId, numX*numY - 1, -1, -1)


if __name__ == "__main__":
    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    setupWorld()
    p.setRealTimeSimulation(1)
    while p.isConnected():
        time.sleep(1./240.)
