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

    planeOrn = [0,0,0,1] #p.getQuaternionFromEuler([0.3,0,0])
    planeId = p.loadURDF("plane.urdf", [0,0,-2], planeOrn)
    boxId = p.loadURDF("cube.urdf", [0,1,2], useMaximalCoordinates = True)
    clothId = p.loadSoftBody("cloth_z_up.obj",
                             basePosition = [0,0,2],
                             scale = 0.5,
                             mass = 1.,
                             useNeoHookean = 0,
                             useBendingSprings=1,
                             useMassSpring=1,
                             springElasticStiffness=40,
                             springDampingStiffness=.1,
                             useSelfCollision = 0,
                             frictionCoeff = .5,
                             useFaceContact=1)

    p.createSoftBodyAnchor(clothId, 0, -1, -1)
    p.createSoftBodyAnchor(clothId, 1, -1, -1)
    p.createSoftBodyAnchor(clothId, 3, boxId,-1, [0.5,-0.5,0])
    p.createSoftBodyAnchor(clothId, 2, boxId,-1, [-0.5,-0.5,0])
    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    p.setGravity(0,0,-10)


def dumpStateToFile(filename):
    # IPython.embed()
    with open(filename, "w") as file:
        for i in range(p.getNumBodies()):
            bodyId = p.getBodyUniqueId(i)
            baseName, bodyName = p.getBodyInfo(bodyId)
            pos, orn = p.getBasePositionAndOrientation(bodyId)
            linVel, angVel = p.getBaseVelocity(bodyId)
            deformPos = p.getSoftBodyPositions(bodyId)
            deformVel = p.getSoftBodyVelocities(bodyId)

            # numJoints = p.getNumJoints(bodyId)
            # print(bodyName, numJoints)

            txtName = "name = " + str(bodyName) + "\n"
            txtPos =  "pos = " + str(pos) + "\n"
            txtOrn =  "orn = " + str(orn) + "\n"
            txtLinVel = "linVel = " + str(linVel) + "\n"
            txtAngVel = "angVel = " + str(angVel) + "\n"
            txtDeformPos = "deformPos = " + str(deformPos) + "\n"
            txtDeformVel = "deformVel = " + str(deformVel) + "\n"
            file.write(txtPos)
            file.write(txtOrn)
            file.write(txtLinVel)
            file.write(txtAngVel)
            file.write(txtDeformPos)
            file.write(txtDeformVel)
        # for i in range(p.getNumConstraints()):
            # constraintId = p.getConstraintUniqueId(i)
            # parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointType, axis, \
            #     parentFramePosition, childFramePosition, parentFrameOrientation, childFrameOrientation \
            #     maxAppliedForce, gearRatio, gearAuxLink, relativePositionTarget, erp = p.getConstraintInfo(constraintId)
            # print(p.getConstraintInfo(constraintId))
            


def compareFiles(filename1, filename2):
    with open(filename1, "r") as file1, open(filename2, "r") as file2:
        diff = difflib.unified_diff(
            file1.readlines(),
            file2.readlines(),
            fromfile='saveFile.txt',
            tofile='restoreFile.txt',
        )
        # TODO: fix this counting method, doesn't actualy count lines
        numDifferences = 0
        for line in diff:
            # Add 2 newlines for cleaner output
            if numDifferences == 0:
                print("\n")
            numDifferences = numDifferences + 1
            sys.stdout.write(line)
        
        if (numDifferences > 0):
            print("Error:", numDifferences, " lines are different between files.")
        else:
            print("OK, files are identical")


def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial-steps", type=int, default=100)
    parser.add_argument("--restore-saved-state", action="store_true")
    parser.add_argument("--steps-after-restore", type=int, default=100)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()

    if args.render:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    setupWorld()
    for i in range(args.initial_steps):
        p.stepSimulation()
    p.saveBullet(str(args.initial_steps) + "steps.pybullet")
    p.saveState()
    dumpStateToFile(str(args.initial_steps) + "steps_raw.txt")

    total_steps = args.initial_steps + args.steps_after_restore
    for i in range(args.steps_after_restore):
        p.stepSimulation()
    p.saveBullet(str(total_steps) + "steps_raw.pybullet")
    dumpStateToFile(str(total_steps) + "steps_raw.txt")

    if args.restore_saved_state:
        setupWorld()
        p.restoreState(fileName=str(args.initial_steps) + "steps.pybullet")
        dumpStateToFile(str(args.initial_steps) + "steps_restored.txt")
        print("Comparing", args.initial_steps, "steps pre and post restore... ", end="")
        compareFiles(str(args.initial_steps) + "steps_raw.txt", str(args.initial_steps) + "steps_restored.txt")

        for i in range(args.steps_after_restore):
            p.stepSimulation()
        p.saveBullet(str(total_steps) + "steps_restored.pybullet")
        dumpStateToFile(str(total_steps) + "steps_restored.txt")

        print("Comparing", total_steps, "steps pre and post restore... ", end="")
        compareFiles(str(total_steps) + "steps_raw.txt", str(total_steps) + "steps_restored.txt")

    # p.setRealTimeSimulation(1)
    if args.render:
        while p.isConnected():
            time.sleep(1./240.)
