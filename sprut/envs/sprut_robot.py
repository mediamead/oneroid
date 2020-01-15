#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import numpy as np

NP = 4 # number of plates per section
SPS = 240

class Robot:
    NJ = 2 # number of sections

    def __init__(self):
        # Start pybullet simulation
        p.connect(p.GUI)
        #p.connect(p.DIRECT) # don't render

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)

        from os.path import dirname, abspath
        cwd = dirname(abspath(__file__))
        # load urdf file path (to load 'plane.urdf' from)
        #p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(cwd)

        #p.setAdditionalSearchPath(parent_dir)

    def reset(self, tPos):
        p.resetSimulation()

        self._loadBody("plane.urdf", [0, 0, 3], [0, np.pi, 0])

        self.bodyId = self._loadBody("manipulator.urdf")
        assert(p.getNumJoints(self.bodyId) == self.NJ * NP * 2 + 1)

        # get id of link the camera is attached to -- the very last joint of the body
        self.cameraLinkId = p.getNumJoints(self.bodyId) - 1
        self.projection_matrix = p.computeProjectionMatrixFOV(120, 1.0, 0.01, 100)

        self.targetId = self._loadBody("target.urdf", [tPos[0], tPos[1], 3])

    def _loadBody(self, f, startPos=[0, 0, 0], startOrientationEuler=[0, 0, 0]):
        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        bodyId = p.loadURDF(f, startPos,
                            startOrientation, useFixedBase=1)
        return bodyId

# --------------------------------------------------------------------

    def setJointMotorPosition(self, joint, pos):
        p.setJointMotorControl2(self.bodyId,
                                joint,
                                p.POSITION_CONTROL,
                                targetPosition=pos,
                                targetVelocity=0,
                                force=1000,
                                positionGain=1,
                                velocityGain=0,
                                maxVelocity=1)

    def setJointPosition(self, sec, pos0, pos1):
        pos0 /= NP
        pos1 /= NP

        for p in range(NP):
            j = (sec * NP + p) * 2
            self.setJointMotorPosition(j, pos0)
            self.setJointMotorPosition(j + 1, pos1)

# --------------------------------------------------------------------

    def getOffCenter(self):
        # Center of mass position and orientation of the camera box
        cam_p, cam_o, _, _, _, _ = p.getLinkState(self.bodyId, self.cameraLinkId)
        rot_matrix = p.getMatrixFromQuaternion(cam_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (0, 0, 1)  # z-axis
        init_up_vector = (1, 0, 0)  # x-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(
            cam_p, cam_p + 0.1 * camera_vector, up_vector)
        imgs = p.getCameraImage(200, 200, view_matrix, self.projection_matrix)
        (w, h) = (imgs[0], imgs[1])
        # get segmask
        segmask = imgs[4]
        segmask = np.reshape(segmask, (h, w))
        # ---
        # find center mass and mass of the target
        x = y = m = 0
        for i in range(h):
            for j in range(w):
                if segmask[i][j] == self.targetId:
                    x += j
                    y += i
                    m += 1
        if m > 0:
            x = x / m / w - 1/2
            y = y / m / h - 1/2
            #print("mass = %3d, target = %.3f %.3f" % (m, x, y))
            offCenter = np.sqrt(x**2 + y**2)
            return (x, y, m, offCenter)
        else:
            return None

    def _print_joints_pos(self):
        for i in range(p.getNumJoints(self.bodyId)):
            js = p.getJointState(self.bodyId, i)
            #print("%4.1f/%4.1f " % (js[0], js[1]), end="")
            print("%4.3f " % (js[0]), end="")
        print()

    # -----

    def get_nphis(self):
        return self.NJ * 2

    # step through the simluation
    def step(self, phis, nsteps=1):
        for i in range(self.NJ):
            self.setJointPosition(i, phis[i*2], phis[i*2+1])
        self.idle(nsteps)
    
    def idle(self, nsteps=1):
        for _ in range(nsteps):
            p.stepSimulation()

        #t += timeStep
        #print("%.2f" % t)

    def __done__(self):
        p.disconnect()

    # step and calculate the reward
    def getRewardDone(self, offCenter0, offCenter1):
        deltaOff = offCenter1 - offCenter0
        done = False
        if deltaOff < 0:
            # offset goes down
            reward = - deltaOff * 1000
        else:
            reward = - deltaOff * 1000 * 3

        if offCenter1 <= 0.04:
            done = True

        return reward, done

if __name__ == "__main__":
    timeStep = 1./240 # default Bullet's timestep
    
    LOGFILE = "log10s.txt"
    logf = open(LOGFILE, "w")

    r = Robot()

    # Target and manipulator are both NNE
    r.reset([0.5, 0.25, 3])
    phis = [0.1, 0.05, 0.1, 0.05]

    nsteps = int(SPS / 10) # want to get 10 pfs

    t = 0
    (_, _, _, offCenter0) = r.getOffCenter()
    r.step(phis)
    done = False

    while not done:
        t += timeStep * nsteps
        r.idle(nsteps)
        (x, y, _, offCenter1) = r.getOffCenter()
        (reward, done) = r.getRewardDone(offCenter0, offCenter1)
        deltaOff = offCenter1 - offCenter0

        log = "%f [%f %f | %s %f %s] %f" % (t, x, y, offCenter0, deltaOff, offCenter1, reward)
        print(log, file=logf)
        print(log)

        offCenter0 = offCenter1

    logf.close()
