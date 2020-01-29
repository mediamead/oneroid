#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import numpy as np

NP = 4 # number of plates per section
NJ = 4 # number of sections

H = 200
W = 200

import os

BASEDIR = os.path.dirname(__file__)

class Robot:

    def __init__(self, render):
        print("*** Robot(render=%s) inited ***" % render)
        # Start pybullet simulation
        if render:
            p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        else:
            p.connect(p.DIRECT) # don't render

        # load urdf file path (to load 'plane.urdf' from)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # load urdf and set gravity
        p.resetSimulation()

        #self._loadBody("plane.urdf", [0, 0, 3], [0, np.pi, 0])

        self.bodyId = self._loadBody("manipulator.urdf")
        assert(p.getNumJoints(self.bodyId) == NJ * NP * 2 + 1)

        # get id of link the camera is attached to -- the very last joint of the body
        self.cameraLinkId = p.getNumJoints(self.bodyId) - 1
        aspect = W / H
        self.projection_matrix = p.computeProjectionMatrixFOV(120, aspect, 0.01, 100)

        self.targetId = None

    def _loadBody(self, f, startPos=[0, 0, 0], startOrientationEuler=[0, 0, 0]):
        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        f = os.path.join(BASEDIR, f)
        bodyId = p.loadURDF(f, startPos,
                            startOrientation, useFixedBase=1)
        return bodyId

    def setTarget(self, pos):
        if self.targetId is not None:
            p.removeBody(self.targetId)
            self.targetId = None

        self.targetId = self._loadBody("target.urdf", pos)

# --------------------------------------------------------------------

    def _setJointMotorPosition(self, joint, pos):
        p.resetJointState(self.bodyId, joint, pos)
        #p.setJointMotorControl2(self.bodyId,
        #                        joint,
        #                        p.POSITION_CONTROL,
        #                        targetPosition=pos,
        #                        targetVelocity=0,
        #                        force=100,
        #                        positionGain=1000,
        #                        velocityGain=0,
        #                        maxVelocity=5)

    def _setJointPosition(self, sec, pos0, pos1):
        pos0 /= NP
        pos1 /= NP

        for p in range(NP):
            j = (sec * NP + p) * 2
            self._setJointMotorPosition(j, pos0)
            self._setJointMotorPosition(j + 1, pos1)

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
        imgs = p.getCameraImage(W, H, view_matrix, self.projection_matrix)
        assert((W, H) == (imgs[0], imgs[1]))
        # get segmask
        segmask = imgs[4]
        segmask = np.reshape(segmask, (H, W))
        # ---
        # find center mass and mass of the target
        x = y = m = 0
        for i in range(H):
            for j in range(W):
                if segmask[i][j] == self.targetId:
                    x += j
                    y += i
                    m += 1
        if m > 0:
            # map to (-1, 1) range
            dx = 1. - 2.*x/m/W
            dy = 1. - 2.*y/m/H
            #dr = np.sqrt(dx**2 + dy**2)
            return (dx, dy)
        else:
            return None

    # def _print_joints_pos(self):
    #     for i in range(p.getNumJoints(self.bodyId)):
    #         js = p.getJointState(self.bodyId, i)
    #         #print("%4.1f/%4.1f " % (js[0], js[1]), end="")
    #         print("%4.3f " % (js[0]), end="")
    #     print()
    #
    def update(self):
        oc = self.getOffCenter()
        if oc is not None:
            (dx, dy) = oc
            if np.sqrt(dx**2+dy**2) < 0.02:
                qval = 10
                done = True
            else:
                qval = 0
                done = False
        else:
            qval = -100
            done = True
            dx = dy = 0

        self.qval = qval
        self.done = done
        self.dx = dx
        self.dy = dy

    def get_nphis(self):
        return NJ

    # step through the simluation
    def step(self, phis):
        for i in range(NJ):
            self._setJointPosition(i, phis[i*2], phis[i*2+1])
        p.stepSimulation()
        self.update()

        #t += timeStep
        #print("%.2f" % t)

    def close(self):
        p.disconnect()
        print("*** Robot() closed ***")

if __name__ == "__main__":
    t = 0
    sps = 240
    timeStep = 1./sps # default Bullet's timestep

    #LOGFILE = "log10s.txt"
    #logf = open(LOGFILE, "w")

    gui = True
    r = Robot(render=gui)

    # Target and manipulator are both NNE
    r.setTarget([1.5, 1.5, 1])

    if gui:
        s_t_phi = p.addUserDebugParameter("t_phi", -1.5, 1.5, 0)

        phiSliders = []
        for i in range(NJ * 2):
            title = "%d:%d" % (i / 2, i % 2)
            s = p.addUserDebugParameter(title, -1.5, 1.5, 0)
            phiSliders.append(s)
    else:
        phis = []
        for i in range(NJ * 2):
            phis.append(0)

    while True:
        if gui:
            TR = 3
            t_phi = p.readUserDebugParameter(s_t_phi)
            t_x = TR * np.sin(t_phi)
            t_y = 0
            t_z = TR * np.cos(t_phi)
            r.setTarget([t_x, t_y, t_z])

            phis = []
            for s in phiSliders:
                phi = p.readUserDebugParameter(s)
                phis.append(phi)

        r.step(phis)

        t += timeStep
        print("t=%f oc=%s, qval=%f, done=%s" % (t, (r.dx, r.dy), r.qval, r.done))

        #print("%f %f" % (t, reward), file=logf)

    #logf.close()