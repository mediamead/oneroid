#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data
import numpy as np

import cv2

#H = 720
#W = 1280
#H = 144
#W = 256
# https://www.chiefdelphi.com/t/horizontal-fov-of-microsoft-lifecam-cinema/156204/7
HFOV = 40 # 64.4

import os

BASEDIR = os.path.dirname(__file__)

class PyBulletRobot(object):
    # camera resolution
    H = 1080
    W = 1920
    # chessboard pattern tile size
    D = 0.0423 # TV & VR

    def __init__(self, NS, NP, render=False):
        self.NS = NS
        self.NP = NP

        print("*** Initializing PyBulletRobot(ns=%d, render=%s) ..." % (self.NS, render))
        # Start pybullet simulation
        if render:
            p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
        else:
            p.connect(p.DIRECT) # don't render

        # load urdf file path (to load 'plane.urdf' from)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # load urdf and set gravity
        p.resetSimulation()
        self._loadBody("plane.urdf", [0, 0, 0], [0, 0, 0])
        self._loadBody("chessboard.urdf", [2, 0, 0.59], [np.pi/2, -np.pi/2, -np.pi/2])        
        #self._loadBody("urdfs/plane.urdf", [0, 0, 3], [0, np.pi, 0])
        #self._loadBody("urdfs/green-line.urdf", [1.5, 0, 0.5], [np.pi/2, 0, 0])

        self.bodyId = self._loadBody("manipulator-%d-%d.urdf" % (self.NS, self.NP))
        assert(p.getNumJoints(self.bodyId) == self.NS * self.NP * 3 + 1)

        # get id of link the camera is attached to -- the very last joint of the body
        self.cameraLinkId = p.getNumJoints(self.bodyId) - 1

        # configure the camera
        aspect = self.W / self.H
        self.projection_matrix = p.computeProjectionMatrixFOV(HFOV, aspect, 0.1, 3)

        self.targetId = None
        print("*** Initializing PyBulletRobot() done")

    def _loadBody(self, f, startPos=[0, 0, 0], startOrientationEuler=[0, 0, 0]):
        startOrientation = p.getQuaternionFromEuler(startOrientationEuler)
        f = os.path.join(BASEDIR, os.path.join("urdfs", f))
        bodyId = p.loadURDF(f, startPos,
                            startOrientation, useFixedBase=1)
        return bodyId

    def setTarget(self, pos):
        if self.targetId is not None:
            p.removeBody(self.targetId)
            self.targetId = None

        self.targetId = self._loadBody("target.urdf", pos)

    def addHeadposMarker(self, pos):
        self._loadBody("urdfs/marker.urdf", pos)

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
        ##("## PYBULLETROBOT: joint=%d pos=%.3f" % (joint, pos))

    def _setJointPosition(self, sec, pos0, pos1):
        j = (sec * self.NP) * 3
        self._setJointMotorPosition(j, pos0)
        self._setJointMotorPosition(j + 1, pos1)

# --------------------------------------------------------------------

    def euler2orn(self, alpha, beta, gamma):
        return list(p.getQuaternionFromEuler([alpha, beta, gamma]))

    def getHeadcamPO(self):
        # Center of mass position and orientation of the camera box
        cam_p, cam_o, _, _, _, _ = p.getLinkState(self.bodyId, self.cameraLinkId)
        return list(cam_p), list(cam_o)
    
    def getHeadcamPVU(self):
        cam_p, cam_o = self.getHeadcamPO()
        cam_v, cam_u = self.orn2vu(cam_o)
        return [cam_p, cam_v, cam_u]

    def orn2vu(sel, cam_o):
        rot_matrix = p.getMatrixFromQuaternion(cam_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Initial vectors
        init_camera_vector = (0, 0, 1)  # z-axis
        init_up_vector = (-1, 0, 0)  # x-axis

        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)

        return camera_vector, up_vector

    def getCameraImages(self, pvu=None):
        """
        Get image given Position, Vector, and Up
        """
        if pvu is None:
            pvu = self.getHeadcamPVU()
            (cam_p, camera_vector, up_vector) = pvu
        else:
            cam_p = np.array(pvu[0])
            camera_vector = np.array(pvu[1])
            up_vector = np.array(pvu[2])

        view_matrix = p.computeViewMatrix(
            cam_p, cam_p + 0.1 * camera_vector, up_vector)
        imgs = p.getCameraImage(self.W, self.H, view_matrix, self.projection_matrix)
        assert((self.W, self.H) == (imgs[0], imgs[1]))

        return imgs

    def getCameraImage(self, pvu=None, segmask=False):
        imgs = self.getCameraImages()
        rgba = np.reshape(imgs[2], (self.H, self.W, 4)).astype(np.uint8)
        img = cv2.merge((rgba[:,:,2], rgba[:,:,1], rgba[:,:,0])) # take BGR from RBGA

        return img

    def getOffCenter(self):
        segmask = self.getCameraImages()[4]
        # get segmask
        #segmask = imgs[4]
        #print(imgs.shape)
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

    def getImbalance(self):
        R = np.zeros(2)
        for i in range(p.getNumJoints(self.bodyId)):
            r = p.getLinkState(self.bodyId, i)[0] # linkWorldPosition
            #n = p.getJointInfo(self.bodyId, i)[12].decode('UTF-8')
            R += [r[0], r[1]] # take only XJ proj
        return np.linalg.norm(R)

    def _print_joints_pos(self):
        for i in range(p.getNumJoints(self.bodyId)):
            js = p.getJointState(self.bodyId, i)
            pos, orn, _, _, _, _ = p.getLinkState(self.bodyId, i)

            rot_matrix = p.getMatrixFromQuaternion(orn)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)
            v = rot_matrix.dot((0, 0, 1))

            #print("#J%d %f" % (i, js[0]))
            #print("#B%d %s %s" % (i, pos, v))

    # def updateQ(self):
    #     oc = self.getOffCenter()
    #     if oc is not None:
    #         self.target_found = True
    #         (dx, dy) = oc
    #         dr = np.sqrt(dx**2+dy**2)

    #         if dr < 0.02:
    #             qval = 10
    #             done = True
    #         else:
    #             qval = 0
    #             done = False
    #     else:
    #         self.target_found = False
    #         qval = -100
    #         done = True
    #         dx = dy = dr = 0

    #     self.qval = qval
    #     self.done = done
    #     self.dx = dx
    #     self.dy = dy
    #     self.dr = dr

    # step through the simluation
    def step(self, phis):
        for i in range(self.NS):
            self._setJointPosition(i, phis[i, 0], phis[i, 1])
        p.stepSimulation()

        #t += timeStep
        #print("%.2f" % t)

    def close(self):
        p.disconnect()
        print("*** PyBulletRobot() closed ***")

    def stepSimulation(self):
        p.stepSimulation()

if __name__ == "__main__":
    r = PyBulletRobot(4, 4)
    ls = np.array([[0,0],[0,0],[0,0],[0,0]], dtype=np.float32)
    if True:
        print("# ls=%s" % ls)
        r.step(ls)
        (p, v, u) = r.getHeadcamPVU()
        print("p=%s v=%s u=%s" % (p, v, u))