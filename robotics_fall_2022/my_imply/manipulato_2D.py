import math
import numpy as np
import mujoco
import mujoco_viewer


class Manipulator_2D_fk():
    def __init__(self):
        print("init end")
        self.model_path = "./model/manipulator.xml"
        self.visual = True
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        if self.visual:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        self.cal_fk_path()

    def cal_fk_path(self):
        N = 500
        q0_start = 0
        q0_end = 1.57
        q1_start = 0
        q1_end = 1.57
        self.q0 = np.linspace(q0_start, q0_end, N)
        self.q1 = np.linspace(q1_start, q1_end, N)
        self.i = 0

    def step(self, action):
        self.data.qpos[0] = self.q0[self.i]
        self.data.qpos[1] = self.q1[self.i]
        mujoco.mj_forward(self.model, self.data)
        self.i += 1
        if self.visual:
            self.render()


    def render(self):
        return self.viewer.render()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
