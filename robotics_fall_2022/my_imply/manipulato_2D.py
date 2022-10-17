import math
import numpy as np
import mujoco
import mujoco_viewer
from matplotlib import pyplot as plt


class Manipulator_2D(object):
    def __init__(self):
        self.model_path = "./model/manipulator.xml"
        self.visual = True
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.done = False

        if self.visual:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data,
                                                     width=1200, height=900)
            # 设置默认相机位置
            self.viewer.cam.azimuth = 90
            self.viewer.cam.elevation = -90
            self.viewer.cam.distance = 5
            self.viewer.cam.lookat = np.array([0.0, 0.0, 0.0])

    def step(self, action):
        pass

    def render(self):
        return self.viewer.render()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)


class Manipulator_2D_fk(Manipulator_2D):
    def __init__(self):
        super(Manipulator_2D_fk, self).__init__()
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
        if self.i == 500:
            self.done = True
        return self.done



class Manipulator_2D_ik(Manipulator_2D):
    def __init__(self):
        super(Manipulator_2D_ik, self).__init__()
        self.cal_ik_path()

    def cal_ik_path(self):
        self.N = 100
        self.theta1 = np.pi / 3
        self.theta2 = -np.pi / 2

        # initialize
        self.data.qpos[0] = self.theta1
        self.data.qpos[1] = self.theta2
        mujoco.mj_forward(self.model, self.data)
        # xpos的位置是相对于世界坐标系的
        position_Q = self.data.site_xpos[0]
        # print(position_Q)
        r = 0.5
        center = np.array([position_Q[0] - r, position_Q[1]])
        phi = np.linspace(0, 2 * np.pi, self.N)
        self.x_ref = center[0] + r * np.cos(phi)
        self.y_ref = center[1] + r * np.sin(phi)
        self.x_all = []
        self.y_all = []
        self.i = 0

    def step(self, action):
        # Compute Jacobian J

        # void mj_jac(const mjModel* m, const mjData* d,
        #     mjtNum* jacp, mjtNum* jacr, const mjtNum point[3], int body);
        position_Q = self.data.site_xpos[0]
        jacp = np.zeros((3, 2))  # 3 is for x,y,z and 2 is for theta1 and theta2
        mujoco.mj_jac(self.model, self.data, jacp, None, position_Q, 2)
        # print(jacp)
        J = jacp[[0, 1], :]
        # print(J)

        # Compute inverse Jacobian Jinv
        Jinv = np.linalg.inv(J)
        # print(Jinv)

        # Compute dX
        # dX = X_ref - X
        dX = np.array([self.x_ref[self.i] - position_Q[0], self.y_ref[self.i] - position_Q[1]])
        # print(dX)

        # Compute dq = Jinv*dX
        dq = Jinv.dot(dX)
        # print(dq)

        self.x_all.append(position_Q[0])
        self.y_all.append(position_Q[1])

        # update theta1 and theta2
        self.theta1 += dq[0]
        self.theta2 += dq[1]

        self.data.qpos[0] = self.theta1
        self.data.qpos[1] = self.theta2
        mujoco.mj_forward(self.model, self.data)
        # mj.mj_step(model, data)
        self.i += 1
        if self.visual:
            self.render()
        if self.i >= self.N:
            plt.figure(1)
            plt.plot(self.x_all, self.y_all, 'bx')
            plt.plot(self.x_ref, self.y_ref, 'r-.')
            plt.ylabel("y")
            plt.xlabel("x")
            plt.gca().set_aspect('equal')
            plt.show(block=False)
            plt.pause(5)
            plt.close()
            self.done = True
        return self.done

