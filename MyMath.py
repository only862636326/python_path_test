

import numpy as np
import matplotlib.pyplot as plt


class Pose(object):

    def __init__(self, x:'float', y:'float', v:'float', theta = 0):
        """

        :param x:
        :param y:
        :param v:
        :param theta: 运动方向
        """
        self.x = x
        self.y = y
        self.v = v
        self.theta = theta


class Bezier(object):

    @staticmethod
    def get_all(pose_all:'list', namda1=2.0, namda2=2.0, num = 100):
        """
        返回所有的点的路径规则
        :param pose_all: 所有的位姿信息
        :param namda1:
        :param namda2:
        :param num: 每两个点要返回的点的个数
        :return:
        """
        namda2 = - np.abs(namda2)
        for pose in pose_all:
            if isinstance(pose, Pose):
                pass
            else:
                pose_all.remove(pose)

        pose_num = len(pose_all)
        if pose_num < 2:
            return None, None
        else:
            x = []
            y = []
            u = np.linspace(0, 1, num)

            for i in range(pose_num-1):
                x_, y_ = Bezier.get_one(pose_all[i], pose_all[i+1], namda1, namda2, u)
                if i == 0:
                    y = np.append(y, y_)
                    x = np.append(x, x_)
                else:
                    # 去掉后一段与前二段相交的第一个点
                    x = np.append(x, x_[1:])
                    y = np.append(y, y_[1:])
            return x, y

    @staticmethod
    def get_one(start_pose:'Pose', end_pose:'Pose',  namda1, namda2, u):
        """
        两点的路径规划
        :param start_pose:
        :param end_pose:
        :param namda1:
        :param namda2:
        :param u: [0，1]中的等差数组，个数决定返回的点的个数
        :return:
        """
        if isinstance(start_pose, Pose) and isinstance(end_pose, Pose):
            x2 = start_pose.x + namda1 * start_pose.v * np.cos(np.deg2rad(start_pose.theta))
            y2 = start_pose.y + namda1 * start_pose.v * np.sin(np.deg2rad(start_pose.theta))

            x3 = end_pose.x + namda2 * end_pose.v * np.cos(np.deg2rad(end_pose.theta))
            y3 = end_pose.y + namda2 * end_pose.v * np.sin(np.deg2rad(end_pose.theta))

            # print(x2, y2, x3, y3)

            x = start_pose.x * (1 - u) ** 3 + 3 * x2 * (1 - u) ** 2 * u + 3 * x3 * (1 - u) * u ** 2 + end_pose.x * u ** 3
            y = start_pose.y * (1 - u) ** 3 + 3 * y2 * (1 - u) ** 2 * u + 3 * y3 * (1 - u) * u ** 2 + end_pose.y * u ** 3

            return x, y
        else:
            return None, None


class MyMath(object):
    """
    一些用来处理贝塞尔曲线相关的数学函数
    离线的的点处理
    """
    @staticmethod
    def dx(x:'np.ndarray', y:'np.ndarray'):
        """
        求导
        也就是当前点到下一个点的斜率
                没有最后一个点的导数
        :param x: 坐标数组
        :param y: 坐标数组
        :return: 导数数组
        """
        x_ = x[1:]
        y_ = y[1:]

        _x = x[:-1]
        _y = y[:-1]
        derta_y = y_ - _y
        derta_x = x_ - _x

        dx = np.divide(derta_y, derta_x)
        # dx = np.append(dx, dx[-1])
        return dx

    @staticmethod
    def dert(x:'np.ndarray'):
        """
        一个点到下一个点的差值
        :param x:
        :return: 差值数组,没有最后一个点的
        """
        return x[1:] - x[:-1]

    @staticmethod
    def distences(x: 'np.ndarray', y: 'np.ndarray'):
        """
        到下一个点的距离
        :param x:
        :param y:
        :return: 距离数组,没有最后一个点
        """
        x_ = x[1:]
        y_ = y[1:]

        _x = x[:-1]
        _y = y[:-1]
         
        xv2 = (x_ - _x) ** 2
        yv2 = (y_ - _y) ** 2

        return np.sqrt(xv2 + yv2)

    @staticmethod
    def tan_theta(x: 'np.ndarray', y: 'np.ndarray'):
        return MyMath.dx(x, y)

    @staticmethod
    def r(x:'np.ndarray', y:'np.ndarray'):
        """
        离散的点每三点确定的圈的半经
        :param x:
        :param y:
        :return: 半经数组, 没有头尾对应的半经
        """
        distence = MyMath.distences(x, y)
        # print(distence)
        l1 = distence[:-1]
        l2 = distence[1:]
        # print('l1, l2', l1, l2)

        k = MyMath.tan_theta(x, y)
        # print("k : ", k)
        theta = np.arctan(k)
        # print('theta', theta)
        derta_theta = theta[1:] - theta[:-1]
        # print('derta_theta', np.rad2deg(derta_theta))
        tan_theta = np.abs(np.tan(derta_theta))
        # print('tan_theta', tan_theta)
        # r = np.sqrt(l1 ** 2 - (tan_theta * l1) ** 2)
        l_len = ((l1 + l2) / (2 * tan_theta) + np.sqrt((l1 + l2) ** 2 / (4 * tan_theta ** 2) + l1 * l2)) / 2
        r = np.sqrt(l_len ** 2 + (l2 / 2) ** 2)
        return r


if __name__ == '__main__':

    pose_all = []
    num = 10
    # a = 7000
    nameda = 0.8

    pose_all.append(Pose(0,     0    ,1000, 0))
    pose_all.append(Pose(1000, 1000, 1000, 45))
    # pose_all.append(Pose(1000, 1000, 1000, 90))
    # pose_all.append(Pose(1000, 2000, 1000, 180))
    # pose_all.append(Pose(2000,   0,   0, 90))

    x, y = Bezier.get_all(pose_all, nameda, nameda, num=num)
    # distence = MyMath.distences(x, y)
    # rata = 1000 / distence[num-2]
    # v = distence * rata
    # print(v)

    # r = MyMath.r(x, y)
    # v = np.sqrt(a * r)
    # # print('r', r)
    # print('v',  v)
    plt.figure()
    plt.grid()
    plt.scatter(x, y)
    plt.plot(x, y)
    plt.show()
