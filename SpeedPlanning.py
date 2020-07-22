import numpy as np
from MyMath import *


class SpeedPlanning(object):

    def __init__(self, *args):
        """

        :param args:
        """
        self.pose_all = []
        self.MAX_V = 4000.0
        self.max_a = 7000.0   # 加速度
        self.num = 10    # 返回的点个数
        self.nameda = 0.5
        self.max_v_r_cal = None   # 由半经计算的最大速度
        self.max_v_bezier_cal = None #由贝塞尔曲线计算的最大速度
        self.max_v_cai_cla    = None  # 用蔡同学的方法计算得到

        self.v_bezier_cla = None  # 用赛尔曲线根据点比率算得
        self.v_path_r_cla = None  # 根据最大速度及加速度算得
        self.v_cai_cla    = []
        self.x = None
        self.y = None

        for pose in args:
            if isinstance(pose, Pose):
                self.pose_all.append(pose)

    def change_parameter(self, a=None, num=None, nameda=None, max_v = None):
        if isinstance(a, (int, float)):
            self.max_a = a
        if isinstance(num, (float, int)):
            self.num = num
        if isinstance(nameda, (float, int)):
            self.nameda = nameda
        if isinstance(max_v, (int, float)):
            self.MAX_V = max_v

    def cal_max_v(self):
        self.x, self.y = Bezier.get_all(self.pose_all, self.nameda, self.nameda, num=self.num)
        distence = MyMath.distences(self.x, self.y)
        rata = 1000 / distence[self.num - 2]
        self.max_v_bezier_cal = distence * rata

        r = MyMath.r(self.x, self.y)
        max_v = np.sqrt(self.max_a * r)
        self.max_v_r_cal = max_v

    def cal_max_v_cai(self):
        self.x, self.y = Bezier.get_all(self.pose_all, self.nameda, self.nameda, num=self.num)

        distence = MyMath.distences(self.x, self.y)
        self.max_v_cai_cla = []
        v_i = self.pose_all[0].v
        r = MyMath.r(self.x, self.y)
        self.max_v_cai_cla.append(v_i)

        a_all = []
        for i in range(len(r)):
            a_ni = v_i ** 2 / r[i]

            if a_ni > np.float64(self.max_a):
                v_max = np.sqrt(r[i] * self.max_a)
                v_ii = np.minimum(self.MAX_V, v_max)
            else:
                a_all.append(a_ni)
                a_ti = np.sqrt(self.max_a ** 2 - a_ni ** 2)
                v_ii = np.minimum(self.MAX_V, np.sqrt(v_i ** 2 + 2 * a_ti * (distence[i])))

            self.max_v_cai_cla.append(v_ii)
            v_i  = v_ii

        self.max_v_cai_cla.append(self.pose_all[-1].v)
        self.v_cai_cla = self.max_v_cai_cla.copy()
        v0 = self.max_v_cai_cla[-1]

        for i in range(len(distence) - 1, -1, -1):
            x = distence[i]
            vt = np.sqrt(v0 ** 2 + 2 * self.max_a * x)
            min_v = np.minimum(vt, self.max_v_cai_cla[i])
            self.v_cai_cla[i] = min_v
            v0 = min_v

    def cal_path_v(self):
        """

        :param start_v: 开始速度,
        :param end_v: 结束速度
        :return:
        """
        start_v = self.pose_all[0].v
        end_v   = self.pose_all[-1].v

        distences = MyMath.distences(self.x, self.y)
        max_vs    = np.append(self.max_v_r_cal.copy(), end_v)

        # print('len dis, len max_vs:', len(distences), len(max_vs))

        path_v = [start_v]
        for i in range(len(max_vs)):
            vt = np.sqrt(2 * self.max_a * distences[i] + path_v[i] ** 2)
            vt = np.clip(vt, 0, self.MAX_V)

            if vt <= max_vs[i]:
                path_v.append(vt)
            else:
                path_v.append(max_vs[i])
                for j in range(i, -1, -1):
                    vt = np.sqrt(2 * self.max_a * distences[j] + path_v[j + 1] ** 2)
                    if vt < path_v[j]:
                        path_v[j] = vt
                    else:
                        break
        self.v_path_r_cla = np.array(path_v, dtype=np.float)

        return path_v

    def save(self):
        point_num = "uint16_t g_AllPoint_num = {};\n".format(len(self.x))
        define = "TYPE_Point_Info_t g_Points[{}]".format(len(self.x)+2)
        others = """={
//          x                y             v             theta            vx             vy
"""
        head = point_num + define + others
        points = np.zeros((len(self.x), 6), dtype=np.float)
        points[:,0] = self.x
        points[:,1] = self.y
        points[:,2] = self.v_path_r_cla

        np.savetxt('points.txt', points, "%13.5ff,", ' ', '\n',
                   head, '};', '', )


if __name__ == '__main__':
    paths = []

    paths.append(Pose(0, 0, 0, 0))
    paths.append(Pose(1000, 1000, 1000, 90))
    paths.append(Pose(0, 2000, 1000, 180))
    paths.append(Pose(-1000, 1000, 1000, 270))
    paths.append(Pose(0, 0, 0, 0))

    a = SpeedPlanning(*paths)
    a.change_parameter(num=10)
    a.cal_max_v()
    a.cal_path_v()
    a.cal_max_v_cai()
    a.save()

    plt.figure('cai')
    plt.plot(a.max_v_cai_cla)
    plt.plot(a.v_cai_cla)
    #
    plt.figure('my')
    plt.plot([0,*a.max_v_r_cal])
    plt.plot(a.v_path_r_cla)

    plt.figure()
    plt.plot(a.x, a.y)
    plt.scatter(a.x, a.y)
    print(len(a.x), len(a.v_path_r_cla))

    plt.show()












