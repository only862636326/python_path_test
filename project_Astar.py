import numpy as np
import random
import matplotlib.pyplot as plt
import copy
import math
import os
import MyMath

plt.rcParams['font.sans-serif']=['SimHei']
plt.rcParams['axes.unicode_minus'] = False

class point:
    path = 0
    value = 0
    parent = 0
    flag = 1
    def __init__(self,x,y):
        self.x = x
        self.y = y


# 判断是不是终点状态
def is_end(a,b):
    if a.x==b.x and a.y==b.y:
        return True
    else:
        return False


# 列出当前节点的下一个节点
def list_next(current):
    temp1 = []
    for i in range(-1,2):
        for j in range(-1,2):
            if 0 <= current.x+i <= length - 1 and 0 <= current.y+j <= width - 1 and G[current.x+i][current.y+j] == 0:
                temp1.append(point(current.x+i,current.y+j))
    return temp1


# 判断节点是不是在open表中
def in_open(open,next_point):
    for i in range(len(open)):
        if open[i].x == next_point.x and open[i].y == next_point.y:
            return True
    return False


# 计算当前节点和下一个节点的距离 1,1.414
def count_path(current, next_point):
    if current.x == next_point.x or current.y == next_point.y:
        return 1
    else:
        return math.sqrt(2)


# 估价函数，表示离终点的欧几里得距离
def h(next_point):
    a = abs(next_point.x - end.x)
    b = abs(next_point.y - end.y)
    return math.sqrt(a**2+b**2)


# 表示open的内容 测试使用
def show_open(open):
    list = []
    for i in range(len(open)):
        list.append([open[i].x,open[i].y])
        print("({},{})".format(open[i].x,open[i].y),end="")
    print("")
    for i in range(len(open)):
        print("({:.3f},{:.3f})".format(open[i].path, open[i].value), end="")
    print("")
    return list


# 在closed表中逆序展示路径
def show_path(closed,current):
    list1 = []
    list1.append(current)
    i = current.parent
    while i:
        list1.append(closed[i])
        i = closed[i].parent
    list1.append(closed[i])
    list1.reverse()
    listx = []
    listy = []
    path_current = []
    for i in range(len(list1)):
        listx.append(list1[i].y)
        listy.append(list1[i].x)
        path_current.append([list1[i].x,list1[i].y])
    plt.plot(listx, listy, c="red", label='初始路径')
    print("初始路径",path_current)
    print("初始路径长度",path_length(path_current))

    for i in range(len(list1)-2):
        if list1[i].flag == 1:
            for j in range(len(list1)-1, i+1, -1):
                if list1[j].flag == 1:
                    # print("a,b", list1[i].x, list1[i].y, "  ", list1[j].x, list1[j].y)
                    if obstacle(list1[i], list1[j]) == False:
                        continue
                    else:
                        for k in range(i+1, j):
                            list1[k].flag = 0
                        break

    listx = []
    listy = []
    path = []
    for i in range(len(list1)):
        if list1[i].flag == 1:
            listx.append(list1[i].y)
            listy.append(list1[i].x)
            path.append([list1[i].x,list1[i].y])
    plt.plot(listx, listy, c="blue", label='拟合路径')
    print("新路径",path)
    print("新路径长度",path_length(path))

    pose_all = []
    num = 10
    nameda = 0.6
    print('angle')
    last_angle = 0
    for i in range(0,len(path)-1):
        if i == 0:
            pose_all.append(MyMath.Pose(start.x, start.y, 0))
        else:
            # if path[i+1][1] == path[i][1]:
            #     if path[i+1][0] < path[i][0]:
            #         angle = 90
            #     else:
            #         angle = -90
            # else:
            angle = math.degrees(math.atan(-(path[i+1][0]-path[i][0])/(path[i+1][1]-path[i][1])))
            # print(angle)
            pose_all.append(MyMath.Pose(path[i][0], path[i][1], 1, (angle + last_angle)/2 + 90))
            last_angle = angle
    pose_all.append(MyMath.Pose(path[len(path)-1][0], path[len(path)-1][1], 0, 0))
    
    x, y = MyMath.Bezier.get_all(pose_all, nameda, nameda, num=num)
    # plt.scatter(y, x)
    plt.plot(y, x, c='yellow')


def obstacle(a,b):
    for i in range(min(a.x,b.x), max(a.x, b.x)+1):
        for j in range(min(a.y,b.y), max(a.y, b.y)+1):
            if G[i][j] == 1:
                # print("dis",getDistance(j,i,a.y,a.x,b.y,b.x))
                if(getDistance(j,i,a.y,a.x,b.y,b.x) <= round(1/math.sqrt(2),5)):
                    return False
    return True


def getDistance(pointX, pointY, lineX1, lineY1, lineX2, lineY2):
    a = lineY2 - lineY1
    b = lineX1 - lineX2
    c = lineX2 * lineY1 - lineX1 * lineY2
    dis = (math.fabs(a * pointX + b * pointY + c)) / (math.pow(a * a + b * b, 0.5))
    return dis


def path_length(list_test):
    length = 0
    for i in range(len(list_test)-1):
        length += math.sqrt((list_test[i][0]-list_test[i+1][0])**2+(list_test[i][1]-list_test[i+1][1])**2)

    return length


def A_star(start,end):
    open = []
    closed = []
    open.append(start)
    while len(open):
        current = copy.deepcopy(open[0])
        open.pop(0)
        # 判断是不是终点
        if is_end(current,end):
            print("已经找到目标")
            show_path(closed,current)
            return True
        # 列出当前节点的所有子状态
        next_point = []
        next_point = list_next(current)
        # 画出当前节点
        plt.plot(current.y, current.x, "xc")
        # plt.pause(0.001)

        if len(next_point) == 0:
            continue
        for i in range(len(next_point)):
            flag1 = 0
            flag2 = 0
            next_point[i].path = current.path + count_path(current, next_point[i])
            # 若在open表中，比较已经走过的距离，若短就替换
            for j in range(len(open)):
                if open[j].x == next_point[i].x and open[j].y == next_point[i].y:
                    flag1 = 1
                    if open[j].path > next_point[i].path:
                        open[j].path = next_point[i].path
            # 若在closed表中，标志位置1
            for j in range(len(closed)):
                if closed[j].x == next_point[i].x and closed[j].y == next_point[i].y:
                    flag2 = 1
                    break
            # 若不在open表也不再closed表，就在open表中添加这个节点
            if flag1 == 0 and flag2 == 0:
                next_point[i].value = next_point[i].path + h(next_point[i])
                next_point[i].parent = len(closed)
                open.append(next_point[i])

        closed.append(current)
        open = sorted(open, key=lambda x: x.value)
        # print("open")
        # show_open(open)

    print("没找到终点")


if __name__ == '__main__':
    # length, width = map(int, input("输入地图的长宽(20 20): ").split(" "))
    # startx, starty = map(int, input("输入起点(0 0): ").split(" "))
    # start = point(startx,starty)
    # endx,endy = map(int, input("输入终点(19 19): ").split(" "))
    # end = point(endx,endy)

    length, width = 20, 20
    start = point(10,0)
    end = point(10,19)
    G = np.zeros((length, width))
    for i in range(round(length*width/4)):  # 1/3的点添加障碍
        G[random.randint(0, length-1)][random.randint(0, width-1)] = 1

    # for i in range(5, 10):
    #     for j in range(5, 10):
    #         G[i][j] = 1
    #
    # for i in range(10, 15):
    #     for j in range(10, 15):
    #         G[i][j] = 1

    # G[1][2] = 1
    # G[2][1] = 1
    # G[6][3] = 1
    # G[6][4] = 1
    # G[6][5] = 1
    # G[6][6] = 1
    # G[6][7] = 1
    # G[6][8] = 1

    # for i in range(4, 15):
    #     for j in range(4, 15):
    #         G[i][j] = 1
    G[start.x][start.y] = 0
    G[end.x][end.y] = 0
    plt.imshow(G, cmap='gray_r', interpolation='none')
    # G[start.x][start.y] = 1
    plt.plot(start.y, start.x, "og")
    plt.plot(end.y, end.x, "xb")
    A_star(start, end)
    plt.title("A*算法在路径规划的应用")
    plt.legend()
    plt.show()


