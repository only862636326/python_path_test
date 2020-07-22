import cv2
import matplotlib.pyplot as plt
import numpy as np

def creat_base():
    map = np.ndarray((10000, 13300), dtype=np.uint8)
    # map[200:300, 200:3000] = 255
    # cv2.imshow("sdfa", map)
    print(map.shape)
    map.fill(0)
    #boundery
    map[0, :] = 255
    map[-1, :] = 255
    map[:, 0] = 255
    map[:, -1] = 255
    
    # 护栏
    map[0:1590-1, 1575+2500-1] = 255
    map[0:1590-1, 1575+2500 + (2075 + 500) * 2 - 1] = 255

    map[10000-3090-1:, 1575+2500-1] = 255
    map[10000-3090-1:, 1575+2500 + (2075 + 500) * 2 - 1] = 255

    #守门员
    map[2233+1965-1,   1570+2500-1] = 255
    map[2233+1965-1,   1575+2500 + (2075 + 500) * 2 - 1] = 255

    map[2233+1965-1,   1570+2500-1] = 255
    map[2233+1965-1,   1575+2500 + (2075 + 500) * 2 - 1] = 255

    map[1590+1330-1,   1575+985-1] = 255
    map[1590+1330-1,   13300-(1575+985)-1] = 255
    map[1590+1330*3-1, 1575+985-1] = 255
    map[1590+1330*3-1, 13300-(1575+985)-1] = 255

    #中线
    map[:, 13300//2] = 255

    #腐蚀
    kernel = np.ones((70,70), np.uint8)
    # erosion = cv2.erode(map, kernel, iterations=1)
    #膨胀
    dilation = cv2.dilate(map, kernel, iterations=1)

    np.save("map", dilation)

    map1 = cv2.resize(dilation, (650, 500))
    cv2.imshow("base", map1)
    cv2.waitKey(0)

def creat_bgr():
    map = np.load("map.npy")
    h = np.ndarray((10000, 13300), dtype=np.uint8)
    s = np.ndarray((10000, 13300), dtype=np.uint8)


    # 起点
    h[0:1000-1,0:1000-1] = 255
    h[0:1000-1,-1000:] = 255
    h[-1000:,1570+2500-1:1570+2500-1+1000] = 255
    h[-1000:,1575+2500 + (2075 + 500) * 2 - 1-1000:1575+2500 + (2075 + 500) * 2 - 1] = 255

    # 放球点
    h[1300*1-1:580+1300*1-1, 1575+2500+2075-1:1000+1575+2500+2075-1] = 255
    h[1300*2-1:580+1300*2-1, 1575+2500+2075-1:1000+1575+2500+2075-1] = 255
    h[1300*3-1:580+1300*3-1, 1575+2500+2075-1:1000+1575+2500+2075-1] = 255
    h[1300*4-1:580+1300*4-1, 1575+2500+2075-1:1000+1575+2500+2075-1] = 255
    h[1300*5-1:580+1300*5-1, 1575+2500+2075-1:1000+1575+2500+2075-1] = 255

    # 分界线
    h[:, 1575-1-20:1575-1+20,] = 255
    h[:, -(1575+20):-(1575-20),] = 255

    alpha = np.zeros((10000, 13300), dtype=np.uint8)
    alpha.fill(255)
    hsv = cv2.merge((h, s, map,alpha))
    # hsv[]


    map2 = cv2.resize(hsv, (650, 500))

    # cv2.imshow("map2", map2)
    # cv2.waitKey()
    plt.imshow(map2)
    plt.show()



if __name__ == '__main__':
    # creat_base()
    creat_bgr()



















