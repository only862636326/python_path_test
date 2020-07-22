import numpy as np
import cv2
import matplotlib.pyplot as plt




if __name__ == '__main__':
    map = np.load("map.npy")
    print(type(map))
    map2 = cv2.resize(map, (650, 500))
    # cv2.imshow('map', map2)
    # cv2.waitKey(0)
    plt.imshow(map)
    plt.show()


