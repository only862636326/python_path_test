# -*- encode=utf-8 -*-

class Circle(object):

    def __init__(self, r=0):
        self.__radius = r

    def SetRadius(self, r):
        self.__radius = r

    def Area(self):
        return 3.1415926*self.__radius*self.__radius

    def __str__(self):
        return str(self.Area())

    def __ge__(self, other):
        if isinstance(other, Circle):
            area = other.Area()
        elif isinstance(other, (int, float)):
            area = other
        else:
            return False
        if self.Area() >= area:
            return True
        else:
            return False


if __name__ == '__main__':
    a, b = Circle(1), Circle(2)

    print(a, b)
    print((a>= 'asdf'))
