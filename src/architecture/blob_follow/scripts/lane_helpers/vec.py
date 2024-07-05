import math

class Vec:
    """
    Helper class to make some of the math cleaner
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f'({self.x},{self.y})'
    def __repr__(self):
        return f'({self.x},{self.y})'

    def smul(self,s):
        """ Scalar multiplication """
        return Vec(self.x*s,self.y*s)
        
    def add(self,other):
        """ Vector addition """
        return Vec(self.x+other.x, self.y+other.y)

    def sdiv(self,s):
        """ scalar division """
        return Vec(self.x/s,self.y/s)

    @staticmethod
    def dist(p0, p1) -> float:
        """ Distance between two points """
        return math.sqrt((p0.x-p1.x)**2 + (p0.y-p1.y)**2)