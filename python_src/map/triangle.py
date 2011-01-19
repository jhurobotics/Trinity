import math

class triangle:
    def __init__(self, x1, y1, x2, y2):

        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
    def __str__(self):
        return "<(%f, %f),(%f, %f)>" % (self.x1, self.y1, self.x2, self.y2)

    def __repr__(self):
        return str(self)

    def interval(self, lengthofIntervals):
        self.lengthofIntervals = lengthofIntervals
        ydist=math.fabs(self.y1-self.y2)
        xdist=math.fabs(self.x1-self.x2)

        d=math.sqrt((ydist*ydist)+(xdist*xdist))
        theta=math.atan2(ydist,xdist)

        dy=(self.lengthofIntervals)*math.sin(theta)
        dx=(self.lengthofIntervals)*math.cos(theta)


        return (dx,dy)

def get_interval(p1, p2, interval):
    x1, x2 = p1
    y1, y2 = p2
    return triangle(x1, x2, y1, y2).interval(interval)

if __name__== '__main__':
    a=triangle(0,0,1,1)
    print a,'\n'
    print a.interval(1)

    print '1/sqrt(2)= ',1/math.sqrt(2)

