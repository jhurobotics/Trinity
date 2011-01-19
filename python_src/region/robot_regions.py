from .region import Region
from .rectangle import rectangle

class room(rectangle):
    def __init__(self, x, y):
        rectangle.__init__(self, x, y)
        self.char = 'r'
class candle(rectangle):
    def __init__(self, x, y):
        rectangle.__init__(self, x, y)
        self.char = 'c'
class door(rectangle):
    def __init__(self, x, y):
        rectangle.__init__(self, x, y)
        self.char = 'd'
class start(rectangle):
    def __init__(self, x, y):
        rectangle.__init__(self, x, y)
        self.char = 'x'
class robot(rectangle):
    def __init__(self, x, y):
        rectangle.__init__(self, x, y)
        self.char = 'R'

class solid(rectangle):
    def __init__(self, x, y):
        Region.__init__(self, True)
        rectangle.__init__(self, x, y)
        self.char = '#'
    def is_solid(self):
        return True

