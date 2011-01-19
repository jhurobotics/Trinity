from .region import Region
class rectangle(Region):
	def __init__(self, x, y):
		Region.__init__(self, False)
		self.x = x
		self.y = y
                self.char = ' '
	def in_region(self, point):
		x, y = point
		return 0 <= x <= self.x and 0 <= y <= self.y
	def __str__(self):
		return "<rectangle[%s] (%f, %f)>" % (self.char, self.x, self.y)
	__repr__ = __str__
