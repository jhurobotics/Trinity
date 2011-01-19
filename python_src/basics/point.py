class point:
	def __init__(self, x, y):
		self.x = x
		self.y = y
	def offset(self, x, y):
		return point(self.x - x, self.y - y)
	def __str__(self):
		return "<%f, %f>" % (self.x, self.y)
	__repr__ = __str__
	def __iter__(self):
		return iter((self.x, self.y))

def offset(p1, p2):
	return tuple(point(*p1).offset(*p2))
