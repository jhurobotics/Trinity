class Region:
	''' 
	this is an expandable class that represents the contents of a region.
	'''
	def __init__(self, _is_solid):
		self._is_solid = _is_solid
	def is_solid(self):
		return self._is_solid
	def in_region(self, point):
		return False

