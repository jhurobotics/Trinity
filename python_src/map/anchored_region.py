class AnchoredRegion:
	def __init__(self, region, base):
		self.region = region
		self.base = base
	def contains_point(self, p):
		return self.region.contains_point(p.offset(*self.base))
