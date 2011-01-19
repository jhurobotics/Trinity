from .anchored_region import AnchoredRegion
from .triangle import get_interval
from .counter import Counter
from .basics.point import offset
from .basics.utility import pairwise_add, pairwise_map

get_next_id = Counter()

class Map:
    '''
    This contains a list of regions of various types. 
    '''
    def __init__(self, region_size = 10):
        '''
        The region is a square of side length region_size. 
        '''
        self.region_size = region_size
        self._regions = []
    def add_region(self, region, base):
        region.id = get_next_id()
        region.base = base
        self._regions.append(region)
    def regions_containing(self, point):
        regions = []
        for region in self._regions:
            base = region.base
            if region.in_region(offset(point, base)):
                regions.append((tuple(base), region))
        return regions
    def region_step(self, point, incrementor, finished_predicate):
        regions = []
        while not finished_predicate(point, regions):
            regions.append(self.regions_containing(point))
            point = incrementor(point)
        return regions
    def step_until(self, point, increment, finished_predicate):
        regions = []
        while not finished_predicate(point, regions):
            regions.append(self.regions_containing(point))
            point = pairwise_add(point, increment)
        return regions
    def regions_between(self, point, end_point, interval, finishedp = None):
        if finishedp is None:
            finishedp = lambda x, y: False
        def incrementor(point):
            return pairwise_add(point, get_interval(point, end_point, interval))
        def my_finishedp(point, regions):
            if finishedp(point, regions):
                return True
            the_interval = get_interval(point, end_point, interval)
            difference = pairwise_map(lambda x, y: x - y)(point, end_point)
            if (difference[0] * the_interval[0] > 0):
                print 'same sign', point, the_interval, difference
                return True
            return False #TODO
        return self.region_step(point, incrementor, my_finishedp)
    def get_regions_with_predicate(self, predicate):
        regions = []
        for region in self._regions:
            if predicate(region):
                regions.append(region)
        return regions
    def __iter__(self):
        return iter(self._regions)


def region_list_to_set(regions_list):
    s = set()
    for regions in regions_list:
        print 'regions are', regions
        s = s.union(map(tuple, regions))
    return s

