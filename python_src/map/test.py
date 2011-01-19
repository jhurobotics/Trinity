from .region.robot_regions import solid
from .map import *
from .basics.point import offset

def main():
    region_size = 10
    thickness = 0.1

    vwall = lambda : solid(thickness, region_size)
    hwall = lambda : solid(region_size, thickness)

    my_map = Map(region_size)
    my_map.add_region(vwall(), (0, 0))
    my_map.add_region(vwall(), (region_size - thickness, 0))
    my_map.add_region(hwall(), (0, 0))
    my_map.add_region(hwall(), (0, region_size - thickness))
    my_map.add_region(solid(1, region_size), (5, 0))

    print my_map.regions_containing((0.05, 9.95))
    print my_map.regions_containing((9.95, 9.95))
    def f(x_y):
            x, y = x_y
            return x + 0.1, y + 0.1
    def outofbounds(x_y):
            x, y = x_y
            return not (1 <= x <= region_size - 1 and 1 <= y <= region_size - 1)

    #print my_map.intercept_set((1, 1), (10, 10), 0.1)
#print region_list_to_set(my_map.region_step((1, 1), f, outofbounds))