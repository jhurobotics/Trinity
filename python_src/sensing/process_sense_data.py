from map.map import Map
from simulation.input_to_map import read_file, print_map
from map.region.robot_regions import *
from basics.utility import pairwise_add, pairwise_mul, pairwise_sub

def strip_sonar_information(base, walls):
    base = tuple(map(int, base))
    walls_new = {}
    for key, value in walls.items():
        value = value[0]
        if key in ['north', 'south']:
            walls_new[key] = pairwise_sub(base, value)[1]
        elif key in ['east', 'west']:
            walls_new[key] = -pairwise_sub(base, value)[0]
    return walls_new

def read_sonar(m, r, base = None):
    if base is None:
        for region in m:
            base = region.base
            if isinstance(region, robot):
                break  #this means that base is now location of the subregion that is a robot
    return strip_sonar_information(base, read_sonar_raw(m, r, base))
def read_sonar_raw(m, r, base = None):
    if not (isinstance(m, Map) and isinstance(r, robot)):
        raise Exception("Not a map or not a robot")
    if base is None:
        for base, region in m.subregions:
            if isinstance(region, robot):
                break  #this means that base is now location of the subregion that is a robot
    walls = {}
    walls['east'] = isolate_wall(m.step_until(base, (0.5, 0), stop_at_wall))
    walls['south'] = isolate_wall(m.step_until(base, (0, 0.5), stop_at_wall))
    walls['west'] = isolate_wall(m.step_until(base, (-0.5, 0), stop_at_wall))
    walls['north'] = isolate_wall(m.step_until(base, (0, -0.5), stop_at_wall))
    return walls
    
def stop_at_wall(base, regions):
    if regions:
        for _, subregion in regions[-1]:
            if subregion.is_solid():
                return True
    return False
           
def isolate_wall(regions):
    if regions:
        for base, region in regions[-1]:
            if isinstance(region, solid):
                return base, region
            else:
                print 'not a wall, but region =', region
                return base, region
    else:
        return []

