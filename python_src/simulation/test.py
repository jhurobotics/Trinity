from input_to_map import read_file, print_map
from map.region.robot_regions import *
from basics.utility import pairwise_add, pairwise_mul, pairwise_sub
from sensing.process_sense_data import read_sonar
from simulation.simulation_classes import SimulatedRobotIO
from map.map import *
from simulation.instantiate_map import *

m = read_file(open('input.txt'))
robot_region = instantiate_robot(m)
robot_io = SimulatedRobotIO(m)
print print_map(m)
walls = read_sonar(m, robot_region)
print walls
print robot_io.move(1, 'south')
print print_map(m)
