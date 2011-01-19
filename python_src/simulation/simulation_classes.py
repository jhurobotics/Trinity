from simulation.instantiate_map import *
from basics.utility import directions
from map.region.robot_regions import robot
from sensing.process_sense_data import read_sonar
from map.map import Map, region_list_to_set

class RobotException(Exception):
    pass

class WallException(RobotException):
    pass

class RobotIO:
    def __init__(self):
        pass
    def move(self, displacement, direction):
        ''' 
        This raises as a WallException if the robot would move into a wall. 
        It returns a tuple with the new position of the robot and a list of sense
        outputs for each step. 
        
        If a WallException is raised, the robot will be in the square adjacent to
        the wall. 
        '''
        if direction not in ['north', 'east', 'west', 'south']:
            raise RobotException()
        # might raise WallException
        new_position = None
        return (new_position, []) # this should be the sensings of each square. 
    def sense(self):
        return {} # current position sense info. 
    def on_door(self):
        return False
    def extinguish_candle(self):
        return False
    def detect_candle(self):
        return False

class SimulatedRobotIO(RobotIO):
    def __init__(self, my_map):
        def is_robot(region):
            return isinstance(region, robot)
        self.my_map = my_map
        for region in my_map.get_regions_with_predicate(is_robot):
            base = region.base
            self.robot_location = base
            self.robot = region
        if not hasattr(self, 'robot'):
            raise Exception("Couldn't find a robot")
        self.last_sense = read_sonar(self.my_map, self.robot, self.robot_location)
    def move(self, displacement, direction):
        senses = []
        move_direction = directions[direction]
        print 'move_direction =', move_direction
        for i in range(displacement):
            new_location = pairwise_add(self.robot_location, move_direction)
            regions = region_list_to_set(self.my_map.regions_between( \
                    self.robot_location, new_location, 0.5))
            for base, region in regions:
                if isinstance(region, solid):
                    raise WallException
                self.robot_location = new_location
                self.robot.base = self.robot_location
            senses.append(read_sonar(self.my_map, self.robot, self.robot_location))
        self.last_sense = senses[-1]
        return senses
    def sense(self):
        return self.last_sense
