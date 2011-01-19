class abstract_robot:
    def __init__(self, robot_io):
        self.robot_io = robot_io
    def act_in_map(self, robot_map):
        while STUFF:
            results = self.robot_io.move(*JUNK)
            if MAYBE:
                break
