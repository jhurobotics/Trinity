def instantiate_robot(m):
    def is_start(region):
        return isinstance(region, start)
    for region in m.get_regions_with_predicate(is_start):
        base = region.base
        robot_region = robot(0.5, 0.5)
        robot_region.show_region = True
        robot_region.errata = True
        robot_position = pairwise_add(base, (0.25, 0.25))
        m.add_region(robot_region, robot_position)
        return robot_region
    #for base, region in m.subregions:
        #if isinstance(region, start):
