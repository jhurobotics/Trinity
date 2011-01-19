def pairwise_map(f):
    def add(x, y):
        return map(lambda x: f(*x), zip(x, y))
    return add

pairwise_add = pairwise_map(lambda x, y: x + y)
pairwise_sub = pairwise_map(lambda x, y: x - y)
pairwise_mul = pairwise_map(lambda x, y: x * y)

directions = {
        'east': (1, 0),
        'west': (-1, 0),
        'north': (0, 1),
        'south': (0, -1),
        }
