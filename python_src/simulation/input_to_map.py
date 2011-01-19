from sys import argv
from map.map import Map
from map.region.robot_regions import *

symbol_meanings = {
        ' ': None,
        '#': solid,
        'r': room,
        'c': candle,
        'd': door,
        'x': start
        }

def read_file(f):
    columns = int(f.readline())
    rows = int(f.readline())
    
    m = Map(max(rows, columns))
    for i in range(rows - 1, 0, -1):
        line = pad_to_length(f.readline(), columns)
        for j, region in parse_line(line):
            m.add_region(region, (j, rows - i))
    return m
    

def parse_line(line):
    regions = []
    for i in range(len(line)):
        meaning = symbol_meanings.get(line[i], None)
        if meaning is not None:
            region = meaning(1, 1)
            region.show_region = True
            regions.append((i, region))
    return regions

def print_map(m):
    drawing = [[' ' for x in range(m.region_size)] for x in range(m.region_size)]
    errata = []
    for region in m:
        base = region.base
        if hasattr(region, 'show_region') and region.show_region:
            drawing[int(base[1])][int(base[0])] = region.char
        if hasattr(region, 'errata') and region.errata:
            errata.append("<%f, %f> %s" % (base[0], base[1], str(region)))
    map_text = list(map(lambda x: ''.join(x), map(lambda y: map(lambda x: '%s ' %
        x, y), drawing)))
    return '\n'.join(map_text + errata)



def pad_to_length(line, length):
    return line + (' ' * (length - len(line)))

if __name__ == '__main__':
    m = read_file(open('input.txt'))
    for _, region in m.subregions:
        if isinstance(region, start):
            region.errata = True
            region.x = 0.5
            region.y = 0.5
    print print_map(m)
