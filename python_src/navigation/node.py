class node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return "<%f, %f>" % (self.x, self.y)
    def __repr__(self):
        return str(self)

if __name__ == '__main__':
    a = node(4, 5)
    b = node(3, 2)
    h = {a: [a, b],
            b: [b]}
    print 'h is', h
    print 'h[a] is', h[a]
    h[b] = h[b] + [node(2, 2)]
    print 'h[b] is', h[b]