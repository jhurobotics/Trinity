class Counter:
    def __init__(self):
        self._x = 0
    def __call__(self):
        self._x += 1
        return self._x - 1
