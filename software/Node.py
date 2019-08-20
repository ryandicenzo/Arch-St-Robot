class Node:
    x = 0
    y = 0
    
    def __init__(self, parent=None, position=None, g=0, h=0):

        self.parent = parent
        self.position = position
        self.g = g
        self.h = h

        self.f = g + h

    def __eq__(self, other):
        return self.position == other.position