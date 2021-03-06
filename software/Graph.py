import numpy as np
from Node import Node
from PIL import Image

class Tilemap:

    walk_tile = np.array([255,255,255,255])
    border_tile = np.array([0,0,0,255])

    def __init__(self, floorplan):
        self.init_tiles(floorplan)

    def init_tiles(self, file):
        im = Image.open('./floor_plan.png')
        self.tiles = np.asarray(im)

        w, h, d = self.tiles.shape
        self.tiles = [[0 if (self.tiles[x][y]==self.walk_tile).all() else 1 for x in range(w)] for y in range(h)]


    def gen_tile_image(self):
        img = Image.fromarray(self.tiles)
        return img

    def print_tiles(self):
        for i in self.tiles:
            for j in i:
                print(j, end="")
            print()


    def get_tiles(self):
        return self.tiles

    def draw_horizontal_line(self, x_start, x_end, y_level):

        return False

    def draw_vertical_line(self, y_start, y_end, x_level):

        return False

#
# Referenced from https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
#
class AStar:
    def __init__(self, maze):
        self.maze = maze


    def gen_path(self, start, end):
        start_node = Node(None, start)
        end_node = Node(None, end)

        open = []
        closed = []

        open.append(start_node)

        while len(open) > 0:
            curr = open[0]
            curr_index = 0

            for index, item in enumerate(open):
                if item.f < curr.f:
                    curr = item
                    curr_index = index

            open.pop(curr_index)
            closed.append(curr)

            if curr == end_node:
                path = []
                tracer = curr
                while tracer is not None:
                    path.append(tracer.position)
                    tracer = tracer.parent
                return path[::-1]

            children = []
            for new_pos in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                node_pos = (curr.position[0] + new_pos[0], curr.position[1] + new_pos[1])

                if node_pos[0] > (len(self.maze) - 1) or node_pos[0] < 0 or node_pos[1] > (len(self.maze[len(self.maze)-1])-1) or node_pos[1] < 0:
                    continue

                print(node_pos[1])
                if self.maze[node_pos[0]][node_pos[1]] != 0:
                    continue

                new_node = Node(curr, node_pos)
                children.append(new_node)

            for child in children:
                for closed_child in closed:
                    if child == closed_child:
                        continue

                child.g = curr.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                for open_node in open:
                    if child == open_node and child.g > open_node.g:
                        continue

                open.append(child)

    def draw_path(self, path_to_img, path_to_draw):
        img = Image.open(path_to_img)
        pixels = img.load()

        for x,y in path_to_draw:
            pixels[x,y] = (255, 0, 0, 255)

        img.show()


