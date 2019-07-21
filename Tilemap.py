import numpy as np
from PIL import Image


class Tilemap:
    def __init__(self, floorplan):
        self.init_tiles(floorplan)

    def init_tiles(self, file):
        im = Image.open('floor_plan.png')
        self.tiles = np.asarray(im)


    def gen_tile_image(self):
        img = Image.fromarray(self.tiles)
        return img

    def print_tiles(self):
        for i in self.tiles:
            for j in i:
                print(j, end="")
            print()

    def draw_horizontal_line(self, x_start, x_end, y_level):

        return False

    def draw_vertical_line(self, y_start, y_end, x_level):

        return False


tm = Tilemap('floor_plan.png')
tm.gen_tile_image()
