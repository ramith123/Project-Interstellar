#!/usr/bin/env python
from std_msgs.msg import String


class Cube():

    def __init__(self, x=0, y=0, z=0):

        self.x_position = x
        self.y_position = y
        self.z_position = z

    def print_cube_position(self):
        print("I heard %f %f %f", self.x, self.y, self.z)
