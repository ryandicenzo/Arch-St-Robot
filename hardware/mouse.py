import random
import struct
stop = False
xpos = 0
ypos = 0
file = open('/dev/input/mice', 'rb')
while stop != True:
    x, y = get_mouse_event()
    xpos += x
    ypos += y
    data = open('position.py', 'w')     #open, write data, then close file
    data.write(str(x)+' '+str(y))
    data.close()
    commands = open('commands.txt', 'r')
    command = commands.read()
    commands.close()
    if command == 'stop':
        stop = True
    elif command == 'reset':
        xpos = 0
        ypos = 0

def get_mouse_event():
    buf = file.read(3)
    x, y = struct.unpack('bb', buf[1:])
    return x, y
