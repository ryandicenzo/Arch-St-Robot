import random
stop = False
xpos = 0
ypos = 0

r = random.random()

while stop != True:
    fake_data = r.randint()        #generate fake data

    data = open('position.py', 'w')     #open, write data, then close file
    data.write(str(fake_data))
    data.close()
    commands = open('commands.txt', 'r')
    command = commands.read()
    if command == 'stop':
        stop = True
    elif command == 'reset':
        xpos = 0
        ypos = 0
