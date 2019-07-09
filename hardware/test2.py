"""
import time
time_prev = time.time()
print(time.time() - time_prev)
"""

"""
a = 1
b = 2
file = open('position.txt', 'w')
file.write(str(a)+' '+str(b))
file.close()

file = open('position.txt', 'r')
print(file.read().split(' ')[1])
"""

import subprocess
subprocess.call('ls', shell = True)
