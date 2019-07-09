import subprocess
import time
data = open('position.txt', 'r')
my_process = subprocess.Popen(['python3', 'test.py'])
#my_process = subprocess.call(['python3', 'test.py'])
time.sleep(1)
print(data.read())
