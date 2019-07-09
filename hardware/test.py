import time

for i in range(100):
    data = open('position.txt','w')
    data.write(str(i))
    data.close()
    time.sleep(0.05)
