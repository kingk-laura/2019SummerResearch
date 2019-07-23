import numpy
import matplotlib.pyplot as plt


f = open('./rolldata.txt')
xl = []
yl = []

for line in f:
    line = line[:-1]
    ld = line.split(',')
    xl.append(float(ld[0]))
    yl.append(float(ld[4]))


plt.plot(xl,yl)
plt.ylabel('angle')
plt.xlabel('time')
plt.show()

