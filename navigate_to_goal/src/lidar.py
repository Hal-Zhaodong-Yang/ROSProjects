import math


boxRange = []
r = 0.12
for i in range(20):
    boxRange.append(r / math.cos(i * 3 * math.pi / 180))
for i in range(20, 40):
    boxRange.append(r * 2)
for i in range(40, 61):
    boxRange.append(r / math.cos((180 - 3 * i) * math.pi / 180))

for i in range(20):
    print(i, boxRange[i], 60 - i, boxRange[60 - i])