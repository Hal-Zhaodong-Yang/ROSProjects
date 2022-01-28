#!/usr/bin/env python
import numpy as np
f = open("global_waypoints.txt")

for x in f:
    x = np.array(x.split(","))
    print(x.astype(np.float))


f.close()
