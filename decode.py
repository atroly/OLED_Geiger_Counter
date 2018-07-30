#!/usr/bin/python
#
# Convert the compressed Geiger counter readings into actual count values
#
# (C) Chris Taylor, 2018
#
file = open("rad", "r")
saved=0
for line in file:
    data = line.split()
    value = int(data[1])
    if not saved and value > 127:
        saved = value
    else:
        if saved:
            value += ((saved - 128) * 256)
            saved = 0
        print value

