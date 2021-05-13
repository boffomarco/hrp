#!/usr/bin/env python3

import rosbag

import matplotlib.pyplot as plt
from matplotlib import colors

import seaborn as sns

import numpy as np

import math

folder = "/home/marco/Videos/"
filename = "2021-04-01-16-21-10"

folder = "/media/marco/writable/home/ubuntu/Bags/"
filename = "2021-04-21-12-42-20" # 120 sec
#filename = "2021-04-30-15-31-23" # 144 sec
#filename = "2021-04-30-15-38-12" # 144 sec
#filename = "2021-04-30-15-47-56" # 528 sec
#filename = "2021-04-30-16-01-31" # 156 sec
#filename = "2021-04-30-16-06-24" # 78 sec

path = folder + filename + ".bag"

bag = rosbag.Bag(path)


# Define list of measures to plot
xVel = list()
yawVel = list()
difft = list()
# Initialise Time difference values
tstamp = 0
# Iterate over topics of the bag
for topic, msg, t in bag.read_messages(topics=["/cmd_vel"]):
    xVel.append(msg.linear.x)
    yawVel.append(msg.angular.z)

    t = float(str(t))
    difft.append((t - tstamp)/1e9)
    tstamp = t

difft = difft[1:]
xVel = xVel[:-1]
yawVel = yawVel[:-1]

x = 0
y = 0
yaw = 0
for i in range(0, len(difft)):
    x += difft[i]*xVel[i]*math.cos(yaw)
    y += difft[i]*xVel[i]*math.sin(yaw)
    yaw += difft[i]*yawVel[i]

    print("x = " + str(round(x,3)) +  " \t y = " + str(round(y,3)) +  " \t yaw = " + str(round(yaw/math.pi*180,3)))


# Plot histogram of features
#fig, ax = plt.subplots()
#ax.plot(xVel, label = "Linear Velocity")
#ax.plot(yawVel, label = "Angular Velocity")
#plt.show()
#plt.pause(1)


# Close the bag
bag.close()
