#!/usr/bin/env python3

import rosbag

import matplotlib.pyplot as plt
from matplotlib import colors


gps = False
imu = True
imu_left = True
imu_right = True


folder = "/home/marco/Videos/"
filename = "2021-04-01-16-21-10"
path = folder + filename + ".bag"

bag = rosbag.Bag(path)

if(gps):

    fig, axs = plt.subplots(1, 2, sharey=True, tight_layout=True)

    lat = list()
    lon = list()
    for topic, msg, t in bag.read_messages(topics=['/GPSfix']):
        lat.append(msg.latitude)
        lon.append(msg.longitude)

    bag.close()

    axs[0].hist(lat)


    axs[1].hist(lon)

    plt.show()

    figur, ax = plt.subplots(tight_layout=True)
    hist = ax.hist2d(lat, lon, bins=40)#, norm=colors.LogNorm())
    plt.show()


if(imu_left):

    fig, axs = plt.subplots(1, 2, sharey=True, tight_layout=True)

    yAcc = list()
    yawVel = list()
    difftime = list()
    difft = list()
    timestamp = 0
    tstamp = 0
    for topic, msg, t in bag.read_messages(topics=['/imu_left/imu/data/enu']):
        yAcc.append(msg.linear_acceleration.y)
        yawVel.append(msg.angular_velocity.z)

        t = float(str(t))
        difft.append(t - tstamp)
        tstamp = t

        stamp = float(str(msg.header.stamp))
        difftime.append(stamp - timestamp)
        timestamp = stamp


    bag.close()

    axs[0].hist(yAcc)
    axs[1].hist(yawVel)
    plt.show()

    fig, axs = plt.subplots(1, 2, sharey=True, tight_layout=True)

    axs[0].hist(difft[1:50])
    axs[1].hist(difftime[1:50])
    plt.show()

    for i in range(1,10):
        print("t : " + str(difft[i]) + "  - time: " + str(difftime[i]))

