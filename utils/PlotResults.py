#!/usr/bin/env python3

# [TODO]
# - Map of different configurations
# - Animation of different configurations

import rosbag

import rospy

import numpy as np

import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib as mpl

import seaborn as sns


#filename = '/home/marco/Videos/OutTests/KistaGardPark_2021-03-18-16-32-32__th0.3-lat59.406827-long17.940565.bag' # KistaGardPark_out0-157m
#filename = '/home/marco/Videos/OutTests/KistaGardPark_2021-03-18-16-39-14__th-0.22-lat59.406820-long17.940523.bag' # KistaGardPark_out1-390m
#filename = '/home/marco/Videos/OutTests/KistaParkLot_2021-03-19-17-15-38__th-2.76-lat59.403067-long17.958173.bag' # KistaParkLot_out1-37m
#filename = '/home/marco/Videos/OutTests/KistaParkLot_2021-03-19-17-04-41__th-2.76-lat59.403070-long17.958150.bag' # KistaParkLot_out0-77m
#filename = '/home/marco/Videos/2021-04-01-15-31-27.bag'


# Plot Odometry messages
def plotOdometry(bag, topic):
    #Odometry= list()
    print("Get " + str(topic))
    hX = np.zeros((2, 1))
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]).reshape(2,1)
        hX = np.hstack((hX, pose))
        #Odometry.append((msg.pose.pose.position.x,msg.pose.pose.position.y))
    return hX

# Plot the topic of a bag with a colour
def plotBag(filename, topic):
    print("Open " + str(filename))
    bag = rosbag.Bag(filename)

    topics = bag.get_type_and_topic_info().topics

    if topic in topics:
        history = plotOdometry(bag, topic)
    else:
        print(str(topic) + " missing in " + str(topics) + " in bag " + str(filename))

    bag.close()

    return history


if __name__ == "__main__":


    ### DEFINE PLOTTING VARIABLES
    plt.rc('text', usetex=True)         # Use LaTexmat
    plt.rc('font', family='serif')
    plt.rc('font', serif='Times New Roman')
    #sns.set(rc={'text.usetex': True})  # Add background

    print("Start Plotter")

    filename = '/home/marco/Pictures/AEKF5-NoNMEA-BWB/2021-06-15-00-03-24-NoNMEA-BWB.bag'

    GT = plotBag('/home/marco/Videos/GT.bag', '/Odom_Ground')
    #plotBag('/home/marco/Pictures/GT.bag')


    GPS = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_GPS')
    GPS_LEFT = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_gps_left')
    GPS_RIGHT = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_gps_right')

    CONTROL = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_GPS')
    WHEEL = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_gps_left')
    VISUAL = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_gps_right')

    AEKF1 = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_AEKF')
    #AEKF2 = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_AEKF')
    #AEKF3 = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_AEKF')
    #AEKF4 = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_AEKF')
    #AEKF5 = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_AEKF')
    #AEKF6 = plotBag('/home/marco/Videos/FILENAME.bag', '/Odom_AEKF')

    plt.figure(20)
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(GPS_LEFT[0, :],
             GPS_LEFT[1, :], color='red', marker='3', linestyle="None",
             label = "GPS Left")
    plt.plot(GPS_RIGHT[0, :],
             GPS_RIGHT[1, :], color='orange', marker='4', linestyle="None",
             label = "GPS Right")
    plt.plot(GPS[0, :],
             GPS[1, :], color='magenta', marker='1', linestyle="None",
             label = "GPS")
    plt.plot(CONTROL[0, :].flatten(),
             CONTROL[1, :].flatten(), color='deepskyblue', linestyle='dashed', linewidth = 2,
             label = "Control")
    plt.plot(WHEEL[0, :].flatten(),
             WHEEL[1, :].flatten(), color='teal', linestyle='dotted', linewidth = 2,
             label = "Wheel")
    plt.plot(VISUAL[0, :].flatten(),
             VISUAL[1, :].flatten(), color='lawngreen', linestyle='dotted', linewidth = 2,
             label = "Visual")
    plt.plot(GT[0, :].flatten(),
             GT[1, :].flatten(), color='black', linestyle='solid', linewidth = 2,
             label = "Ground")
    plt.plot(AEKF1[0, :].flatten(),
             AEKF1[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
             label = "Model1")
    #plt.plot(AEKF2[0, :].flatten(),
    #         AEKF2[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
    #         label = "Model2")
    #plt.plot(AEKF3[0, :].flatten(),
    #         AEKF3[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
    #         label = "Model3")
    #plt.plot(AEKF4[0, :].flatten(),
    #         AEKF4[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
    #         label = "Model4")
    #plt.plot(AEKF5[0, :].flatten(),
    #         AEKF5[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
    #         label = "Model5")
    #plt.plot(AEKF6[0, :].flatten(),
    #         AEKF6[1, :].flatten(), color='blue', linestyle='dashdot', linewidth = 2,
    #         label = "Model6")
    plt.legend()

    plt.show()

    # Wait for the figure
    plt.pause(1)

    # Wait for the user input to terminate the program
    input("Press any key to terminate the program\n")
    # Print bye message
    print("THE END")

