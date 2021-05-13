#!/usr/bin/env python3

import rosbag

import matplotlib.pyplot as plt
from matplotlib import colors

import seaborn as sns

import numpy as np

import pymap3d as pm


def hist_GPS(bag, topic, seconds):

    lat = list()
    lon = list()
    time_i = 0 # Initial time
    time_f = 0 # Final time
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if( time_i == 0):
            time_i = t
        if(int(str(time_f)) - int(str(time_i)) < int(seconds*10e8)):  # Initial seconds of stillness
            #print(int(str(time_f)) - int(str(time_i)))
            lat.append(msg.latitude)
            lon.append(msg.longitude)
        time_f = t
    #print(time_f - time_i) # Time difference in seconds*10^8
    #print(time_i)
    #print(time_f)

    ### LATITUDE
    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(lat, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(lat, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_lat = np.mean(lat)
    sigma_lat = np.std(lat)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_lat)) *
        np.exp(-0.5 * (1 / sigma_lat * (bins - mu_lat))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(lat, np.full_like(lat, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of Latitude: $\mu='+ str(mu_lat)+r'$, $\sigma='+ str(sigma_lat)+r'$') #Assign title
    ax.set_xlabel(r'Latitude: min=' + str(np.min(lat)) + r' MAX=' + str(np.max(lat)) ) #Assign x label
    ax.set_ylabel('Number of measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    #plt.show()

    ### Longitude
    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(lon, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(lon, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_lon = np.mean(lon)
    sigma_lon = np.std(lon)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_lon)) *
        np.exp(-0.5 * (1 / sigma_lon * (bins - mu_lon))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(lon, np.full_like(lon, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of Longitude: $\mu='+ str(mu_lon)+r'$, $\sigma='+ str(sigma_lon)+r'$') #Assign title
    ax.set_xlabel(r'Longitude: min=' + str(np.min(lon)) + r' MAX=' + str(np.max(lon)) ) #Assign x label
    ax.set_ylabel('Number of measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    #plt.show()

    figur, ax = plt.subplots(tight_layout=True)
    hist = ax.hist2d(lat, lon, bins=100)#, norm=colors.LogNorm())


    lat = list()
    lon = list()
    time_i = 0 # Initial time
    time_f = 0 # Final time
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if( time_i == 0):
            time_i = t
        if(float(str(time_f)) - float(str(time_i)) < float(seconds*1e9)):  # Initial seconds of stillness
            #print(int(str(time_f)) - int(str(time_i)))
            lon_, lat_ , alt_ = pm.geodetic2enu(msg.latitude, msg.longitude, 0, mu_lat, mu_lon, 0)

            lat.append(lat_)
            lon.append(lon_)
        time_f = t
    #print(time_f - time_i) # Time difference in seconds*10^8
    #print(time_i)
    #print(time_f)

    ### LATITUDE
    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(lat, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(lat, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_lat = np.mean(lat)
    sigma_lat = np.std(lat)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_lat)) *
        np.exp(-0.5 * (1 / sigma_lat * (bins - mu_lat))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(lat, np.full_like(lat, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of Latitude: $\mu='+ str(mu_lat)+r'$, $\sigma='+ str(sigma_lat)+r'$') #Assign title
    ax.set_xlabel(r'Latitude: min=' + str(np.min(lat)) + r' MAX=' + str(np.max(lat)) ) #Assign x label
    ax.set_ylabel('Number of measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    #plt.show()

    ### Longitude
    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(lon, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(lon, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_lon = np.mean(lon)
    sigma_lon = np.std(lon)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_lon)) *
        np.exp(-0.5 * (1 / sigma_lon * (bins - mu_lon))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(lon, np.full_like(lon, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of Longitude: $\mu='+ str(mu_lon)+r'$, $\sigma='+ str(sigma_lon)+r'$') #Assign title
    ax.set_xlabel(r'Longitude: min=' + str(np.min(lon)) + r' MAX=' + str(np.max(lon)) ) #Assign x label
    ax.set_ylabel('Number of measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    #plt.show()

    figur, ax = plt.subplots(tight_layout=True)
    hist = ax.hist2d(lat, lon, bins=100)#, norm=colors.LogNorm())


    plt.show()
    plt.pause(1)


def hist_IMU(bag, topic, seconds):

    # Define list of measures to plot
    yAcc = list()
    yawVel = list()
    difftime = list()
    difft = list()
    # Initialise Time difference values
    timestamp = 0
    tstamp = 0
    time_i = 0 # Initial time
    time_f = 0 # Final time
    # Iterate over topics of the bag
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if( time_i == 0):
            time_i = t
        if(float(str(time_f)) - float(str(time_i)) < float(seconds*1e9)):  # Initial seconds of stillness

            yAcc.append(msg.linear_acceleration.y)
            yawVel.append(msg.angular_velocity.z)

            t = float(str(t))
            difft.append((t - tstamp)/1e9)
            tstamp = t

            stamp = float(str(msg.header.stamp))
            difftime.append((stamp - timestamp)/1e9)
            timestamp = stamp
        time_f = t

    difft = difft[1:]
    difftime = difftime[1:]

    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(yAcc, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(yAcc, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_yAcc = np.mean(yAcc)
    sigma_yAcc = np.std(yAcc)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_yAcc)) *
        np.exp(-0.5 * (1 / sigma_yAcc * (bins - mu_yAcc))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(yAcc, np.full_like(yAcc, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of Linear Acc: $\mu='+ str(mu_yAcc)+r'$, $\sigma='+ str(sigma_yAcc)+r'$') #Assign title
    ax.set_xlabel(r'Linear Acc: min=' + str(np.min(yAcc)) + r' MAX=' + str(np.max(yAcc)) ) #Assign x label
    ax.set_ylabel('Measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    #plt.show()

    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(yawVel, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(yawVel, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_yawVel = np.mean(yawVel)
    sigma_yawVel = np.std(yawVel)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_yawVel)) *
        np.exp(-0.5 * (1 / sigma_yawVel * (bins - mu_yawVel))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(yawVel, np.full_like(yawVel, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of Angular Vel: $\mu='+ str(mu_yawVel)+r'$, $\sigma='+ str(sigma_yawVel)+r'$') #Assign title
    ax.set_xlabel(r'Angular Vel: min=' + str(np.min(yawVel)) + r' MAX=' + str(np.max(yawVel)) ) #Assign x label
    ax.set_ylabel('Measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()
    #plt.show()


    plt.show()



    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(difft, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(difft, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_difft = np.mean(difft)
    sigma_difft = np.std(difft)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_yawVel)) *
        np.exp(-0.5 * (1 / sigma_yawVel * (bins - mu_yawVel))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(difft, np.full_like(difft, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of ROS time diff: $\mu='+ str(mu_difft)+r'$, $\sigma='+ str(sigma_difft)+r'$') #Assign title
    ax.set_xlabel(r'ROS time diff: min=' + str(np.min(difft)) + r' MAX=' + str(np.max(difft)) ) #Assign x label
    ax.set_ylabel('Measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()

    # Plot histogram of features
    fig, ax = plt.subplots()
    # the histogram of the data
    n, bins, patches = ax.hist(difftime, density=True, stacked=True)
    # Gaussian Kernel
    sns.distplot(difftime, hist=False, kde=True,
             color = 'blue',
             hist_kws={'edgecolor':'black'},
             label = "Gaussian Kernel",
             kde_kws={'linewidth': 1})
    # add a 'best fit' line
    mu_difftime = np.mean(difftime)
    sigma_difftime = np.std(difftime)
    y = ((1 / (np.sqrt(2 * np.pi) * sigma_yawVel)) *
        np.exp(-0.5 * (1 / sigma_yawVel * (bins - mu_yawVel))**2))
    ax.plot(bins, y, '--', label = "Gaussian Distribution")
    # Plot real values
    ax.plot(difftime, np.full_like(difftime, -0.01), '|k', markeredgewidth=1)
    # Labels
    ax.set_title(r'Histogram of real time diff: $\mu='+ str(mu_difftime)+r'$, $\sigma='+ str(sigma_difftime)+r'$') #Assign title
    ax.set_xlabel(r'Real time diff: min=' + str(np.min(difftime)) + r' MAX=' + str(np.max(difftime)) ) #Assign x label
    ax.set_ylabel('Measures') #Assign y label
    plt.legend()
    # Tweak spacing to prevent clipping of ylabel
    fig.tight_layout()

    plt.show()

    plt.pause(1)



folder = "/home/marco/Videos/"
filename = "2021-04-01-16-21-10"


folder = "/media/marco/writable/home/ubuntu/Bags/"
filename = "2021-04-21-12-42-20" # 120 sec
filename = "2021-04-30-15-31-23" # 144 sec
filename = "2021-04-30-15-38-12" # 144 sec
filename = "2021-04-30-15-47-56" # 528 sec
#filename = "2021-04-30-16-01-31" # 156 sec
filename = "2021-04-30-16-06-24" # 78 sec

seconds = 78

path = folder + filename + ".bag"

bag = rosbag.Bag(path)


#hist_GPS(bag, '/GPSfix', seconds)
#hist_GPS(bag, '/gps_left/fix', seconds)
#hist_GPS(bag, '/gps_right/fix', seconds)

hist_IMU(bag, '/imu_left/imu/data_raw', seconds)
hist_IMU(bag, '/imu_right/imu/data_raw', seconds)


# Close the bag
bag.close()
