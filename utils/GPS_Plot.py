#!/usr/bin/env python3

import rosbag

import rospy

import gmplot


#filename = '/home/marco/Videos/OutTests/KistaGardPark_2021-03-18-16-32-32__th0.3-lat59.406827-long17.940565.bag' # KistaGardPark_out0-157m
#filename = '/home/marco/Videos/OutTests/KistaGardPark_2021-03-18-16-39-14__th-0.22-lat59.406820-long17.940523.bag' # KistaGardPark_out1-390m
#filename = '/home/marco/Videos/OutTests/KistaParkLot_2021-03-19-17-15-38__th-2.76-lat59.403067-long17.958173.bag' # KistaParkLot_out1-37m
#filename = '/home/marco/Videos/OutTests/KistaParkLot_2021-03-19-17-04-41__th-2.76-lat59.403070-long17.958150.bag' # KistaParkLot_out0-77m
#filename = '/home/marco/Videos/2021-04-01-15-31-27.bag'

#if('/GPSfix' in topics):
#    plotNatSav(gmap, bag, '/GPSfix', 'red')
#if('/gnss_left/fix' in topics):
#    plotNatSav(gmap, bag, '/gnss_left/fix', 'green')
#if('/gnss_right/fix' in topics):
#    plotNatSav(gmap, bag, '/gnss_right/fix', 'blue')

#info = bag.get_type_and_topic_info()
#topics = []
#for topic in info.topics:
#    topics.append(str(topic))
#print(topics)


# Plot NatSavFix messages
def plotNatSav(gmap, bag, topic, colour):
    NatSavfix = list()
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        NatSavfix.append((msg.latitude,msg.longitude))

    gmap.plot(*zip(*NatSavfix), colour, edge_width=5)

# Plot Odometry messages
def plotOdometry(gmap, bag, topic, colour):
    Odometry= list()
    print(topic)
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        Odometry.append((msg.pose.pose.position.x,msg.pose.pose.position.y))

    print(len(Odometry))
    gmap.plot(*zip(*Odometry), colour, edge_width=5)

if __name__ == "__main__":

    # Initial Coordinates of the map
    lat = 59.403083
    lon = 17.958198

    # Create the map plotter:
    apikey = 'apikey'
    gmap = gmplot.GoogleMapPlotter(lat, lon, 18, apikey=apikey, map_type='hybrid')

    filename = '/home/marco/Pictures/AEKF5-NoNMEA-BWB/2021-06-15-00-03-24-NoNMEA-BWB.bag'

    bag = rosbag.Bag(filename)

    topics = bag.get_type_and_topic_info().topics

    #plotOdometry(gmap, bag, "/Odom_AEKF", "red")

    if('/GPSfix' in topics):
        plotNatSav(gmap, bag, '/GPSfix', 'red')
    if('/gnss_left/fix' in topics):
        plotNatSav(gmap, bag, '/gnss_left/fix', 'green')
    if('/gnss_right/fix' in topics):
        plotNatSav(gmap, bag, '/gnss_right/fix', 'blue')

    bag.close()



    print("Save Map")
    # Draw the map to an HTML file:
    gmap.draw(filename+'.html')

