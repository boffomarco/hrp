#!/usr/bin/env python3

import rosbag

import rospy

import gmplot

print("Open Plotter")

lat = 59.40963333
lon = 17.94516667

# Create the map plotter:
apikey = 'AIzaSyCKYrM5uXHCxsWOHNv-bxW6DMutzuunexg'
gmap = gmplot.GoogleMapPlotter(lat, lon, 18, apikey=apikey, map_type='hybrid')



print("Open RosBag")

#filename = '/home/marco/Videos/OutTests/KistaGardPark_2021-03-18-16-32-32__th0.3-lat59.406827-long17.940565.bag' # KistaGardPark_out0-157m
#filename = '/home/marco/Videos/OutTests/KistaGardPark_2021-03-18-16-39-14__th-0.22-lat59.406820-long17.940523.bag' # KistaGardPark_out1-390m
#filename = '/home/marco/Videos/OutTests/KistaParkLot_2021-03-19-17-15-38__th-2.76-lat59.403067-long17.958173.bag' # KistaParkLot_out1-37m
#filename = '/home/marco/Videos/OutTests/KistaParkLot_2021-03-19-17-04-41__th-2.76-lat59.403070-long17.958150.bag' # KistaParkLot_out0-77m
filename = '/home/marco/Videos/2021-04-01-15-31-27.bag'

bag = rosbag.Bag(filename)

print("Read RosBag")

info = bag.get_type_and_topic_info()
#print(info.topics)
topics = []
for topic in info.topics:
    #print(topic)
    topics.append(str(topic))
#print(topics)

print("Plot Lines")
if('/GPSfix' in topics):
    GPSfix = list()
    for topic, msg, t in bag.read_messages(topics=['/GPSfix']):
        GPSfix.append((msg.latitude,msg.longitude))

    gmap.plot(*zip(*GPSfix), 'red', edge_width=5)


if('/gnss_right/fix' in topics):
    GPSfix = list()
    for topic, msg, t in bag.read_messages(topics=['/gnss_right/fix']):
        GPSfix.append((msg.latitude,msg.longitude))

    gmap.plot(*zip(*GPSfix), 'blue', edge_width=3)


if('/gnss_left/fix' in topics):
    GPSfix = list()
    for topic, msg, t in bag.read_messages(topics=['/gnss_left/fix']):
        GPSfix.append((msg.latitude,msg.longitude))

    gmap.plot(*zip(*GPSfix), 'blue', edge_width=3)

bag.close()

print("Close RosBag")


print("Save Map")

# Draw the map to an HTML file:
gmap.draw(filename+'.html')

