#!/usr/bin/env python
import rospy
from pyproj import Proj, transform
import numpy as np
from math import cos, sin, pi
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT

#Projection definition
#UTM-K
proj_UTMK = Proj(init='epsg:5178')
#WGS1984
proj_WGS84 = Proj(init='epsg:4326')

class Tm_heading:
    def __init__(self):
        self.pubtm = rospy.Publisher('current_tm', Pose, queue_size=1)
        self.pubhead = rospy.Publisher('heading', Float64, queue_size=1)
        self.subtm = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.tm,queue_size=1)
        self.subheading = rospy.Subscriber("ublox_gps/navpvt", NavPVT, self.heading,queue_size=1)
        self.run()

    def tm(self,Fix):
        current_tm= Pose()
        lon=Fix.longitude
        lat=Fix.latitude
        x, y = transform(proj_WGS84, proj_UTMK, lon, lat)
        current_tm.position.x = x
        current_tm.position.y = y
        self.pubtm.publish(current_tm)

    def heading(self,head):
        heading = Float64()
        heading.data=5*pi/2 - np.deg2rad(float(head.heading / 100000))
        self.pubhead.publish(heading)

    def run(self):
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            
            rate.sleep()


def main():
    rospy.init_node('tm_heading',anonymous=True)
    Tm_heading()
    

if __name__ == "__main__":
    
    main()