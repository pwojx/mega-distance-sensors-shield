#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range

from time import sleep

import serial

mega = serial.Serial(
    port='/dev/ttyMEGA0',
    baudrate=115200
)

pub_tfmini=[]
for i in range(4):
    pub_tfmini.append(rospy.Publisher('mega_driver/tfmini/scan_'+str(i), Range, queue_size=10))

pub_pololu=[]
for i in range(4):
    pub_pololu.append(rospy.Publisher('mega_driver/pololu/scan_'+str(i), Range, queue_size=10))

rospy.init_node('mega_driver', anonymous=False)

msg = Range()

def publishers():
    rate = rospy.Rate(1000) # 1 kHz
    while not rospy.is_shutdown():

        if mega.isOpen():
            #data = mega.read_until()
            data = mega.readline()
            data = data.split(',')
            try:
                data = map(int, data)
            except ValueError:
                data = [-1 for _ in range(8)]

        for i, pub in enumerate(pub_pololu):
            msg.range = data[i]
            pub.publish(msg)

        msg.range = data[5]
        pub_tfmini[0].publish(msg)

        msg.range = data[6]
        pub_tfmini[1].publish(msg)

        msg.range = data[7]
        pub_tfmini[2].publish(msg)

        msg.range = data[4]
        pub_tfmini[3].publish(msg)

        mega.flush()
        rate.sleep()


if __name__ == '__main__':
    try:
        publishers()

    except rospy.ROSInterruptException:
        pass
