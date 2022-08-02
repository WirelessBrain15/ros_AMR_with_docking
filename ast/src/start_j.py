#!/usr/bin/env python

import rospy as rp
from std_msgs.msg import Empty


def p():
    pub=rp.Publisher('/start_journey',Empty,queue_size=10)
    rate=rp.Rate(10)
    while not rp.is_shutdown():
       pub.publish()
       rate.sleep()
if __name__ =='__main__':
     try:
         rp.init_node('wav',anonymous=True)
         p()
     except :
         pass
