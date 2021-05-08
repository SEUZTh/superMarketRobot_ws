#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from webots_ros.msg import arm


class arm2Pos:
    def __init__(self,
                 type="normal",
                 grabFlag=True,
                 operateFlag=[False] * 5,
                 angle=[0.] * 5):
        self.type = type
        self.grabFlag = grabFlag
        self.operateFlag = operateFlag
        self.angle = angle

    def arm2Pos_pub(self):
        pub = rospy.Publisher("/youbot/arm", arm, queue_size=1)
        mv = arm()
        mv.type = self.type
        mv.grabFlag = self.grabFlag
        mv.operateFlag = self.operateFlag
        mv.angle = self.angle
        rospy.sleep(0.3)
        pub.publish(mv)


if __name__ == '__main__':
    rospy.init_node('arm2Pos_pub', anonymous=False)
    m = arm2Pos("preGrab", True, [False] * 5, [0.] * 5)
    m.arm2Pos_pub()
