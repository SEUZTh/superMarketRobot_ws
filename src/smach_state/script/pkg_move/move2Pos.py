#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from webots_ros.msg import move


class move2Pos:
    def __init__(self, moveType="", speed=0.0, des_pos=(0.0, 0.0), alphe=0.0):
        self.moveType = moveType
        self.speed = speed
        self.des_pos = des_pos
        self.alphe = alphe

    def move2Pos_pub(self):
        pub = rospy.Publisher("/youbot/move", move, queue_size=1)
        mv = move()
        mv.moveType = self.moveType
        mv.speed = self.speed
        mv.x = self.des_pos[0]
        print mv.x

        mv.z = self.des_pos[1]
        print mv.z
        mv.alphe = self.alphe
        rospy.sleep(0.3)
        pub.publish(mv)


if __name__ == '__main__':
    rospy.init_node('move2Pos_pub', anonymous=False)
    # m = move2Pos("Goto", 0.05, (1.3, 0), 1.5)
    # m.move2Pos_pub()

    # 货架 1 拍照位置
    m = move2Pos("Go forward", 0.01, (1.1, 0), 0)
    m.move2Pos_pub()
    rospy.sleep()