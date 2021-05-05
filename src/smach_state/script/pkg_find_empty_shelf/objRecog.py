#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from webots_ros.msg import RecognitionObject


class objRecog():
    def __init__(self):
        self.count = 0
        self.count1 = 0
        self.obj_recog_msg = None
        self.empty_shelf = -1

    def countObj(self):
        seq = [[-1], [-1]]
        for i in range(2):
            try:
                self.obj_recog_msg = rospy.wait_for_message(
                    '/supermarketRobot/camera_front/recognition_objects',
                    RecognitionObject,
                    timeout=20)
                temp = str(self.obj_recog_msg.header)
                temp = temp.split('seq: ')[1]
                temp = temp.split('stamp:')[0]
                seq[i] = int(temp)
            except:
                print 'Get objects\' postion msg timeout!'
                flag = False
                pass

        if seq[0] != -1 and seq[1] != -1:
            self.count = seq[1] - seq[0]
            print("object_count = %d" % self.count)
            self.empty_shelf = 14 - self.count
            print("empty_shelf_num = %d" % self.empty_shelf)
            # self.obj_pos = np.arange(self.count * 3).reshape(self.count, 3)
            self.obj_pos = np.zeros((self.count, 3))

    def getObjPos(self):
        self.obj_recog_msg = rospy.Subscriber(
            '/supermarketRobot/camera_front/recognition_objects',
            RecognitionObject, self.callback)
        rospy.spin()

    def callback(self, msg):
        temp = str(msg.position_on_image)
        temp = temp.split('x: ')[1]
        x = float(temp.split('\ny:')[0])
        temp = temp.split('y: ')[1]
        y = float(temp.split('\nz:')[0])
        z = float(temp.split('z: ')[1])
        self.count1 = self.count1 % self.count
        self.obj_pos[self.count1][0] = x
        self.obj_pos[self.count1][1] = y
        self.obj_pos[self.count1][2] = z
        self.count1 = self.count1 + 1
        self.obj_pos = np.lib.arraysetops.unique(self.obj_pos,
                                                 axis=0)  # 删除重复的行
        while np.size(self.obj_pos, 0) < self.count:
            add_a_line = np.array([[0, 0, 0]])
            self.obj_pos = np.r_[self.obj_pos, add_a_line]

        print self.obj_pos


if __name__ == '__main__':
    rospy.init_node('obj_recog_sub', anonymous=False)
    o = objRecog()
    o.countObj()
    o.getObjPos()