#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import smach
import smach_ros


class toFindEmptyShelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['outcome_empty', 'outcome_full'],
            input_keys=['shelfPos_in',
                        'tfes_full_shelf_count_in'],  # shelfPos_in: 拍照位置
            output_keys=[
                'empty_shelfPos_out', 'tfes_objName_out',
                'tfes_full_shelf_count_out'
            ])  # empty_shelfPos_out： 空货架空格位置; tfes_objName_out: 需要去拿的商品的名称

    def execute(self, userdata):
        rospy.loginfo('Executing state toFindEmptyShelf...')
        # 到一个货架的前面
        #

        # 识别空货架位置及需要摆放的物体
        #
        isEmpty = False
        if isEmpty == True:
            return 'outcome_empty'
        else:
            userdata.tfes_full_shelf_count_out = userdata.tfes_full_shelf_count_in + 1
            return 'outcome_full'


class toCarryObject(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['outcome_re_recog'],
            input_keys=['objName_in', 'shelfPos_in'
                        ])  # objName_in: 需要去拿的商品的名称; shelfPos_in: 需要被摆放的货架空格位置

    def execute(self, userdata):
        rospy.loginfo('Executing state toCarryObject...')
        # 到一个制定的物体前
        #

        # 抓住物体
        #

        # 前往货架
        #

        # 放在货架指定位置上
        #

        return 'outcome_re_recog'  # 再次识别该货架


class reRecogShelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_repeat'])

    def execute(self, userdata):
        rospy.loginfo('Executing state reRecogShelf...')
        return 'outcome_repeat'


class toNextShelf(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['outcome_next_shelf', 'outcome_end'],
            input_keys=[
                'full_shelf_count_in'  # full_shelf_count_in: 已经装满了的货架的位置)
            ])

    def execute(self, userdata):
        rospy.loginfo('Executing state toNextShelf...')

        if userdata.full_shelf_count_in < 4:
            return 'outcome_next_shelf'
        else:
            return 'outcome_end'


def main():
    rospy.init_node('smach_state_machine')
    sm = smach.StateMachine(outcomes=['outcome_finish'])
    # 站在此位置对货架进行识别，类型为浮点型
    sm.userdata.dest_shelf_pos = [[(1.11458, -0.036635, -0.0867876),
                                   (0., 0., 0., 0.)], [(1, 1, 0),
                                                       (0, 0, 0, 0)],
                                  [(2, 2, 0), (0, 0, 0, 0)],
                                  [(3, 3, 0), (0, 0, 0, 0)]]
    # 空货架位置声明
    sm.userdata.obj_put_pos = [(-1, -1, -1), (0, 0, 0, 0)]
    # 需要去搬运的商品的名称
    sm.userdata.carry_obj_name = ""
    # 已经装满了的货架的数量
    sm.userdata.full_shelf_count = 0
    with sm:
        smach.StateMachine.add('__toFindEmptyShelf',
                               toFindEmptyShelf(),
                               transitions={
                                   'outcome_empty': '__toCarryObject',
                                   'outcome_full': '__toNextShelf'
                               },
                               remapping={
                                   'shelfPos_in': 'dest_shelf_pos',
                                   'tfes_full_shelf_count_in':
                                   'full_shelf_count',
                                   'empty_shelfPos_out': 'obj_put_pos',
                                   'tfes_objName_out': 'carry_obj_name',
                                   'tfes_full_shelf_count_out':
                                   'full_shelf_count'
                               })
        smach.StateMachine.add(
            '__toCarryObject',
            toCarryObject(),
            transitions={'outcome_re_recog': '__reRecogShelf'},
            remapping={
                'objName_in': 'carry_obj_name',
                'shelfPos_in': 'dest_shelf_pos'
            })
        smach.StateMachine.add(
            '__reRecogShelf',
            reRecogShelf(),
            transitions={'outcome_repeat': '__toFindEmptyShelf'})
        smach.StateMachine.add(
            '__toNextShelf',
            toNextShelf(),
            transitions={
                'outcome_next_shelf': '__toFindEmptyShelf',
                'outcome_end': 'outcome_finish'
            },
            remapping={'full_shelf_count_in': 'full_shelf_count'})
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
