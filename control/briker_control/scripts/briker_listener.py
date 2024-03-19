#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dataclasses import dataclass
from mimetypes import common_types
import rospy 
from std_msgs.msg import Int16

#启动和关闭的command还没传递
def walk_callback_x_move(data):
    x_move = data
def walk_callback_y_move(data):
    y_move = data
def walk_callback_angle(data):
    angle =data
def walk_callback_y_speed(data):
    y_speed = data
def walk_callback_x_speed(data):
    x_speed = data
def walk_callback_angle_speed(data):
    angle_speed = data
def walk_callback_command_all(data):
    command = data
def walk_callback_command_under(data):
    command_under_start=data
def walk_callback_next_mode(data):
    next_mode = data
def walk_callback_command_arm_start(data):
    command_arm_start = data
def walk_callback_command_arm_close(data):
    common_arm_close = data
def listener():
    rospy.init_node("briker_listener",anonymous=True)
    walk_sub_x_move = rospy.Subscriber("/briker_strategy_node/walkParam_x_move",Int16,walk_callback_x_move)
    walk_sub_y_move = rospy.Subscriber("/briker_strategy_node/walkParam_y_move",Int16,walk_callback_y_move)
    walk_sub_angle = rospy.Subscriber("/briker_strategy_node/walkParam_angle",Int16,walk_callback_angle)
    walk_sub_x_speed = rospy.Subscriber("/briker_strategy_node/walkParam_x_speed",Int16,walk_callback_x_speed)
    walk_sub_y_speed = rospy.Subscriber("/briker_strategy_node/walkParam_y_speed",Int16,walk_callback_y_speed)
    walk_sub_angle_speed = rospy.Subscriber("/briker_strategy_node/walkParam_angle_speed",Int16,walk_callback_angle_speed)
    walk_sub_command_all=rospy.Subscriber("/briker_strategy_node/command",Int16,walk_callback_command_all)
    walk_sub_command_under=rospy.Subscriber("/briker_strategy_node/command2",Int16,walk_callback_command_under)
    walk_sub_next_mode=rospy.Subscriber("/briker_strategy_node/armMode",Int16,walk_callback_next_mode)
    walk_sub_command_arm_start=rospy.Subscriber("/briker_strategy_node/commmand3",Int16,walk_callback_command_arm_start)
    walk_sub_command_arm_close=rospy.Subscriber("/briker_strategy_node/command3_no",Int16,walk_callback_command_arm_close)
    rospy.spin()
    
#不需要add那一步，只要install python里面的操作 
if __name__=="__main__":
    listener()
