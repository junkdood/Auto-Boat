#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import paho.mqtt.client as mqtt
import struct
from geometry_msgs.msg import Vector3

# client用于与船进行连接，设置为全局变量
client = mqtt.Client()
client.connect("192.168.0.11",1883,600)

# 订阅节点的回调函数，在收到信息后向船发送控制信息
def callback(data):
    print('callbacking')
    # 对接收到的消息进行处理并打包
    topic = '/ctrl'
    p = 50
    v = data.x
    if v > 0.8:
        v = 0.8
    elif v < -0.8:
        v = -0.8
    else:
        v = v
    
    r = data.y
    if r > 1:
        r = 1
    elif r < -1:
        r = -1
    else:
        r = r
    # v = 0.5
    # r = 0.2
    print('v',v)
    print('r',r)
    # 指令需要按照主控数据交换里的格式进行打包
    message = struct.pack('>ffB',v,r,p)
    # 向船的topic节点发布指令
    client.publish(topic, payload=message)
    # loop函数很重要，用于保持与船的连接，如果删除，则指令将发送失败
    client.loop()
    print('publish success',v,r,p)

# 订阅control节点
def listener():
    # 节点初始化
    rospy.init_node('control', anonymous=True)
    print('init success')
    # 节点订阅
    rospy.Subscriber("vtg", Vector3, callback)
    print('callback success')
    rospy.spin()

listener()