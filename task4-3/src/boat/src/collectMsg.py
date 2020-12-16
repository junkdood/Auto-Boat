#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import struct
import socket
from sensor_msgs.msg import NavSatFix
import string 


class SIM():
	def __init__(self):
		self.latitude = 0.0
		self.longitude = 0.0
		self.ID = -1
		self.client = mqtt.Client()
		self.pub = rospy.Publisher('/simulated', NavSatFix, queue_size=1)
		rospy.init_node("talker", anonymous=True)
	
	def on_connect(self, client, userdata, flags, rc):
		self.client.subscribe("/usv/simulated")
		print("connect success")

	def on_message(self, client, userdata, msg):
		info = struct.unpack('!cdd',msg.payload)
		self.ID = info[0]
		self.longitude = info[2]
		self.latitude = info[1]

		current_fix = NavSatFix()
		current_fix.latitude = self.latitude
		current_fix.longitude = self.longitude
		current_fix.position_covariance[1] = self.ID[0]
		print("latitude ", self.latitude)
		print("longitude ", self.longitude)
		print("heading ", self.ID[0])
		self.pub.publish(current_fix)
		print("finish publish")


	def worker(self):
		self.client.on_connect = self.on_connect
		self.client.on_message = self.on_message
		self.client.connect("192.168.0.11", 1883, 600)
		self.client.loop_forever()

sim = SIM()
sim.worker()