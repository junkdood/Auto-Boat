#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
import csv
import time
import struct
from turtlesim.msg import Color
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
import casadi as ca
import casadi.tools as ca_tools
import numpy as np
from threading import Thread
import xlwt
from matplotlib import pyplot as plt



def gps_to_mkt1(longitude,latitude):
	x = longitude*20037508.34/180
	y = math.log(math.tan((90+latitude)*math.pi/360))/(math.pi/(180))
	y = y*20037508.34/180
	return x,y

'''
mpc local planning 部分参数
'''
T = 0.3  # sampling time [s]
# 预测时域
N = 15  # prediction horizon
# USV的半径
rob_diam = 0.2  # [m]
# USV允许的最大速度
v_max = 5
# USV允许的最大角速度
omega_max = np.pi*0.8

# 状态变量权重矩阵
Q = np.array([[5.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 0.1]])
# 控制变量权重矩阵
R = np.array([[0.5, 0.0], [0.0, 0.05]])

# cost function 对应目标的权重
goal_weight = 5 # 达到当前给定目标点的权重
delta_control_weight = 0.05 # 控制变化量的权重，越大则使得控制量不能变化过快
ref_weight = 30 # 贴合参考轨迹的权重
dynamic_weight = 10000 # 运动学约束
obs_avoid_weight = 12000 # 远离障碍物的权重，软约束
obs_safe_dist = 5 # 远离障碍物的安全距离，硬约束

## 优化器迭代求解的次数
iter_num = 300 

## 状态量x,y,theta的范围
x_lowerbound = -np.inf
x_upperbound = np.inf
y_lowerbound = -np.inf
y_upperbound = np.inf
theta_lowerbound = -np.inf
theta_upperbound = np.inf

# 规划路径刷新频率
map_frequence = 4
'''
全局变量
'''
## 用于记录log文件，便于通过数据进行debug
file_path = r'data_log.csv'
data_to_log = ["pointx","pointy","angle","time","goal_num","target_num","ki","heading_dis","et","decide_angle","speed","w","miss","max_miss"] #存储文件的内容

## 船的默认初始位置
boat = [0, 0, 0, 0] #pointx,pointy,heading,time

## stanley预瞄点的位置
target_num = 0

## 判断是否达到最终目标点
had_arrive_the_dis = False
## 判断是否达到当前目标点
if_arrived_current = False

# planning 规划出来的点，大小取决于预测时域
target_pointx = []  
target_pointy = []

# 定义要到达的目标点，经纬度（可以自行添加多个目标点）
x1,y1 = gps_to_mkt1(113.69978958,22.01934276)
x2,y2 = gps_to_mkt1(113.69839026,22.02088831)
des = [[x1,y1,0],[x2,y2,0]]
## 这里是最终的目标点，用在判断是否需要继续规划的过程中
destination = des[-1]

# 用于记录那些目标点已经经过（目标点的列表是按照顺序插入的）
goal_num = 0

'''
感知部分
'''
# 激光得到的障碍物位置信息，第一个是x，第二个是y
obs = [[],[]]

# 图片处理后发出的决策信息
deci = False
'''
stanley 控制部分
'''
decide_angle = 0
## 存储规划出来的路径上每个相邻点之间的角度
points_angle = []
## 某某系数，我不懂
ki = 0.2
## 某某最大误差？不懂
max_miss = 0

#马达动力百分比
pub_speed = 0.6 

obsfi = 100 #两个障碍物距离的平方小于这个数时，被认为是同一个障碍物

goalsize = 5 #离目标点多远时认为已经到达

# 记录数据
def write_data():
	with open(file_path, "a+") as log_file:
		writer = csv.writer(log_file)
		writer.writerow(data_to_log)



# 在给定点的一周得到五个新的目标点，使得可以闭合的绕圈运动
def round_goal(point):
	global boat,goal_num,des
	p1 = [point[0] - 6,point[1] - 6,0]
	p2 = [point[0] - 6,point[1] + 6,0]
	p3 = [point[0] + 6,point[1] + 6,0]
	p4 = [point[0] + 6,point[1] - 6,0]
	des.insert(-1,p1)
	des.insert(-1,p4)
	des.insert(-1,p3)
	des.insert(-1,p2)
	des.insert(-1,p1)

# 在给定点的左侧得到一个新的目标点
def left_goal(point):
	global boat,goal_num,des
	if point[0] == boat[0]:
		des.insert(-1,[point[0],point[1]+5,0])
	else:
		theta = math.atan((point[1]-boat[1])/(point[0]-boat[0]))
		phi = math.sqrt((point[1]-boat[1])**2 + (point[0]-boat[0])**2)
		if point[0]-boat[0] < 0:
			theta -= math.pi
		theta += 15/180*math.pi
		x = boat[0] + phi*math.cos(theta)
		y = boat[1] + phi*math.sin(theta)
		des.insert(-1,[x,y,0])

# 在给定点的右侧得到一个新的目标点
def right_goal(point):
	global boat,goal_num,des
	if point[0] == boat[0]:
		des.insert(-1,[point[0],point[1] - 5,0])
	else:
		theta = math.atan((point[1]-boat[1])/(point[0]-boat[0]))
		phi = math.sqrt((point[1]-boat[1])**2 + (point[0]-boat[0])**2)
		if point[0]-boat[0] < 0:
			theta += math.pi
		theta -= 15/180*math.pi
		x = boat[0] + phi*math.cos(theta)
		y = boat[1] + phi*math.sin(theta)
		des.insert(-1,[x,y,0])

# 在给定两点的中间得到一个新的目标点
def center_goal(i,j):
	global goal_num,des,obs
	des.insert(-1,[(obs[0][i]+obs[0][j])/2,(obs[1][i]+obs[1][j])/2,0])

# 障碍物铝箔，false表示需要滤掉
def obs_filter(point):
	global obs,goal_num,des
	for i in range(len(obs[0])):
		dist = (point[0] - obs[0][i])**2 + (point[1] - obs[1][i])**2
		if dist < obsfi:
			obs[0][i] = (point[0] + obs[0][i])/2
			obs[1][i] = (point[1] + obs[1][i])/2
			return False
	return True

class mpc_stanly_com(object):
	# 初始化和接受信息的部分
	def __init__(self):
		self.pub_control = rospy.Publisher('vtg',Vector3,queue_size=1000)
		self.flag = 0	# 记录收到gps的次数
		print('init has been done\n')

	def listener(self):
		rospy.Subscriber("unionstrong/gpfpd",NavSatFix,self.get_gps)
		rospy.Subscriber("filtered_points",Float64MultiArray,self.get_obs,queue_size=1)
		#rospy.Subscriber("ballcolor",Color,self.get_deci)
		print('start to listen the msg and MPCing\n')
		rospy.spin()


	def gps_to_mkt(self,longitude,latitude):
		x = longitude*20037508.34/180
		y = math.log(math.tan((90+latitude)*math.pi/360))/(math.pi/(180))
		y = y*20037508.34/180
		return x,y

	# 事实证明效果不太行
	def if_change_ref(self,x1,y1,x2,y2):
		return True
		# error = 0
		# thr = planfi
		# for i in range(len(x1)):
		# 	error = error + (x1[i]-x2[i])**2 + (y1[i]-y2[i])**2
		# if error < thr:
		# 	return True
		# else:
		# 	return False

	# 回调函数u，接收到视觉的节点消息，就会执行一次
	def get_deci(self, msg):
		print("strat recv deci\n")
		global deci
		print('======================================================')
		print("color : r g b",msg.r,msg.b,msg.g)
		print('======================================================')
		if msg.r == 1 or msg.g == 1:
			deci = True 
		else:
			deci = False	


	# 回调函数，接收到障碍物位置的节点消息，就会执行一次
	def get_obs(self, msg):
		print("strat recv obs\n")
		local_pos = msg.data
		print("num of ob:",len(local_pos),len(local_pos)/2)
		for i in range(int(len(local_pos)/2)):
			localx = local_pos[2*i]
			localy = local_pos[2*i+1]
			gox=boat[0] - localx*math.cos(boat[2])-localy*math.sin(boat[2])
			goy=boat[1] - localy*math.cos(boat[2])+localx*math.sin(boat[2])
			# 滤波器，判断是否是同一障碍物
			if obs_filter([gox,goy]):
				obs[0].append(gox)
				obs[1].append(goy)			

				# 执行不同的决策动作
				if len(obs[0]) == 2:
					center_goal(0,1)
				elif len(obs[0]) == 4:
					left_goal([gox,goy])
				elif len(obs[0]) == 5:
					round_goal([gox,goy])
				elif len(obs[0]) == 6:
					right_goal([gox,goy])
				elif len(obs[0]) == 8:
					center_goal(5,6)
				else:
					pass

				
	# 回调函数u，接收到gps的节点消息，就会执行一次
	def get_gps(self, msg):
		global had_arrive_the_dis


		print("strat recv gps\n")

		heading = msg.position_covariance[1]
		# if boat[3] == msg.position_covariance[0]:
		# 	print('收到相同的时间戳\n')

		boat[0],boat[1] = self.gps_to_mkt(msg.longitude,msg.latitude) # 坐标系转换
		boat[2] = heading/180*math.pi #弧度制
		boat[3] = msg.position_covariance[0] #对应的时间戳
		data_to_log[0] = boat[0]
		data_to_log[1] = boat[1]
		data_to_log[2] = boat[2]
		data_to_log[3] = boat[3]


		# 坐标系角度变换（从墨卡托坐标系转换到planning的坐标系）
		if boat[2] < -np.pi/2 and boat[2] > -np.pi:
			tempan = -boat[2] - 3*np.pi/2
		else:
			tempan = -boat[2] + np.pi/2

		if not had_arrive_the_dis:
			if self.flag % map_frequence == 0: # 规划路径刷新频率，过快的话计算量会变大
				temp_target_pointx,temp_target_pointy = planning([boat[0], boat[1], tempan], des[goal_num], 20)
				# 滤波，消除变化过大的轨迹
				if self.if_change_ref(temp_target_pointx,temp_target_pointy,target_pointx,target_pointy) :
					target_pointx = temp_target_pointx
					target_pointy = temp_target_pointy

			# 判断是否到达当前目标点，到达则更换下一目标点
			if self.get_dis([boat[0], boat[1]], des[goal_num][:2]) < goalsize:
				goal_num += 1
				target_pointx, target_pointy = planning([boat[0], boat[1], tempan], des[goal_num], 20)

			# 判断是否到达最终目标点
			if self.get_dis([boat[0], boat[1]], destination) < 5:
				had_arrive_the_dis = True

			print('================================================')
			print('obs:  ',obs)
			print('des: ',des,'  goal_num:  ',goal_num)
			print('================================================')
			self.flag +=1
			self.point_pre_process()
			self.find_future_point()
			self.find_deci_angle()
			self.change_motor()
			write_data()
		return

	# 获得两个点之间的角度
	def get_angle(self,point_pre, point_post):
		temp_x = point_post[0] - point_pre[0]
		temp_y = point_post[1] - point_pre[1]
		temp_angle = math.atan2(temp_x, temp_y)

		return temp_angle

	# 获得两个角度之间的差
	def get_angle_dis(self,angle1, angle2):  # 逆时针方向为负，顺时针为正
		temp_dis = angle2 - angle1
		if temp_dis < -1 * math.pi:
			return temp_dis + 2 * math.pi
		elif temp_dis > math.pi:
			return temp_dis - 2 * math.pi
		else:
			return temp_dis


	# 获得相邻两点之间的角度
	def point_pre_process(self):
		points_angle.clear()
		i = 0
		while i < len(target_pointx) - 1:
			points_angle.append(
				self.get_angle([target_pointx[i], target_pointy[i]], [target_pointx[i + 1], target_pointy[i + 1]]))
			i = i + 1


	# 得到两点之间的角度
	def get_dis(self,point_pre, point_post):
		return math.sqrt(math.pow(point_post[0] - point_pre[0], 2) + math.pow(point_post[1] - point_pre[1], 2))

	# 寻找当前状态下的预瞄点
	def find_future_point(self):
		i = 0
		temp_dis = 1000000 # temp 用于找距离最小的点
		while i < len(target_pointx) - 3:
			dis = self.get_dis([target_pointx[i], target_pointy[i]], [boat[0], boat[1]])
			if temp_dis > dis:
				temp_dis = dis
				target_num = i + 3  # 3 这里可以调参，选择合适的预瞄点
			i = i + 1

		data_to_log[5] = target_num
		return

	# 得到转向的决策量
	def find_deci_angle(self):
		delta1 = self.get_angle_dis(points_angle[target_num], boat[2])

		dis_u2p = self.get_dis([boat[0], boat[1]], [target_pointx[target_num], target_pointy[target_num]])

		degree_usv = self.get_angle([boat[0], boat[1]], [target_pointx[target_num], target_pointy[target_num]])

		judge_degree = -self.get_angle_dis(points_angle[target_num], degree_usv)

		if judge_degree < -math.pi / 2:
			temp_judge_degree = -judge_degree - math.pi
		elif judge_degree > math.pi / 2:
			temp_judge_degree = math.pi - judge_degree
		else:
			temp_judge_degree = -judge_degree
		e_dis = dis_u2p * math.sin(temp_judge_degree)

		if math.fabs(e_dis) > max_miss:
			max_miss = math.fabs(e_dis)

		et = ki * e_dis / 3
		decide_angle = et + delta1

		# 记录数据
		data_to_log[6] = ki
		data_to_log[7] = delta1
		data_to_log[9] = decide_angle
		data_to_log[8] = et
		data_to_log[12] = e_dis
		data_to_log[13] = max_miss
		return

	# 发送控制量，一个ros消息
	def change_motor(self):
		current_cmd = Vector3()
		current_cmd.x = pub_speed #恒速
		if decide_angle >= 0:
			w = -math.sqrt(decide_angle/math.pi)
		else:
			w = math.sqrt(math.fabs(decide_angle/math.pi))
		if math.fabs(w)>1:
			current_cmd.y = w/math.fabs(w)
		else:
			current_cmd.y = w

		self.pub_control.publish(current_cmd)
		
def plot_ponit():
	# 用于存储船的实时位置
	x=[]
	y=[]
	plt.ion()
	while True:
		plt.clf()
		plt.axis('equal')
		plt.xlim(boat[0] - 30, boat[0] + 30) #画图界面显示范围
		plt.ylim(boat[1] - 50, boat[1] + 30)
		plt.xlabel(boat[0])
		plt.ylabel(boat[1])
		plt.title(goal_num)
		for i in range(len(des)):	# 目标点
			plt.plot(des[i][0],des[i][1],'go')
		plt.plot(obs[0],obs[1], "ro")	# 障碍物点

		plt.plot(boat[0], [boat[1]], '*')
		plt.plot(target_pointx[target_num], target_pointy[target_num], 'o')	# mpc实时规划点、两个预瞄点
		plt.plot(target_pointx[target_num + 2], target_pointy[target_num + 2], '^')
		plt.plot(target_pointx, target_pointy, '.')

		x.append(boat[0])
		y.append(boat[1])
		plt.plot(x,y, 'b.')

		plt.pause(0.01)
		plt.ioff()


def planning(x_c,x_s,N_pre):

	# 按照casadi的格式，声明状态变量，x，y，theta航向角
	x = ca.SX.sym('x')
	y = ca.SX.sym('y')
	theta = ca.SX.sym('theta')
	states = ca.vertcat(x, y)
	states = ca.vertcat(states, theta)  # 构建小车状态向量
	# n_states表示状态变量的个数
	n_states = states.size()[0]

	# 按照casadi的格式，声明控制变量，v，omega角速度
	v = ca.SX.sym('v')
	omega = ca.SX.sym('omega')
	controls = ca.vertcat(v, omega)
	# n_controls表示控制量的个数
	n_controls = controls.size()[0]

	# 按照casadi的模板定义状态方程
	## rhs
	rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta))  
	rhs = ca.vertcat(rhs, omega)

	## function,[]输入，[]输出
	f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

	## for MPC
	U = ca.SX.sym('U', n_controls, N)
	X = ca.SX.sym('X', n_states, N + 1)  # 通常比控制多1
	P = ca.SX.sym('P', n_states + n_states)  # 在这里每次只需要给定当前/初始位置和目标终点位置

	#### cost function

	obj = 0  # cost
	g = []  # equal constrains

	g.append(X[:, 0] - P[:3]) # 约束初始状态为已知的传入函数的初始状态

	for i in range(N):
		## 距离目标点状态的cost，只考虑几何坐标x,y
		state_error = (X[0, i] - P[3]) ** 2 + (X[1, i] - P[4]) ** 2

		## 控制的变化的cost
		control_error = (U[0, i]) ** 2 + (U[1, i]) ** 2

		## 靠近ref的cost
		if goal_num == 0:
			horizon_error = 0
		else:
			horizon_error = ((des[goal_num][1] - des[goal_num - 1][1]) * X[0, i] - (des[goal_num][0] - des[goal_num - 1][0]) * X[1, i]+ (des[goal_num][0] * des[goal_num - 1][1] - des[goal_num][1] * des[goal_num - 1][0])) ** 2 / ((des[goal_num][0] - des[goal_num - 1][0]) ** 2 + (des[goal_num][1] - des[goal_num - 1][1]) ** 2)
		
		## 如果能够得出运动状态方程，比如这里将船当成差速模型。得出下一个状态的状态量
		x_next_ = f(X[:, i], U[:, i]) * T + X[:, i]

		## 尽可能满足运动学约束的cost，这里设置为软约束
		dynamic_error = (X[0,i+1] - x_next_[0])**2 + (X[0,i+1] - x_next_[0])**2 + (X[1,i+1] - x_next_[1])**2

		## 导航的cost
		obj = obj + goal_weight * state_error + delta_control_weight * control_error + ref_weight * horizon_error + dynamic_weight * dynamic_error
		
		## 这里是将运动学约束变为硬约束，必须满足
		#g.append(X[:, i + 1] - x_next_)

	## 避障的约束
	for i in range(N + 1):
		for j in range(len(obs[0])):
			## 软约束
			obj = obj + obs_avoid_weight/ca.sqrt((X[0, i] - obs[0][j]) ** 2 + (X[1, i] - obs[1][j]) ** 2)

			## 硬约束
			#g.append(ca.sqrt((X[0, i] - obs[0][j]) ** 2 + (X[1, i] - obs[1][j]) ** 2))

	opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

	nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
	opts_setting = {'ipopt.max_iter': iter_num, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-8,
					'ipopt.acceptable_obj_change_tol': 1e-6}

	solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

	# subject to 部分定义
	## 这里要注意g这个list里面的添加顺序
	lbg = []
	ubg = []
	lbx = []
	ubx = []
	# 初始状态
	lbg.append(0.0)
	lbg.append(0.0)
	lbg.append(0.0)
	ubg.append(0.0)
	ubg.append(0.0)
	ubg.append(0.0)

	## 此处为避障的硬约束
	# for _ in range(N + 1):
	# 	for i in range(len(obs[0])):
	# 		lbg.append(obs_safe_dist)  # safe distance
	# 		ubg.append(np.inf)

	# 速度和角速度的范围约束
	for _ in range(N):
		lbx.append(0)
		lbx.append(-omega_max)
		ubx.append(v_max)
		ubx.append(omega_max)
	
	# x，y，theta的范围约束
	for _ in range(N + 1):
		lbx.append(x_lowerbound)
		lbx.append(y_lowerbound)
		lbx.append(theta_lowerbound)
		ubx.append(x_upperbound)
		ubx.append(y_upperbound)
		ubx.append(theta_upperbound)

	# Simulation
	x0 = np.array(x_c).reshape(-1, 1)  # initial state
	x0_ = x0.copy()
	x_m = np.zeros((n_states, N + 1))
	next_states = x_m.copy()
	xs = np.array(x_s).reshape(-1, 1)  # final state
	u0 = np.array([1, 0] * N).reshape(-1, 2).T  # np.ones((N, 2)) # controls

	## start MPC
	## 设置初始点和目标点(通过这里实现强制左转弯和右转弯和绕圈)
	c_p = np.concatenate((x0, xs))
	init_control = np.concatenate((u0.T.reshape(-1, 1), next_states.T.reshape(-1, 1)))

	res = solver(x0=init_control, p=c_p, lbg=lbg, lbx=lbx, ubg=ubg, ubx=ubx)
	estimated_opt = res['x'].full()  # the feedback is in the series [u0, x0, u1, x1, ...]
	u0 = estimated_opt[:N * n_controls].reshape(N, n_controls).T  # (n_controls, N)
	x_m = estimated_opt[N * n_controls:].reshape(N + 1, n_states).T  # [n_states, N]

	# 这里返回的是几何坐标，因为用不上控制量
	return x_m[0],x_m[1]


if __name__ == '__main__':
	# 用于实时画图的线程
	t1 = Thread(target=plot_ponit)
	t1.start()

	# 先规划一次，否则某些全局变量为空，导致程序无法运行
	target_pointx,target_pointy = planning([x1,y1, 0], des[goal_num], 20)

	# 创建节点，一直监听，不断回调
	rospy.init_node("Mpc_Stanly_com",anonymous=True)
	print("thread create\n")
	mpc_stanly = mpc_stanly_com()
	mpc_stanly.listener()
	t1.join()
