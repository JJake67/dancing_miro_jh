#!/usr/bin/python3
import rospy
import time
import miro2 as miro
import math
from numbers import Number

A = None
t = None
B = None

rospy.init_node("dance", anonymous=True)
# connect to robot
robot = miro.lib.RobotInterface(
				None,
				robot_name="miro",
				control_period=0.1,
				flags=0,
				command='frame=360w@5',
				mirocode_mode=False,
				use_pose_control=False
				)
#### robot is now connected ####

A = 0
t = 0
if False:
	while True:
		A = 37.5 + math.sin(t / 180.0 * math.pi) * 22.5
		t = (t if isinstance(t, Number) else 0) + 5
		robot.neck_angle(miro.constants.JOINT_LIFT, A)
else:
	while False:
		A = 37.5 + math.sin(t / 180.0 * math.pi) * 22.5
		B = math.sin((2 * t) / 180.0 * math.pi) * 60
		t = (t if isinstance(t, Number) else 0) + 5
		robot.neck_angle(miro.constants.JOINT_LIFT, A)
		robot.neck_angle(miro.constants.JOINT_YAW, B)
	while True:
		# def shine():
		# 	print("MiRo turning on LEDs")
		# 	color = 0xFF00FF00
		# 	i = 0
		# 	# while rospy.Time.now() < t0 + self.ACTION_DURATION:
		# 	ic = int(np.mod(i, 6))
		# 	ip = int(np.mod(i + 1, 6))
		# 	illum.data[ic] = color
		# 	illum.data[ip] = 0x00000000
		# 	pub_illum.publish(self.illum)
		# 	# i += self.TICK
		# 	# self.illum.data[ic] = 0x00000000
		# 	# self.pub_illum.publish(self.illum)
		s = 5
		robot.set_vel(lin_vel=None, ang_vel=s/math.pi)
		robot.set_illum(miro.constants.ILLUM_LF, [255, 0, 0], 255, timeout=1)
		#LEFT FRONT
		# shine()
		rospy.sleep(1)
		robot.set_illum(miro.constants.ILLUM_LM, [0, 255, 0], 255, timeout=1)

		robot.set_vel(lin_vel=None, ang_vel=-s/math.pi)
		robot.set_illum(miro.constants.ILLUM_LR, [0, 0, 255], 255, timeout=1)

		rospy.sleep(1)
		robot.set_illum(miro.constants.ILLUM_RR, [0, 0, 255], 255, timeout=1)

		robot.set_vel(lin_vel=None, ang_vel=s/math.pi)
		robot.set_illum(miro.constants.ILLUM_RM, [0, 255, 0], 255, timeout=1)
		
		rospy.sleep(1)
		robot.set_vel(lin_vel=None, ang_vel=-s/math.pi)
		robot.set_illum(miro.constants.ILLUM_RF, [255, 0, 0], 255, timeout=1)

		#robot.turn_speed(-60)
		# set_illum(self, leds, rgb, brightness=255, timeout=0):
		# robot.control_led(miro.constants.ILLUM_ALL, [255, 0, 0], 255)
		robot.set_cos(4, 0.0, timeout=0)
		robot.set_cos(5, 0.0, timeout=0)
		rospy.sleep(3)
		robot.set_cos(4, 1.0, timeout=0)
		robot.set_cos(5, 1.0, timeout=0)
#launhing the visualising before the daning and creates nodes in the launcing file / main rythem = main bpm
		#robot.set_joints(joint.JOINT_EAR_L, ang, timeout=0 miro.constants.JOINT_EAR_L, 0.0)
		#self.pub_kin_joints.publish(self.msg_kin_joints.get())
		#kin_joints = self.msg_kin_joints.get()
		#if joint == miro.constants.JOINT_LIFT:
		#	kin_joints.position[joint] = self.clip_with_warn(ang, miro.constants.LIFT_RAD_MIN, miro.constants.LIFT_RAD_MAX,
		#		"set_kin", "lift angle (radians)")
		robot.joint_position(miro.constants.JOINT_EAR_R, 0.0)
		robot.sleep(0.5)
		robot.turn_speed(-60)
		robot.control_led(miro.constants.ILLUM_ALL, [51, 102, 255], 255)
		robot.joint_position(miro.constants.JOINT_EAR_L, 1.0)
		robot.joint_position(miro.constants.JOINT_EAR_R, 1.0)
		robot.sleep(0.5)
	robot.turn_speed(0.0)


'''tilt, lift, yaw, pitch = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
freq, volume, duration = range(3)
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)'''