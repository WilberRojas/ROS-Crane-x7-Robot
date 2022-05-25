#!/usr/bin/env python3
import rospy
import time
from sympy import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point 
from random import random
j1=Symbol('j1')
j2=Symbol('j2')
j3=Symbol('j3')
j4=Symbol('j4')
j5=Symbol('j5')
j6=Symbol('j6')
j7=Symbol('j7')

header = Header()
joint_msg = JointState()
seq = 0
header.frame_id = ''
joint_msg.name = ['crane_x7_shoulder_fixed_part_pan_joint',
				'crane_x7_shoulder_revolute_part_tilt_joint', 
				'crane_x7_upper_arm_revolute_part_twist_joint',
				'crane_x7_upper_arm_revolute_part_rotate_joint',
				'crane_x7_lower_arm_fixed_part_joint',
				'crane_x7_lower_arm_revolute_part_joint',
				'crane_x7_wrist_joint']
angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
alpha=0.3 #learning rate
iterations = 100
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

#con las ecuaciones de DH:
px = -0.079*(((-sin(j1)*sin(j3) + cos(j1)*cos(j2)*cos(j3))*cos(j4) - sin(j2)*sin(j4)*cos(j1))*cos(j5) + (-sin(j1)*cos(j3) - sin(j3)*cos(j1)*cos(j2))*sin(j5))*sin(j6) + 0.079*(-(-sin(j1)*sin(j3) + cos(j1)*cos(j2)*cos(j3))*sin(j4) - sin(j2)*cos(j1)*cos(j4))*cos(j6) - 0.25*(-sin(j1)*sin(j3) + cos(j1)*cos(j2)*cos(j3))*sin(j4) - 0.25*sin(j2)*cos(j1)*cos(j4) - 0.25*sin(j2)*cos(j1)
py = -0.079*(((sin(j1)*cos(j2)*cos(j3) + sin(j3)*cos(j1))*cos(j4) - sin(j1)*sin(j2)*sin(j4))*cos(j5) + (-sin(j1)*sin(j3)*cos(j2) + cos(j1)*cos(j3))*sin(j5))*sin(j6) + 0.079*(-(sin(j1)*cos(j2)*cos(j3) + sin(j3)*cos(j1))*sin(j4) - sin(j1)*sin(j2)*cos(j4))*cos(j6) - 0.25*(sin(j1)*cos(j2)*cos(j3) + sin(j3)*cos(j1))*sin(j4) - 0.25*sin(j1)*sin(j2)*cos(j4) - 0.25*sin(j1)*sin(j2)
pz = -0.079*((1.0*sin(j2)*cos(j3)*cos(j4) + 1.0*sin(j4)*cos(j2))*cos(j5) - 1.0*sin(j2)*sin(j3)*sin(j5))*sin(j6) + 0.079*(-1.0*sin(j2)*sin(j4)*cos(j3) + 1.0*cos(j2)*cos(j4))*cos(j6) - 0.25*sin(j2)*sin(j4)*cos(j3) + 0.25*cos(j2)*cos(j4) + 0.25*cos(j2) + 0.105

#con las ecuaciones de Rodrigues:
'''
px = 
py = 
pz = 
'''

J=Matrix([[diff(px,j1),diff(px,j2),diff(px,j3),diff(px,j4),diff(px,j5),diff(px,j6),diff(px,j7)],
		[diff(py,j1),diff(py,j2),diff(py,j3),diff(py,j4),diff(py,j5),diff(py,j6),diff(py,j7)],
		[diff(pz,j1),diff(pz,j2),diff(pz,j3),diff(pz,j4),diff(pz,j5),diff(pz,j6),diff(pz,j7)]])

def callback(data): #callback function	
	target=Matrix([data.x,data.y,data.z])
	qi = Matrix([0.001,0.001,0.001,0.001,0.001,0.001,0.001])#cambiar n joint
	for i in range(0,iterations):				
		cp = Matrix([[px.subs([(j1,qi[0]),(j2,qi[1]),(j3,qi[2]),(j4,qi[3]),(j5,qi[4]),(j6,qi[5]),(j7,qi[6])]),
							py.subs([(j1,qi[0]),(j2,qi[1]),(j3,qi[2]),(j4,qi[3]),(j5,qi[4]),(j6,qi[5]),(j7,qi[6])]),
							pz.subs([(j1,qi[0]),(j2,qi[1]),(j3,qi[2]),(j4,qi[3]),(j5,qi[4]),(j6,qi[5]),(j7,qi[6])])]])#cambiar n joint
		
		cp2= Matrix([[cp[0]], [cp[1]], [cp[2]]])
		e = target - cp2
		Jsubs=J.subs([(j1,qi[0]),
					(j2,qi[1]),
					(j3,qi[2]),
					(j4,qi[3]),
					(j5,qi[4]),
					(j6,qi[5]),
					(j7,qi[6])])
		
		Jinv=Jsubs.H*(Jsubs*Jsubs.H)**-1
		dt=Jinv*e
		qi=qi+alpha*dt
		#print (cp)
		global seq
		header.seq = seq
		header.stamp = rospy.Time.now()
		joint_msg.header = header
		joint_msg.position = angles
		joint_msg.position = [qi[0],
							qi[1],
							qi[2],
							qi[3],
							qi[4],
							qi[5],
							qi[6]]
		pub.publish(joint_msg)
		seq += 1
		time.sleep(0.5)

def subscriber():
    rospy.init_node('inverse', anonymous=True)
    rospy.Subscriber("position", Point, callback) 
    rospy.spin()

if __name__ == '__main__':
    subscriber()

