#!/usr/bin/env python3
import rospy
from sympy import*
import numpy as np 
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

j1=Symbol('j1')
j2=Symbol('j2')
j3=Symbol('j3')
j4=Symbol('j4')
j5=Symbol('j5')
j6=Symbol('j6')
j7=Symbol('j7')

pub = rospy.Publisher('position', Point, queue_size=10)

def dh_matrix (t,d,a,aph):

	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],[sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],[0,sin(aph), cos(aph), d],[0, 0, 0, 1]])
	return T

T01 = dh_matrix(0,0.041,0.0,0)
T12 = dh_matrix(j1,0.064,0.0,pi/2)
T23 = dh_matrix(j2,0.0,0.0,-pi/2)
T34 = dh_matrix(0,0.065,0.0,0)
T45 = dh_matrix(j3,0.185,0.0,pi/2)
T56 = dh_matrix(j4,0.0,0.0,-pi/2)
T67 = dh_matrix(0,0.121,0.0,0)
T78 = dh_matrix(j5,0.129,0.0,pi/2)
T89 = dh_matrix(j6,0.0,0.0,-pi/2)
T910 = dh_matrix(0,0.019,0.0,0)
T1011 = dh_matrix(j7,0.06,0.0,0)

T02 = T01*T12
T03 = T02*T23
T04 = T03*T34
T05 = T04*T45
T06 = T05*T56
T07 = T06*T67
T08 = T07*T78
T09 = T08*T89
T010 = T09*T910
T011 = T010*T1011

print("px= ", T011[0,3])
print("py= ", T011[1,3])
print("pz= ", T011[2,3])
print("---------------------------------")

def joint_callback(msg):
    angle1=msg.position[0]
    angle2=msg.position[1] 
    angle3=msg.position[2]
    angle4=msg.position[3]
    angle5=msg.position[4]
    angle6=msg.position[5]
    angle7=msg.position[6]
    T03n=T011.subs([(j1,angle1),
                    (j2,angle2),
                    (j3,angle3),
                    (j4,angle4),
                    (j5,angle5),
                    (j6,angle6),
                    (j7,angle7)])
    print("x= ", T03n[0,3])
    print("y= ", T03n[1,3])
    print("z= ", T03n[2,3])
    print("---------------------------------")
    pointmessaje = Point(T03n[0,3], T03n[1,3], T03n[2,3])
    pub.publish(pointmessaje)

def subscriber():
    rospy.init_node('forward', anonymous=True)
    rospy.Subscriber("joint_states", JointState, joint_callback)    
    rospy.spin()

if __name__ == '__main__':
    subscriber()

