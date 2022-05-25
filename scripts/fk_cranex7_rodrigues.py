#!/usr/bin/env python3
import pydoc
import rospy
from sympy import*
import numpy as np 
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

t1 = Symbol('t1') # θ1
t2 = Symbol('t2') # θ2
t3 = Symbol('t3') # θ3
t4 = Symbol('t4') # θ4
t5 = Symbol('t5') # θ5
t6 = Symbol('t6') # θ6
t7 = Symbol('t7') # θ7

# Helper functions
def skew(w):
	return Matrix([
		[  0  , -w[2],  w[1]],
		[ w[2],   0  , -w[0]],
		[-w[1],  w[0],   0  ]
	])

def rodriges(w, t):
	return eye(3) + sin(t) * w + (1 - cos(t)) * w * w

#########################
# Joint 0 (No rotation)
#########################
# Rotation W over Z axis
W0 = skew(Matrix([0, 0, 1]))
R0 = rodriges(W0, 0)
# Displacement (base link or ground)
P0 = Matrix([0, 0, 0.005])
# T for base link
T0 = Matrix(np.row_stack((np.column_stack((R0, P0)), [0, 0, 0, 1])))
# Frames are the same (Identity matrix)
T01 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 1 (θ 1)
#########################
# Rotation W over Z axis
W1 = skew(Matrix([0, 0, 1]))
R1 = rodriges(W1, t1)
# Displacement
P1 = Matrix([0, 0, 0.036])
# T for link 1
T1 = Matrix(np.row_stack((np.column_stack((R1, P1)), [0, 0, 0, 1])))
# Frames are the same
T12 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 2 (θ 2)
#########################
# Rotation W over -Y axis
W2 = skew(Matrix([0, -1, 0]))
R2 = rodriges(W2, t2)
# Displacement
P2 = Matrix([0, 0, 0.064])
# T for link 2
T2 = Matrix(np.row_stack((np.column_stack((R2, P2)), [0, 0, 0, 1])))
# Frames are the same
T23 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 3 (θ 3)
#########################
# Rotation W over Z axis
W3 = skew(Matrix([0, 0, 1]))
R3 = rodriges(W3, t3)
# Displacement
P3 = Matrix([0, 0, 0.065])
# T for link 3
T3 = Matrix(np.row_stack((np.column_stack((R3, P3)), [0, 0, 0, 1])))
# Frames are the same
T34 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 4 (θ 4)
#########################
# Rotation W over -Y axis
W4 = skew(Matrix([0, -1, 0]))
R4 = rodriges(W4, t4)
# Displacement
P4 = Matrix([0, 0, 0.185])
# T for link 4
T4 = Matrix(np.row_stack((np.column_stack((R4, P4)), [0, 0, 0, 1])))
# Frames are the same
T45 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 5 (θ 5)
#########################
# Rotation W over Z axis
W5 = skew(Matrix([0, 0, 1]))
R5 = rodriges(W5, t5)
# Displacement
P5 = Matrix([0, 0, 0.121])
# T for link 5
T5 = Matrix(np.row_stack((np.column_stack((R5, P5)), [0, 0, 0, 1])))
# Frames are the same
T56 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 6 (θ 6)
#########################
# Rotation W over -Y axis
W6 = skew(Matrix([0, -1, 0]))
R6 = rodriges(W6, t6)
# Displacement
P6 = Matrix([0, 0, 0.129])
# T for link 6
T6 = Matrix(np.row_stack((np.column_stack((R6, P6)), [0, 0, 0, 1])))
# Frames are the same
T67 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])

#########################
# Joint 7 (θ 7)
#########################
# Rotation W over Z axis
W7 = skew(Matrix([0, 0, 1]))
R7 = rodriges(W7, t7)
# Displacement
P7 = Matrix([0, 0, 0.019])
# T for link 7
T7 = Matrix(np.row_stack((np.column_stack((R7, P7)), [0, 0, 0, 1])))
# Frames are the same
T78 = Matrix([
	[1, 0, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, 0],
	[0, 0, 0, 1]
])


print ('\nT0 =')
pretty_print (T0)

print ('\nT1 =')
pretty_print (T1)

print ('\nT2 =')
pretty_print (T2)

print ('\nT3 =')
pretty_print (T3)

print ('\nT4 =')
pretty_print (T4)

print ('\nT5 =')
pretty_print (T5)

print ('\nT6 =')
pretty_print (T6)

print ('\nT7 =')
pretty_print (T7)


Res = T0 * T01 * T1 * T12 * T2 * T23 * T3 * T34 * T4 * T45 * T5 * T56 * T6 * T67 * T7 * T78


print ('\nT08 =')
pretty_print (Res)


print("x = ", Res[0, 3])
print("y = ", Res[1, 3])
print("z = ", Res[2, 3])

def joint_callback(msg):
	angle1 = msg.position[0]
	angle2 = msg.position[1]
	angle3 = msg.position[2]
	angle4 = msg.position[3]
	angle5 = msg.position[4]
	angle6 = msg.position[5]
	angle7 = msg.position[6]


	EndEffector = Res.subs([
					(t1, angle1),
					(t2, angle2),
					(t3, angle3),
					(t4, angle4),
					(t5, angle5),
					(t5, angle5),
					(t6, angle6),
					(t7, angle7),
					])

	x = EndEffector[0, 3]
	y = EndEffector[1, 3]
	z = EndEffector[2, 3]

	pretty_print (EndEffector)
	print("x= ", x)
	print("y= ", y)
	print("z= ", z)
	print("---------------------------------")

def subscriber():
	rospy.init_node('rodrigues', anonymous=True)
	rospy.Subscriber("joint_states", JointState, joint_callback)
	rospy.spin()

if __name__ == '__main__':
	subscriber()

