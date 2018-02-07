#!/usr/bin/env python

import numpy as np
import random

import rospy
import os

from sklearn.svm import NuSVR

from axis_camera.msg import Axis
import geometry_msgs.msg


np.random.seed(0)


class localizeController:

    def __init__(self):
    	# Subscribers
        self.camera_info_sub = rospy.Subscriber('/state',
            Axis, self.camera_callback, queue_size = 1)


def train_svms():


	with open(os.path.join('/usr/users/promo2018/tran_cec/projet_track/src/turtle_controller/scripts/data/', 'data_pan_tilt.csv'), 'r') as f:
		lines = [l.rstrip('\n').split(',') for l in f][1:]

	pans = [float(p[0]) for p in lines]
	tilts = [float(t[1]) for t in lines]
	x_carr = [float(x[2]) for x in lines]
	y_carr = [float(y[3]) for y in lines]

	c = list(zip(pans, tilts, x_carr, y_carr))
	random.shuffle(c)
	pans, tilts, x_carr, y_carr = zip(*c)

	train_percentage = 0.8
	train_len = int(train_percentage * len(pans))

	pans_train, pans_test = pans[:train_len], pans[train_len:]
	tilts_train, tilts_test = tilts[:train_len], tilts[train_len:]
	x_carr_train, x_carr_test = x_carr[:train_len], x_carr[train_len:]
	y_carr_train, y_carr_test = y_carr[:train_len], y_carr[train_len:]

	# X
	# Train
	X_train = np.asarray([[i, j] for i, j in zip(pans_train, tilts_train)])
	y_train = np.asarray([i for i in x_carr_train])

	regr_x = NuSVR(nu=0.1, kernel='linear')
	regr_x.fit(X_train, y_train)

	print(regr_x.score(X_train, y_train))

	# Test
	X_test = np.asarray([[i, j] for i, j in zip(pans_test, tilts_test)])
	y_test = np.asarray([i for i in x_carr_test])

	print(regr_x.score(X_test, y_test))

	# Y
	# Train
	X_train = np.asarray([[i, j] for i, j in zip(pans_train, tilts_train)])
	y_train = np.asarray([i for i in y_carr_train])

	regr_y = NuSVR(nu=0.1, kernel='linear')
	regr_y.fit(X_train, y_train)

	print(regr_y.score(X_train, y_train))

	# Test
	X_test = np.asarray([[i, j] for i, j in zip(pans_test, tilts_test)])
	y_test = np.asarray([i for i in y_carr_test])

	print(regr_y.score(X_test, y_test))

	return regr_x, regr_y


def pantilt2xy(pan, tilt):

	global regr_x, regr_y

	pt = np.asarray([[pan, tilt]])
	#pt = [pan, tilt]
	x = regr_x.predict(pt)[0]
	y = regr_y.predict(pt)[0]

	print('#'*50)
	print('pan : ', pan)
	print('tilt : ', tilt)
	print('x : ', x)
	print('y : ', y)

	with open(os.path.join('pt.txt'), 'w') as f:
		f.write('{}\t{}\n'.format(x, y))

	return x, y


def camera_callback(data_cam):

	global pub_pos

	zoom = data_cam.zoom
	tilt = data_cam.tilt
	pan = data_cam.pan
	brightness = data_cam.brightness
	autofocus = data_cam.autofocus

	x, y = pantilt2xy(pan, tilt)

	msg.x = x
	msg.y = y
	msg.z = 0

	pub_pos.publish(msg)


if __name__ == '__main__':

	regr_x, regr_y = train_svms()

	rospy.init_node('localize', anonymous=True)
	msg = geometry_msgs.msg.Point()

	pub_pos = rospy.Publisher('/ArenaPosition', geometry_msgs.msg.Point, queue_size=10)
	camera_info_sub = rospy.Subscriber('/state', Axis, camera_callback, queue_size = 1)

    rospy.spin()
