#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Empty
import argparse
import glob
import datetime
import time
import numpy as np
import cv2 as cv

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64

rospy.init_node('capture_node', anonymous=True)
joint_pub = rospy.Publisher('/right_arm_controller/command', JointTrajectory, queue_size=1)

steps = 0

first = True
# use iterator later
# range -1.22173 to 1.22173, steps
t_stamp = rospy.Time.now()
running_error = 0.0


def moveJoint(jointcmds, prefix='right', nbJoints=7):
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
    point.time_from_start = rospy.Duration.from_sec(5.0)
    for i in range(0, nbJoints):
        jointCmd.joint_names.append(prefix + 'joint_' + str(i + 1))
        point.positions.append(jointcmds[i])
        point.velocities.append(0)
        point.accelerations.append(0)
        point.effort.append(0)
    jointCmd.points.append(point)
    rate = rospy.Rate(100)
    count = 0
    while (count < 5):
        joint_pub.publish(jointCmd)
        count = count + 1
        rate.sleep()

def data_capture(data):
    global steps, t_stamp, running_error

    if data.header.stamp <= t_stamp:
        return
    if running_error > 0.1:
        return

    # max_angle= 1.22173
    max_angle = 1.0
    samples = 20

    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

    # img =  iimage.fromarray(frame)
    # img.show()
    frame = frame[:, :, [2, 1, 0]]
    cv.imshow("win", frame)
    cv.waitKey(1)

    joint1_ind = int(steps % samples)
    joint2_ind = int((steps // samples) % samples)
    file_name = "image_%03d_%03d.png" % (joint1_ind, joint2_ind)
    # cv.imwrite("output/" + file_name, frame)

    joint1_val = (((max_angle * 2) / samples) * (steps % samples) - max_angle) * (1 - 2 * ((steps // samples) % 2)) ++ 3.75 / 2.0
    joint2_val = ((max_angle * 2) / samples) * ((steps // samples) % samples) - max_angle

    moveJoint([1.57, joint2_val, 0.0, 0.0, 0.0, joint1_val, 1.0])
    steps += 1

    time.sleep(0.2)
    # if first:
    #     time.sleep(5)
    t_stamp = rospy.Time.now()

    return


def run(data):
    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    tstart = time.time()

    height = frame.shape[0]
    width = frame.shape[1]
    frame1 = cv.resize(frame, (576, 324))
    frame_np = frame1[:, :, [2, 1, 0]]

    # test
    img = Image()
    img.encoding = "bgr8"
    img.height = height
    img.width = width
    # img.step = (width) * sizeof(float)
    img.step = img.width * 8 * 3
    img.is_bigendian = 0
    img.data = np.asarray(frame, np.uint8).tostring()
    raw_video_pub.publish(img)
    print("in")




def state_error(data):
    # print("in data")
    global running_error
    running_error = max((data.error.positions))
    print(running_error)

index = 0
if __name__ == '__main__':
    print("init done")
    while (not rospy.is_shutdown()):
        # rospy.Subscriber(name='/panda_camera/depth/image_raw', data_class=numpy_msg(Image),
        #                  callback=data_capture,
        #                  queue_size=10)
        # rospy.Subscriber(name='/arm_controller/state', data_class=JointTrajectoryControllerState,
        #                  callback=state_error,
        #                  queue_size=10)
        index = index%10
        val =index*0.1
        moveJoint([0,val,0,-val,0,-val,0])
        index =index+1
        time.sleep(1)

        # print("--",val)
        # rospy.spin()

print datetime.datetime.now()
