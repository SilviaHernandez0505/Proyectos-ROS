#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import Int16
import cv2
import numpy as np


def talker():
    cap = cv2.VideoCapture(0)

    cap.set(3, 480)
    cap.set(4, 320)

    _, frame = cap.read()
    rows, cols,_ = frame.shape

    x_medium = 0 #int(cols/2)
    y_medium = 0 #
    centro_x = int(cols/2)
    centro_y = int(rows/2)

    X_MAX = 180
    X_MIN = 0
    X_HOME = 90

    Y_MAX = 105
    Y_MIN = 75
    Y_HOME = 90

    now_degree_x = X_HOME
    now_degree_y = Y_HOME
    move_degree_x = 0
    move_degree_y = 0
    pub_1 = rospy.Publisher('Mov_ho', Int16, queue_size=10)
    pub_2 = rospy.Publisher('Mov_ve', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while True:
        ret, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        low_red = np.array([162, 155, 85]) ##Definimos el rojo a utilizar el mas alto y el mas bajo
        high_red = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv_frame, low_red, high_red) #Utilizamos mascara
        _, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse = True)

        for cnt in contours:    
            (x, y, w, h) = cv2.boundingRect(cnt)

            x_medium = int((x + x +w) /2)
            y_medium = int((y + y +h) /2)
            break

        if x_medium < centro_x - 30:
            move_degree_x = now_degree_x + 1
        elif x_medium > centro_x + 30:
            move_degree_x = now_degree_x - 1

        if y_medium < centro_y - 30:
            move_degree_y = now_degree_y - 1
        elif y_medium > centro_y + 30:
            move_degree_y = now_degree_y + 1


        if move_degree_x > X_MIN and move_degree_x < X_MAX:
            if move_degree_y > Y_MIN and move_degree_y < Y_MAX:
                now_degree_x = move_degree_x
                now_degree_y = move_degree_y

        #print("posicion x \n", move_degree_x)
        #print("posicion y \n", move_degree_y)q
        cv2.line(frame, (x_medium, 0), (x_medium, 480), (0,255,0),2)
        cv2.line(frame, (0, y_medium), (640, y_medium), (0,255,0),2)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)

    
        dato_1 =  now_degree_x % rospy.get_time()
        dato_2 =  now_degree_y % rospy.get_time()
        rospy.loginfo(dato_1)
        rospy.loginfo(dato_2)
        pub_1.publish(dato_1)
        pub_2.publish(dato_2)
        rate.sleep()
           
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass