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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
dato1 = None
dato2 = None
dato3 = None


def callback1(data):
    global dato1 #Se definio una variable global para cada una de las llamadas
    dato1 = data.data #Se asigno el dato que proviene de cada llamada a su respectiva variable
     

def callback2(data):
    global dato2
    dato2 = data.data    
        
def callback3(data):
    global dato3 
    dato3 = data.data 
    
    
def imprimir():
    rate = rospy.Rate(0.5) #Frecuencia de 0.5 Hz
    while not rospy.is_shutdown():
        rospy.loginfo(rospy.get_caller_id() + 'Entero: %s', dato1) ##Se imprime el dato
        rospy.loginfo(rospy.get_caller_id() + 'Flotante %s', dato2)
        rospy.loginfo(rospy.get_caller_id() + 'Booleano %s', dato3)
        rate.sleep()

def listener():
    
    rospy.init_node('listener', anonymous=False) #Se inicializa el nodo
    
    rospy.Subscriber('int_string_condicion', String, callback1) #Se recibe el topic, su tipo de variable, y la llamada respectiva
    rospy.Subscriber('float_string_condicion', String, callback2)
    rospy.Subscriber('bool_string_condicion', String, callback3)

    imprimir()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        

if __name__ == '__main__':
    
    listener()

    
   
