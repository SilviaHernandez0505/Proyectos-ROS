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
from std_msgs.msg import Int16, String

dato1 = None

def callback(data):
    global dato1 ##Iniciamos una variable global para darle a este el valor del dato que esta llegando
    dato1 = data.data #Se asigna el dato que llega en la funcion de llamada


if __name__ == '__main__':
    #Llega el topic como Int16, se realiza las condiciones y se envia como String
    rospy.init_node('Nodo1', anonymous=False) #Se inicia el nodo
    rospy.Subscriber('int1', Int16, callback) #La función de subscriber indica de que nodo estamos escuchando, en este caso el nombre del nodo que asignamos en arduino
    pub = rospy.Publisher('int_String_Nodo1_to_Nodo4', String, queue_size=10) #Funcion de publicar, es importante indicar el tipo de variable con la que vamos a trabajar
    rate = rospy.Rate(10) #Frecuencia de 10 Hz
    while not rospy.is_shutdown():  
        if dato1 >= 0 and dato1 <10: #Hacemos las respectivas condiciones
            dato2 = "frio"
        
        elif dato1 >= 10 and dato1 < 20:
            dato2 = "tibio" 
        else:
            dato2 = "caliente"
        
       
        pub.publish(dato2) #Se publica el dato2, el cual es donde se encuentra la variable tipo string
        rate.sleep()     
        