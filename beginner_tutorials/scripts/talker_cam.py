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
    cap = cv2.VideoCapture(0) # Se crea el objeto de camara

    cap.set(3, 480) # Se dimensiona el tamaño de la imagen
    cap.set(4, 320)

    _, frame = cap.read() #Leemos la imagen 
    rows, cols,_ = frame.shape # Calculamos la cantidad de filas y columnas que se encuentran en la imagen

    x_medium = 0 # Variable para saber el centro del objeto en horizontal
    y_medium = 0 # Variable para saber el centro del objeto en vertical
    centro_y = int(cols/2) # Centro de columnas
    centro_x = int(rows/2) # Centro de filas

    X_MAX = 180 # Angulo max que puede alcanzar el servo en posicion horizontal 
    X_MIN = 0 # Angulo min que puede alcanzar el servo en posicion horizontal 
    X_HOME = 90 # Angulo medio que puede alcanzar el servo en posicion horizontal 

    ## Los angulos que se definen para el movimiento vertical tienen pocos grados de libertad ya que el servo utilizado no cuenta con la suficiente potencia para subir o bajar la cámara
    Y_MAX = 105 # Angulo max que puede alcanzar el servo en posicion vertical
    Y_MIN = 75 # Angulo min que puede alcanzar el servo en posicion vertical
    Y_HOME = 90 # Angulo medio que puede alcanzar el servo en posicion vertical

    now_degree_x = X_HOME # Declaramos variables para ir variando el angulo
    now_degree_y = Y_HOME
    move_degree_x = 0 # Declaramos variables donde tendremos el valor del angulo que va dando
    move_degree_y = 0
    pub_1 = rospy.Publisher('Mov_ho', Int16, queue_size=10) # Iniciamos los topic de publishe para enviar los datos en el que el motor debe moverse tanto horizontal como vertical
    pub_2 = rospy.Publisher('Mov_ve', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True) # Iniciamos el nodo de talker (publisher)
    rate = rospy.Rate(20) # 20hz
    while True:
        ret, frame = cap.read() 
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #Transformamos el espacio de color, ya que python por defecto lo trae en BGR

        low_red = np.array([162, 155, 85]) ##Definimos los rangos de rojo a utilizar el mas alto y el mas bajo
        high_red = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv_frame, low_red, high_red) # Le indicamos que en la imagen frame se van a encontrar estos rangos
        _, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Encuentra el contorno del objeto, el primer parametro es la imagen que queremos realizar el contorno
                                                                                            # El segundo parametro es el modo de busqueda y el tercero es la aproximacion del controno 
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse = True) # Da elarea de contorno con un rectángulo

        for cnt in contours:    
            (x, y, w, h) = cv2.boundingRect(cnt) # Obtiene la informacion sobre los bordes del contorno
            #Coordenadas como esquina superior izquiera, ancho y alto

            x_medium = int((x + x +w) /2) # Hallamos la mitad del rectangulo que da el contorno
            y_medium = int((y + y +h) /2)
            break

        if x_medium < centro_x - 30: # Establecemos las condiciones para que si el medio se desplaza 30 pixeles para la derecha del centro el angulo se desplace un grado más
            move_degree_x = now_degree_x + 1
        elif x_medium > centro_x + 30: # Establecemos las condiciones para que si el medio se desplaza 30 pixeles para la izquierda del centro el angulo se desplace un grado menos
            move_degree_x = now_degree_x - 1

        if y_medium < centro_y - 30:
            move_degree_y = now_degree_y - 1
        elif y_medium > centro_y + 30:
            move_degree_y = now_degree_y + 1


        if move_degree_x > X_MIN and move_degree_x < X_MAX: # Establecemos que el angulo que estamos mandando este dentro de las condiciones propuestas al inicio
            if move_degree_y > Y_MIN and move_degree_y < Y_MAX:
                now_degree_x = move_degree_x
                now_degree_y = move_degree_y

        
        cv2.line(frame, (x_medium, 0), (x_medium, 480), (0,255,0),2) # Se dibujan las lineas en pantallas del centro del objeto
        cv2.line(frame, (0, y_medium), (640, y_medium), (0,255,0),2)

        cv2.imshow("Frame", frame) # Se muestra en pantalla el fram (video)
        key = cv2.waitKey(1) # Se establece la condicion para que se cierre la pantalla 

    
        dato_1 =  now_degree_x % rospy.get_time() # En una variable establecemos el dato que se va a enviar al arduino
        dato_2 =  now_degree_y % rospy.get_time()
        rospy.loginfo(dato_1) # Imprimimos los dos datos en pantalla
        rospy.loginfo(dato_2)
        pub_1.publish(dato_1) # Enviamos los datos al arduino 
        pub_2.publish(dato_2)
        rate.sleep()
           
        if key == ord('q'): # Si oprimimos la tecla q la pantalla de la cámara se cerrará y detendra el nodo
            break

    cap.release() # Cerramos la cámara
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass