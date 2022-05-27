# Proyecto-ROS

### 1. IROBOT_CREATE
### 2. Comunicación ROS - Arduino
### 3. Tracking camara ROS - Arduino

### Para estos trabajos se recomienda instalar Ubuntu 16.04 y ROS versión Kinetic, ya que más adelante en la instalación de demás dependencias puede generar conflictos.

# 1. IROBOT_CREATE
#### En este primer proyecto se explicará como controlar un irobot create por medio de ROS

Crear carpeta Catkin_ws y SRC
```sh
$ mkdir catkin_ws

$ cd catkin_ws

$ mkdir src

$ catkin_make
```
Al ejecutar catkin_make se crearán las carpetas build y devel
Descargamos el paquete de turtlesim y el rbx1
```sh
$ git clone https://github.com/ros/ros_tutorials.git

$ git clone https://github.com/pirobot/rbx1.git

$ catkin_make
```
Se instalan las librerias requeridas
```sh
$ sudo apt.get install Python-rosinstall

$ sudo apt-get install ros-kinetic-turtlebot

$ sudo apt-get install ros-kinetic-arbotix
```
En este punto ya se tiene todo instalado, verificamos el puerto en el que esta conectado el irobot
```sh
$ cd/dev/ls

$ sudo chmod 777 <puerto_serie>

$ roscore

$ roslaunch rbx1_bringup turtlebot_minimal_create.launch
```
Pueden aparecer varias advertencias, esto pasa mientras se establece conexión con el irobot. También, puede que genere un error en donde dice revisar la batería del irobot, sin embargo, esto se puede solucionar dejando el irobot conectado a su cargador.

Comando para mover el irobot con las teclas.
```sh
$ rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/cmd_vel
```
# 2. COMUNICACIÓN ARDUINO - ROS

#### En este proyecto se desarrollará una comunicación entre python y arduino, creando nodos propios de ros, donde arduino sera el publicador y python el suscriptor
```sh
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
```
Para tener una base se descargaran los ejemplos .py y se guardarán en scripts. El primero de ellos es el topic de talker y el segundo es listener
```sh
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py
```
Luego de esto, editamos los permisos en el archivo CMakeLists.txt que se encuentra en la carpeta beginner_tutorials. Agregamos lo siguiente

catkin_install_python(PROGRAMS scripts/talker.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

Ejecutamos
```sh
$ cd ~/catkin_ws
$ catkin_make
```

#### Creamos los archivos necesarios para tener la comunicación, se enviarán datos enteros, flotantes y booleanos desde arduinos asi que existiran los siguientes archivos:

1. Codigo de arduino donde se inicia el nodo de Serial_node. Este se encuentra en la carpeta codigos_arduino y codigo.

2. En la carpeta beginner_tutorials hay archivos llamados Nodo1, Nodo2, Nodo3, Nodo4, Nodo5, Nodo6 y talker

  Nodo 1: Recibe el dato entero de arduino y lo envia en tipo String al Nodo 4.
  
  Nodo 2: Recibe el dato flotante de arduino y lo envia en tipo String al Nodo 5.
  
  Nodo 3: Recibe el dato booleano de arduino y lo envia en tipo String al Nodo 6.
  
  talker: Recibe los datos del nodo 4, 5 y 6 y los publica en el terminal.

La explicación de los nodos estan en el código, sin embargo, los códigos en el que se detallan los codigos es el Nodo 1 y el Nodo 4.
  
Editamos los permisos en el archivo CMakeLists.txt que se encuentra en la carpeta beginner_tutorials. Agregamos lo siguiente

catkin_install_python(PROGRAMS scripts/Nodo1.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

catkin_install_python(PROGRAMS scripts/Nodo2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/Nodo3.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  
)

catkin_install_python(PROGRAMS scripts/Nodo4.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  
)

catkin_install_python(PROGRAMS scripts/Nodo5.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  
)

catkin_install_python(PROGRAMS scripts/Nodo6.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  
)

Ejecutamos
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Luego para correr los programas se ejecutan los siguientes comandos
