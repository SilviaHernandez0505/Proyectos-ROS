# Proyecto-ROS
# IROBOT_CREATE
### Para este trabajo se recomienda instalar Ubuntu 16.04 y ROS versión Kinetic, ya que más adelante en la instalación de demás dependencias puede generar conflictos.

### Crear carpeta Catkin_ws y SRC
```sh
$ mkdir catkin_ws

$ cd catkin_ws

$ mkdir src

$ catkin_make
```
### Al ejecutar catkin_make se crearán las carpetas build y devel
### Descargamos el paquete de turtlesim y el rbx1
```sh
$ git clone https://github.com/ros/ros_tutorials.git

$ git clone https://github.com/pirobot/rbx1.git

$ catkin_make
```
### Se instalan las librerias requeridas
```sh
$ sudo apt.get install Python-rosinstall

$ sudo apt-get install ros-kinetic-turtlebot

$ sudo apt-get install ros-kinetic-arbotix
```
### En este punto ya se tiene todo instalado, verificamos el puerto en el que esta conectado el irobot
```sh
$ cd/dev/ls

$ sudo chmod 777 <puerto_serie>

$ roscore

$ roslaunch rbx1_bringup turtlebot_minimal_create.launch
```
### Pueden aparecer varias advertencias, esto pasa mientras se establece conexión con el irobot. También, puede que genere un error en donde dice revisar la batería del irobot, sin embargo, esto se puede solucionar dejando el irobot conectado a su cargador.

### Comando para mover el irobot con las teclas.
```sh
$ rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/cmd_vel
```
# COMUNICACIÓN ARDUINO - ROS
