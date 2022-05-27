
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

const int pinon = 2; //Boton
const int pinled = 3; //Led para rectificar el estado del boton 
int encoder = 5; //Encoder para Int
int cont = 0;
long poten; //Potenciometro para Float

int estaon = LOW;
int estado;
int estadoenc = LOW;
int encoderLast = LOW;

ros::NodeHandle nh;

std_msgs::Bool bool1_msg; //Se declara el nombre del objeto a trabajar y el tipo de variable
std_msgs::Int16 int1_msg;
std_msgs::Float32 float1_msg;

ros::Publisher int1("int1", &int1_msg); //Se coloca el nombre del topic con el que se va a trabajar en el codigo,
                                         //nombre del topic con el que se quiere trabajar en genetal con ros y el objeto topic("nodo", objeto)
ros::Publisher float1("float1", &float1_msg);
ros::Publisher bool1("bool1", &bool1_msg);

void setup() {
  
  nh.initNode(); //Inicializa el nodo
  nh.advertise(bool1); //inicializa el topic
  nh.advertise(int1);
  nh.advertise(float1);
  
  pinMode(pinon,INPUT); // Declara pines de entrada y salida
  pinMode(pinled, OUTPUT);
  pinMode(encoder,INPUT);
}

void loop() {
  
  estaon = digitalRead(pinon); //Lectura de cada sensor
  estadoenc = digitalRead(encoder);
  //Potenciometro
  poten = analogRead(A0);

 //Boton y Led indicador
  if(estaon == HIGH){
    digitalWrite(pinled, HIGH);
    estado = digitalRead(pinon);
  }
  
  if(estaon == LOW){
    digitalWrite(pinled, LOW);
    estado = digitalRead(pinon);
  }
  //Encoder
  if((encoderLast == LOW) && (estadoenc == HIGH)){
    cont++;
  }
  
  encoderLast = estadoenc;
  
  bool1_msg.data = estado; //Asigna el mensaje que llevará el objeto 
  bool1.publish( &bool1_msg ); //El topic publica el objeto
  
  int1_msg.data = cont;
  int1.publish( &int1_msg);
  
  float1_msg.data = poten;
  float1.publish( &float1_msg);
  
  nh.spinOnce();
  delay(33); //Frecuencia de 30 Hz ==> 0.033s ===> 33ms

}
