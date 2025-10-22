//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Marcos Barranco Montilla
// Last update: 11.04.2025
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Archivo de cabeceras de c_object_detection.cpp (declaramos la clase, atributos y métodos que definimos en c_object_detection.cpp)




#ifndef C_OBJECT_DETECTION
#define C_OBJECT_DETECTION


// includes (dependencias) :

#include "ros/ros.h"                                    // añadimos libreria roscpp (funciones namespace ros:: )
#include <iostream>                                     // añadimos librería estándar de C++ (funciones namespace std::)
#include "std_msgs/String.h"                            // añadimos mensajes tipo std_msgs::String  ---> permite usar mensajes tipo "String" del paquete "std_msgs"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"                           // datos float64 equivale a datos tipo double en C++ (cero exacto = 0.0)

#include "pal_interaction_msgs/TtsActionGoal.h"   // añadimos mensajes tipo pal_interaction_msgs::TtsActionGoal --> permite usar mensajes tipo "TtsActionGoal" del paquete "pal_interaction_msgs"

#include "actionlib_msgs/GoalStatusArray.h"    // para topic /tts/status 
#include "actionlib_msgs/GoalStatus.h"         

#include "yolov8_ros_msgs/BoundingBoxes.h"       // añadimos mensajes tipo yolov8_ros_msgs::BoundingBoxes
#include "yolov8_ros_msgs/BoundingBox.h" 

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"          // añadimos mensajes tipo [geometry_msgs::Twist] para poder utilizarlos
#include "geometry_msgs/Vector3.h"

#include <vector>
#include <string>

#include <chrono>
#include <ctime>
#include <iomanip>




// defino los Parametros (# define) :

#define PROB_MIN_PERSON 0.50     // Umbral de confianza establecido para los objetos de class = person detectados

#define HEIGHT_MIN_PERSON_DETECTED 200    // altura del cuadro de deteccion (ymax - ymin) a partir de la cual se consider persona detectada : >= 200 px

#define XMAX_GREET_PERSON 240         // <= 240 px
#define XMIN_GREET_PERSON 400         // >= 400 px
#define HEIGHT_MIN_GREET_PERSON 370   // >= 370 px

#define HEIGHT_SAME_PERSON 300  // >= 300 px




//--------------------------------------------------------------------------------------------


// creo el namespace obj_det --> funciones obj_det::

namespace obj_det {


class cObjectDetection {

  public:
  
     //METODOS PUBLICOS :
     
     cObjectDetection();          // constructor
     ~cObjectDetection();         // destructor
     
     void Topics(ros::NodeHandle& n);
     
     void getTimePeriod();  // Funcion que obtiene la hora del reloj del PC y determina el periodo del día en el que nos encontramos
     
     void Run(const std::string language_chosen);
         
     
     
     //ATRIBUTOS PUBLICOS ("variables globales"):
     
     std::string idioma_recibido_visita = "";   // aqui se guarda el idioma elegido en el nodo "visita" al iniciarlo ("es" o "en")
     
     
     
     
  private:
  
     //METODOS PRIVADOS (aqui metemos las callback)
          
     void play_dialog(const std::string& dialog , const std::string& idioma_dialog);   // funcion para reproducir un texto
     
     void talk_person(const std::string idioma);  // Funcion para hablar cuando se detecta una persona que cumple las condiciones establecidas.
     
     void greetPerson(const std::string language);    // Funcion para saludar a una persona detectada con la camara mediante YOLO mientras robot esta en movimiento
     
     
     
     
     
     /* Funciones callback (privadas) */
     
     void language_callback(const std_msgs::String msg1);
     void tts_status_callback(const actionlib_msgs::GoalStatusArray msg2);
     void cmd_vel_out_callback(const geometry_msgs::TwistStamped msg3);
     
     void bboxes_callback(const yolov8_ros_msgs::BoundingBoxes msg4);

     
     
     
     
     
     
     //ATRIBUTOS PRIVADOS:
     
     std::string time_period = "";    // guarda el periodo del dia : "morning","afternoon" o "evening"
     
     bool text_tts_playing = false;    // indica si hay texto reproduciendose (true) o ya ha terminado de reproducirse (false)
     
     double linear_vel_x = 0.0;       // velocidad lineal REAL en x de la base 
     double linear_vel_y = 0.0;       // velocidad lineal REAL en y de la base
     bool robot_isMoving = false;     // variable booleana que nos indica si el robot se esta moviendo (= true) o si esta parado (= false)
     
     yolov8_ros_msgs::BoundingBoxes bb_msg;
     int number_objects_detected = 0;        // numero de objetos detectados 
     int number_persons_detected = 0;        // numero personas detectadas (aplicando las condiciones establecidas)
     
     bool info_active_displayed = false;
     bool info1_disabled_displayed = false;
     bool info2_disabled_displayed = false;
     bool person_detected_to_greet = false;   // nos indica si el robot ha detectado a una persona a la que debe saludar 
     bool greeting_done = false;              // nos indica si el robot ya ha saludado a la persona detectada o no
     int counter_person_greeted_out = 0;      // cuenta numero de veces seguidas que persona ya saludada no se detecta en la imagen
       
     
     
     
          
     /* Aqui declaramos los Publishers y Subscribers (variables privadas de la clase) */
     
     // Publishers :
     
     ros::Publisher tts_pub;                 //declaramos variable tts_pub como objeto tipo ros::Publisher
     

          
     // Subscribers :
     
     ros::Subscriber language_sub;
     ros::Subscriber tts_status_sub;
     ros::Subscriber cmd_vel_out_sub;
     ros::Subscriber bboxes_sub;

     



};  // fin clase cObjectDetection


}  // fin namespace obj_det


#endif   // C_OBJECT_DETECTION

