//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Marcos Barranco Montilla
// Last update: 20.03.2025
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Archivo de cabeceras de cmeasurements_visita.cpp (declaramos la clase, atributos y métodos que definimos en cmeasurements_visita.cpp)




#ifndef CMEASUREMENTS_VISITA_H
#define CMEASUREMENTS_VISITA_H


// includes (dependencias) :

#include "ros/ros.h"                                    // añadimos libreria roscpp (funciones namespace ros:: )
#include <iostream>                                     // añadimos librería estándar de C++ (funciones namespace std::)
#include "std_msgs/String.h"                            // añadimos mensajes tipo std_msgs::String  ---> permite usar mensajes tipo "String" del paquete "std_msgs"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"                           // datos float64 equivale a datos tipo double en C++ (cero exacto = 0.0)

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"          // añadimos mensajes tipo [geometry_msgs::Twist] para poder utilizarlos
#include "geometry_msgs/Vector3.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"


#include <fstream>          // para manejar lectura y/o escritura de archivos
#include <vector>
#include <string>
#include <chrono>
#include <cmath>            // Para usar std::sqrt() . Funcion para calcular raices cuadradas en C++ , que acepta como argumentos tanto datos tipo float como datos tipo double.

#include "ros/package.h"    // para obtener ruta del paquete mesurements_visita ( usar ros::package::getPath() )
#include <iomanip>          // para dar formato que queremos nosotros a los valores de las variables




// defino los parámetros (#define) :






//--------------------------------------------------------------------------------------------


// creo el namespace msr --> funciones msr::

namespace msr {


class cMeasurementsVisita {

  public:
  
     //METODOS PUBLICOS :
     
     cMeasurementsVisita();          // constructor
     ~cMeasurementsVisita();         // destructor
     
     void Topics(ros::NodeHandle& n);
     
     void Run();
     
     
     
     //ATRIBUTOS PUBLICOS ("variables globales"):
     
     
     
     
  private:
  
     //METODOS PRIVADOS (aqui metemos las callback)
          
     double time_duration(const std::chrono::high_resolution_clock::time_point t0 , const std::chrono::high_resolution_clock::time_point t1);
     
     void statusVisita();                 // Funcion que nos dice en que estado se encuentra la visita de los 3 posibles : visita acaba de iniciarse, visita en progreso y visita finalizada
     void get_time_visit_total();         // Funcion para medir el tiempo total de la visita (desde que se inicia hasta que finaliza)
     void get_time_visit_robotMoving();   // Funcion para medir el tiempo en el que el robot se esta moviendo (se esta desplazando) durante la visita
     void get_distanceTraveled();         // Funcion para medir la distancia recorrida por el robot durante una visita
     void get_averageSpeed();             // Funcion para medir la velocidad media (average speed) con la que se ha movido el robot durante la visita
     
     
     void save_measurements();        // Funcion para guardar las medidas tomadas en un archivo de texto
     
     
     
     
     /* Funciones callback (privadas) */
     
     void visita_status_callback(const std_msgs::String msg1);
     void cmd_vel_out_callback(const geometry_msgs::TwistStamped msg2);
     void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped msg3);
     
     
     
     
     
     
     //ATRIBUTOS PRIVADOS:
     
     bool started = false;     
     bool finished = false;    
     
     bool visita_just_started = false;
     bool visita_in_progress = false;
     bool visita_finished = false;
     int counter_visita = 0;
         
     std::chrono::high_resolution_clock::time_point time_start_visita;      // medida tiempo entre inicializacion del nodo y un instante t_start [no unidades] 
     std::chrono::high_resolution_clock::time_point time_finish_visita;     // medida tiempo entre inicializacion del nodo y un instante t_finish [no unidades]
     double time_total_visita_seconds = 0.0;                                // tiempo total de la visita en seg = tiempo transcurrido (en segundos) entre instantes time_start_visita y time_finish_visita
     double time_total_visita_minutes = 0.0;                                // tiempo total de la visita en min = tiempo transcurrido (en minutos) entre instantes time_start_visita y time_finish_visita
     bool time_total_saved = false;
     
     double linear_vel_x = 0.0;       // velocidad lineal REAL en x de la base 
     double linear_vel_y = 0.0;       // velocidad lineal REAL en y de la base
     bool robot_isMoving = false;     // variable booleana que nos indica si el robot se esta moviendo (= true) o si esta parado (= false)
     
     std::chrono::high_resolution_clock::time_point t0;      // medida tiempo entre inicializacion del nodo y un instante t0 [no unidades]
     std::chrono::high_resolution_clock::time_point t1;      // medida tiempo entre inicializacion del nodo y un instante t1 [no unidades]
     double time_moving_measured_seconds = 0.0;
     double time_robotMoving_visita_seconds = 0.0;
     double time_robotMoving_visita_minutes = 0.0;
     bool robot_moving_at_start_visita = false;
     bool new_t0_measured = false;
     bool time_robotMoving_saved = false;
     
     double position_x = 0.0;      // aqui se va a guardar la componente x de la posicion del robot en el mapa (en metros)
     double position_y = 0.0;      // aqui se va a guardar la componente y de la posicion del robot en el mapa (en metros)
     
     double x_inicial = 0.0;
     double y_inicial = 0.0;
     double x1 = 0.0;
     double x2 = 0.0;
     double y1 = 0.0;
     double y2 = 0.0;
     double dl = 0.0;
     double distance_traveled_visita = 0.0;
     bool first_dl_calculated = false;
     bool distance_traveled_saved = false;
     
     double average_speed_visita = 0.0;
     bool average_speed_saved = false;
     
     
     
     bool measurements_saved = false;
     
     
     
     
     /* Aqui declaramos los Publishers y Subscribers (variables privadas de la clase) */
     
     // Publishers :
     
     
     
     // Subscribers :
     
     ros::Subscriber visita_status_sub;
     ros::Subscriber cmd_vel_out_sub;
     ros::Subscriber amcl_pose_sub;
     



};  // fin clase cMeasurementsVisita


} // fin namespace msr


#endif   // CMEASUREMENTS_VISITA_H

