//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Marcos Barranco Montilla
// Last update: 20.03.2025
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// En este archivo se definen los metodos o funciones declarados en cmeasurements_visita.h , y que son los que se van a usar en measurements_visita.cpp


// includes (dependencias) :

#include "cmeasurements_visita.h"





// creo el namespace msr --> funciones msr::

namespace msr {


// Constructor (inicializacion de atributos) :

cMeasurementsVisita::cMeasurementsVisita() {

   started = false;
   finished = false;
   
   visita_just_started = false;
   visita_in_progress = false;
   visita_finished = false;
   counter_visita = 0;
   
   time_total_visita_seconds = 0.0;
   time_total_visita_minutes = 0.0;
   time_total_saved = false;
   
   linear_vel_x = 0.0;
   linear_vel_y = 0.0;
   robot_isMoving = false;
   
   time_moving_measured_seconds = 0.0;
   time_robotMoving_visita_seconds = 0.0;
   time_robotMoving_visita_minutes = 0.0;
   robot_moving_at_start_visita = false;
   new_t0_measured = false;
   time_robotMoving_saved = false;
   
   position_x = 0.0;
   position_y = 0.0;
   
   x_inicial = 0.0;
   y_inicial = 0.0;
   x1 = 0.0;
   x2 = 0.0;
   y1 = 0.0;
   y2 = 0.0;  
   dl = 0.0;
   distance_traveled_visita = 0.0;
   first_dl_calculated = false;
   distance_traveled_saved = false;
   
   average_speed_visita = 0.0;
   average_speed_saved = false;
   
   
   
   measurements_saved = false;
}




// Destructor :

cMeasurementsVisita::~cMeasurementsVisita(){

   std::cout << "cMeasurementsVisita::~cMeasurementsVisita(): shutting down ROS" << std::endl;
   usleep(100000);
   
   if (ros::isStarted()) {
	  
	  ros::shutdown();
	  ros::waitForShutdown();
   }
   
   usleep(100000);
   std::cout << " ... done " << std::endl;
}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Funciones callback para los topics suscriptores del nodo :   (se definen antes que los subscribers)


// 1) /visita/status :   ((( recibe mensaje [std_msgs/String] del topic /visita/status y lo guarda en variable local msg1)))

/* 
Esta funcion callback nos va a decir cuando comienza y finaliza la visita (se publica en /visita/status un unico mensaje cuando empieza la visita y otro cuando finaliza)

msg.data = "started" --> comienza la visita (se publica el primer destino)
msg.data = "finished"  --> finaliza la vista (se reproduce el mensaje de despedida y ha pasado el tiempo programado de espera para el mensaje)
*/

void cMeasurementsVisita::visita_status_callback(const std_msgs::String msg1){

   if (msg1.data == "started") {
   
      started = true;     // visita ha comenzado
      finished = false;
   
   }
   
   
   else if (msg1.data == "finished") {
   
      started = false;     
      finished = true;    // visita ha finalizado
   
   }
 
// usamos estructura if - else if para que cada vez que se ejecute esta funcion callback solo se entre en uno de los 2 bloques. 

}




// 2) /mobile_base_controller/cmd_vel_out :   ((( recibe mensaje [geometry_msgs/TwistStamped] del topic /mobile_base_controller/cmd_vel_out y lo guarda en variable local msg2)))

// En este topic se publican las velocidades REALES que controlan el movimiento del robot, una vez ajustadas por el local planner  (velocidades que la base realmente está usando para moverse)


/* En esta funcion callback se va a determinar si el robot esta parado o esta en movimiento en funcion de las velocidades indicadas en el mensaje que se recibe

   robot_isMoving = false  --> robot esta parado (Vx = Vy = 0.0)
   robot_isMoving = true   --> robot se esta moviendo (al menos una de sus velocidades lineales es != 0.0) 
   
   
   Hemos comprobado que el robot solo se mueve usando Vx (velocidad lineal en x) y Wz (velocidad angular en z) . El resto de las velocidades son siempre = 0.0.
   
   Los valores de las velocidades que se publican en el topic /mobile_base_controller/cmd_vel_out son valores tipo float64 , que en C++ equivale a valores tipo double , y su cero exacto es = 0.0.
   Por tanto , cuando las velocidades se anulan se publica en el topic un valor = 0.0 (comprobado con rostopic echo)
   
*/

void cMeasurementsVisita::cmd_vel_out_callback(const geometry_msgs::TwistStamped msg2){

   // msg2.twist.linear.x y msg2.twist.linear.y son valores tipo float64, que en C++ equivalen a valores tipo double --> Su cero exacto se escribe = 0.0 (es lo que tengo que poner en los bloques if)

   linear_vel_x = msg2.twist.linear.x;    // Vx (lineal) de la base  -->  (dato tipo float64 , que en C++ equivale a un dato tipo double)
   linear_vel_y = msg2.twist.linear.y;    // Vy (lineal) de la base
   
   
   if (linear_vel_x == 0.0 && linear_vel_y == 0.0) {
     
     robot_isMoving = false;   // robot parado
     
   }
   
    
   else {      // si (linear_vel_x != 0.0 || linear_vel_y != 0.0)
      
      robot_isMoving = true;  // robot en movimiento
  
   }

}




// 3) /amcl_pose :   ((( recibe mensaje [geometry_msgs/PoseWithCovarianceStamped] del topic /amcl_pose y lo guarda en variable local msg3)))

/* En el topic /acml_pose se publica un mensaje con la POSICION (en Metros) y ORIENTACION del robot en el mapa en tiempo real (respecto al Fixed Frame o Sistema Fijo del mapa) , y sus COVARIANZAS 
   (incertidumbres de la posicion y orientacion estimadas) */

void cMeasurementsVisita::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped msg3){

   // msg3.pose.pose.position.x y msg3.pose.pose.position.y son valores tipo float64, que en C++ equivalen a valores tipo double (Su cero exacto se escribe = 0.0).
   
   
   position_x = msg3.pose.pose.position.x;   // componente x en este instante de la posicion del robot en el mapa (en metros)
   
   position_y = msg3.pose.pose.position.y;   // componente y en este instante de la posicion del robot en el mapa (en metros)

}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Funcion (metodo) que define los Publishers y Subscribers del nodo measurements_visita (en esta funcion defino las variables Publisher y Subscriber que he declarado como atributos de la clase):     

void cMeasurementsVisita::Topics(ros::NodeHandle& n){

   //Publishers:
   
   
   
   
   
   //Subscribers:
   
   visita_status_sub = n.subscribe("/visita/status", 1, &cMeasurementsVisita::visita_status_callback, this);              // se suscribe a topic /visita/status [std_msgs::String]
   
   cmd_vel_out_sub = n.subscribe("/mobile_base_controller/cmd_vel_out", 1, &cMeasurementsVisita::cmd_vel_out_callback, this);   // se suscribe a topic /mobile_base_controller/cmd_vel_out
   
   amcl_pose_sub = n.subscribe("/amcl_pose", 1, &cMeasurementsVisita::amcl_pose_callback, this);       // se suscribe al topic /acml_pose [geometry_msgs::PoseWithCovarianceStamped]


}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++





// Funcion que devuelve el tiempo transcurrido EN SEGUNDOS entre 2 medidas de tiempo t0 y t1 :

/* t0 y t1 son el tiempo medido entre el momento que se inicia del programa (se inicia del nodo) y 2 puntos distintos del código.
   Las medidas de tiempo se realizan mediante el metodo std::chrono::high_resolution_clock::now() 
   
   t0 se mide antes que t1 , por lo que t1 > t0    */  

double cMeasurementsVisita::time_duration(const std::chrono::high_resolution_clock::time_point t0 , const std::chrono::high_resolution_clock::time_point t1){
   
   std::chrono::duration<double> Duration = t1 - t0;     // calculamos la duracion o tiempo transcurrido entre t0 y t1 (se guarda en objeto Duration)
   
   return Duration.count();                              // Duration.count() = tiempo transcurrido en SEGUNDOS (valor tipo "double")
                                                        
}





// Funcion que determina en que estado se encuentra la visita de los 3 posibles : visita acaba de iniciarse, visita en progreso y visita finalizada

void cMeasurementsVisita::statusVisita(){

   
   // visita_just_started = true --> visita acaba de iniciar y esta es la primera vez que se ejecuta la funcion statusVisita() desde que se inicia la visita (started == true + counter_visita = 0)
   
   if (started && counter_visita == 0) {
   
      visita_just_started = true;   // visita acaba de iniciarse
      visita_in_progress = false;
      visita_finished = false;
      
      counter_visita = 1;
   }
   
   
   // visita_in_progress = true --> visita iniciada y funcion statusVisita() se ha ejecutado mas de una vez desde que se ha iniciado la visita (started == true + counter_visita = 1)
   
   else if (started && counter_visita == 1) {
   
      visita_just_started = false;
      visita_in_progress = true;     // visita en progreso
      visita_finished = false;
   }
   
   
   // visita_finished = true --> visita ha finalizado (finished == true)
   
   else if (finished) {
   
      visita_just_started = false;
      visita_in_progress = false;
      visita_finished = true;        // visita finalizada
      
      counter_visita = 0;
   }


// usamos estructura if - else if para que cada vez que se ejecute la funcion solo se entre en uno de los 3 bloques y se ignoren o salten los otros 2. 
//Esta funcion se va a ejecutar siempre la primera dentro de la Funcion Run() para determinar el estado de la visita , por lo que se va a ejecutar cada vez que se repita el loop (cada 0.1 segundos)

}





// Funcion para medir el tiempo total de la visita (desde que se inicia hasta que finaliza)

void cMeasurementsVisita::get_time_visit_total(){

   // Visita acaba de iniciarse (visita_just_started = true) :
   
   if (visita_just_started){    // == true
      
      /* En este bloque if solo se va a entrar una unica vez por visita : cuando la visita acaba de iniciarse y se ha ejecutado la funcion statusVisita() por primera vez desde el inicio de la visita 
         (started == true && counter_visita == 0). Despues de eso ya no se va a volver a entrar nunca mas hasta que no se inicie una nueva visita */
   
      time_start_visita = std::chrono::high_resolution_clock::now();      // mido y guardo en este punto (justo cuando se inicia la visita) el tiempo transcurrido desde el inicio del nodo
      
      time_total_saved = false;
   
   }
   
   
   // visita ha finalizado (visita_finished = true)
   
   else if (visita_finished){   // == true
   
      if (!time_total_saved){  // == false
      
         time_finish_visita = std::chrono::high_resolution_clock::now();    // mido de nuevo y guardo en este punto (justo finaliza la visita) el tiempo transcurrido desde el inicio del nodo
      
      
         // calculo tiempo transcurrido (en segundos) entre time_start_visita y time_finish_visita = tiempo total de la visita, y lo guardo en time_total_visita_seconds
      
         time_total_visita_seconds = time_duration(time_start_visita , time_finish_visita); 
      
      
         time_total_visita_minutes = time_total_visita_seconds / 60.0;   // paso el tiempo total de la visita medido en segundos a minutos (divido entre 60.0)
         
         
         ROS_INFO("Tiempo total de la visita ha sido medido (en minutos)");
         
         time_total_saved = true;   
         
         /* En este bloque if se entrara una sola vez por visita (cuando time_total_saved == false) y los datos se guardaran una sola vez nada mas terminar la visita. 
            Con esto evitamos que mientras visita_finished = true (visita ha finalizado y no se inicia una nueva) se sobrescriban las variables que contienen 
            los valores finales de los datos*/
      
      }
   
   }

}





// Funcion para medir el tiempo en el que el robot se esta moviendo (se esta desplazando) durante la visita

void cMeasurementsVisita::get_time_visit_robotMoving(){

   // Visita acaba de iniciarse (visita_just_started = true) :
   
   if (visita_just_started){    // en este bloque if solo se entra 1 unica vez por visita (cuando se ejecuta statusVisita() la primera vez tras iniciar la visita).
   
      time_robotMoving_saved = false;
      
      time_robotMoving_visita_seconds = 0.0;    // antes de hacer nada, reiniciamos a 0.0 el tiempo medido en segundos cuando empieza una nueva visita
      
      
      if (robot_isMoving){  // == true (robot moviendose)
      
         robot_moving_at_start_visita = true;   
        
      }
   
   }
   

   
   // visita en progreso (visita_in_progress = true)
   
   else if (visita_in_progress){
   
      if (robot_moving_at_start_visita){  // == true
         
         t1 = std::chrono::high_resolution_clock::now();  // mido tiempo transcurrido desde el inicio del nodo hasta este punto y lo guardo en t1
         
         time_moving_measured_seconds = time_duration(time_start_visita , t1);   // calculo tiempo transcurrido (en segundos) entre time_start_visita y t1
         
         time_robotMoving_visita_seconds = time_robotMoving_visita_seconds + time_moving_measured_seconds;   // sumo tiempo medido en segundos al total 
         
         
         robot_moving_at_start_visita = false; 
          
      }
   
      
      // robot moviendose (robot_isMoving = true) 
      
      if (robot_isMoving && !new_t0_measured){
      
      t0 = std::chrono::high_resolution_clock::now();  // mido tiempo transcurrido desde el inicio del nodo hasta este punto y lo guardo en t0
      
      new_t0_measured = true;
      
      }
      
      
      // robot parado (robot_isMoving = false) 
      
      else if (!robot_isMoving && new_t0_measured){
         
         t1 = std::chrono::high_resolution_clock::now();   // mido tiempo transcurrido desde el inicio del nodo hasta este punto y lo guardo en t1
         
         time_moving_measured_seconds = time_duration(t0 , t1);     // calculo tiempo transcurrido (en segundos) entre t0 y t1
         
         time_robotMoving_visita_seconds = time_robotMoving_visita_seconds + time_moving_measured_seconds;    // sumo tiempo medido en segundos al total
         
         
         new_t0_measured = false;   
      }
      
   }
   
   
   
   // visita ha finalizado (visita_finished = true)
   
   else if (visita_finished){  
   
      if (!time_robotMoving_saved){   // == false
         
         time_robotMoving_visita_minutes = time_robotMoving_visita_seconds / 60.0;    // paso el tiempo medido en segundos a minutos (divido entre 60.0)  
         
         
         ROS_INFO("Tiempo en el que el robot esta desplazandose durante la visita ha sido medido (en minutos)");
         
         time_robotMoving_saved = true; 
      
         /* En este bloque if se entrara una sola vez por visita (cuando time_robotMoving_saved == false) y los datos se guardaran una sola vez nada mas terminar la visita. 
            Con esto evitamos que mientras visita_finished = true (visita ha finalizado y no se inicia una nueva) se sobrescriban las variables que contienen 
            los valores finales de los datos*/
      
      }
      
   }
   

}





// Funcion para medir la distancia recorrida por el robot durante una visita

/* Explicacion :

Esta funcion callback se va a ejecutar cada 0.1 segundos , por lo que suscribiendonos al topic /amcl_pose vamos a recibir la posicion (x,y) del robot en el mapa cada 0.1 segundos (dt = 0.1).
Al ser una variacion de tiempo muy pequeña o diferencial de tiempo (dt) , la variacion del vector de posicion del robot en dt es tambien muy pequeña (diferencial dr = dx*i + dy*j).

Durante dt = 0.1 , la distancia recorrida por el robot sobre su trayectoria es muy pequeña (diferencial dl), y el arco de la curva es despreciable (movimiento aproximadamente recto) --> en dt la distancia recorrida dl es aproximadamente recta y coincide con el modulo del verctor dr (variacion/diferencial del vector de posicion en dt) : 

dl = |dr| = RAIZ[dx² + dy²] = RAIZ[(x2 - x1)² + (y2 - y1)²]


Por tanto : Para calcular la distancia recorrida TOTAL lo que haremos sera que cada vez que se llaman las funciones callback (cada dt = 0.1 segundos) vamos a calcular la distancia recorrida por el robot 
            dl = |dr| en ese intervalo de tiempo dt = 0.1 segundos. Una vez termine la visita y el robot alcance la posicion final , se suman todos los dl = |dr| calculados durante la visita para obtener la 
            distancia recorrida TOTAL (lo que estamos haciendo realmente es INTEGRAR entre el instante inicial y el istante final)
*/


void cMeasurementsVisita::get_distanceTraveled(){
   
   
   // Visita acaba de iniciarse (visita_just_started = true) :
   
   if (visita_just_started){    // 1 unica vez por visita (cuando se ejecuta statusVisita() la primera vez tras iniciar la visita).
   
     distance_traveled_saved = false;
     first_dl_calculated = false;
     
     distance_traveled_visita = 0.0;  // antes de nada , reiniciamos a 0.0 la distancia total recorrida
     
     x_inicial = position_x;   // componente x de la posicion inicial del robot nada mas iniciar la visita (en metros)
     y_inicial = position_y;   // componente y de la posicion inicial del robot nada mas iniciar la visita (en metros)
      
   }
   
   
   
   // visita en progreso (visita_in_progress = true)
   
   else if (visita_in_progress){
      
      if (!first_dl_calculated){  // == false
         
         x2 = position_x;
         y2 = position_y;
         
         
         // Calcular distancia recorrida en dt : dl = modulo del vector dr = |dr| = RAIZ[(x2 - x1)² + (y2 - y1)²]. 
         
         dl = std::sqrt((x2 - x_inicial) * (x2 - x_inicial) + (y2 - y_inicial) * (y2 - y_inicial)); 
         
         
         // sumamos el dl calculado al valor total de la distancia recorrida
         
         distance_traveled_visita = distance_traveled_visita + dl;
         
         
         
         first_dl_calculated = true;        
      
      }
      
      
      
      else {    // first_dl_calculated == true  (estructura if-else : programa entra en bloque if o en bloque else , no entra en los 2)
      
         
         // Posicion anterior guardada en x2 , y2  pasa a x1 , y1 (llamada anterior a la actual de las funciones callback) 
         
         x1 = x2;
         y1 = y2; 
         
         
         // Posicion nueva actual se guarda en x2 , y2 (llamada actual de las funciones callback)
         
         x2 = position_x;
         y2 = position_y;
         
         
         // Calcular distancia recorrida en dt : dl = modulo del vector dr = |dr| = RAIZ[(x2 - x1)² + (y2 - y1)²]. 
         
         dl = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
         
         
         // sumamos el dl calculado al valor total de la distancia recorrida
         
         distance_traveled_visita = distance_traveled_visita + dl;
               
      }
         
   }
   
   
   
   // visita ha finalizado (visita_finished = true)
   
   else if (visita_finished){
   
      if (!distance_traveled_saved){   // == false
      
         ROS_INFO("Dustancia total recorrida por el robot durante la visita ha sido medida (en metros)");
         
         distance_traveled_saved = true;   
         
         // La distancia recorrida en metros es el ultimo valor calculado de la variable distance_traveled_visita durante la etapa de visita en progreso
      
      }
         
   }


}





// Funcion para medir la velocidad media (average speed) con la que se ha movido el robot durante la visita

// Velocidad Media (m/s) = Distancia recorrida (m) / Tiempo robot se esta moviendo o desplazando durante la visita (s)

void cMeasurementsVisita::get_averageSpeed(){

   
   // Visita acaba de iniciarse (visita_just_started = true) :
   
   if (visita_just_started){      // 1 unica vez por visita (cuando se ejecuta statusVisita() la primera vez tras iniciar la visita)
   
      average_speed_saved = false;   
   
   }
   
   
   
   // visita ha finalizado (visita_finished = true)
   
   else if (visita_finished){
   
      if (!average_speed_saved){    // == false
      
         average_speed_visita = distance_traveled_visita / time_robotMoving_visita_seconds;
         
         ROS_INFO("Velocidad media del robot durante la visita ha sido medida (en m/s)");
         
         average_speed_saved = true;
      
      }
   
   }

}





// Funcion para guardar/escribir las medidas tomadas de la visita en un archivo de texto (.txt) cuando finaliza la visita.

/* 

Las medidas se van a guardar/escribir en el archivo de texto "measurements.txt", que se encuentra en la carpeta del paquete measurements_visita.

Una cosa importante a saber es que cuando finaliza una visita y se guardan las medidas en el archivo "measurements.txt" , el archivo se sobreescribe con los datos nuevos y se borran los antiguos que tenia. 
Por tanto , es importante hacer una copia de los datos guardados en el archivo "measurements.txt" nada mas terminar la visita , porque cuando se realice una nueva visita el archivo se va a sobreescribir con las medidas de la nueva visita. */

void cMeasurementsVisita::save_measurements(){
   
   // Visita acaba de iniciarse (visita_just_started = true) :
   
   if (visita_just_started){    // == true
      
      /* en este bloque if solo se entra 1 unica vez por visita (cuando se ejecuta statusVisita() la primera vez tras iniciar la visita). 
      Despues de eso ya no se va a volver a entrar nunca mas hasta que no se inicie una nueva visita */
      
      measurements_saved = false;
   
   }
   
   
   // visita ha finalizado (visita_finished = true)
   
   else if (visita_finished){   
   
      if (!measurements_saved){  
         
         // obtenemos ruta del paquete measurements_visita con ros::package::getPath() y la guardo en variable local package_path [std::string]
         
         std::string package_path = ros::package::getPath("measurements_visita");
         
         
         if (package_path.empty()) {     // si variable package_path esta vacia (== "")
            
            ROS_ERROR("No se pudo encontrar el paquete measurements_visita");
            
            return;  // salgo de la funcion void save_measurements()
         }
         
         
         // creo la RUTA COMPLETA al archivo "measurements.txt" y la guardo en variable local file_path [std::string]
         
         std::string file_path = package_path + "/measurements.txt";
         
         
         // abro archivo "measurements.txt" en modo escritura (std::ofstream), de forma que cuando se escriba en el archivo se sobrescribe el contenido preexistente en el.
         // Para ello creamos un objeto outFile de clase std::ofstream y le pasamos como argumento la ruta completa del archivo. 
         
         std::ofstream outFile(file_path.c_str());
         
         
         // mediante el objeto outFile creado, escribimos en el archivo el valor de las variables que queremos guardar en formato decimal y mostrando un solo decimal 
         
         if (outFile.is_open()) {
         
            outFile << "==== RESULTADOS DE LA ULTIMA VISITA REALIZADA ==== " << "\n\n"
                    << " Tiempo total de la visita : " 
                    << std::fixed << std::setprecision(2) << time_total_visita_minutes << " minutos\n"
                    << " Tiempo que el robot ha estado en movimiento durante la visita : "
                    << std::fixed << std::setprecision(2) << time_robotMoving_visita_minutes << " minutos\n"
                    << " Distancia recorrida por el robot durante la visita : "
                    << std::fixed << std::setprecision(2) << distance_traveled_visita << " metros\n"
                    << " Velocidad media del robot : "
                    << std::fixed << std::setprecision(2) << average_speed_visita << " m/s\n";

            
            // cerramos el archivo y lo guardamos
            
            outFile.close();
            
            ROS_INFO("Datos guardados correctamente en: %s", file_path.c_str());
         
         } 
        
        
         else {
        
            ROS_ERROR("No se pudo abrir el archivo en: %s", file_path.c_str());
         
         }    
                  
         
         
         measurements_saved = true; 
           
      
      } // fin if (!measurements_saved) : Se entra una sola vez por visita (cuando measurements_saved == false) --> datos se guardan en el archivo una sola vez cuando finaliza la visita 
        
   } 

   
}





// Funcion Run() en la que se llevan a cabo las acciones de nodo cada vez que se repite el loop de la funcion main() y despues de llamar a las funciones callback

void cMeasurementsVisita::Run(){
   
   statusVisita();     // Funcion que determina en que estado se encuentra la visita de los 3 posibles : visita acaba de iniciarse, visita en progreso y visita finalizada
   
   
   get_time_visit_total();    // Funcion para medir el tiempo total de la visita (desde que se inicia hasta que finaliza)
   
   get_time_visit_robotMoving();   // Funcion para medir el tiempo en el que el robot se encuentra en movimiento (desplazandose) durante la visita
   
   get_distanceTraveled();       // Funcion para medir la distancia recorrida por el robot durante una visita
   
   get_averageSpeed();        // Funcion para medir la velocidad media (average speed) con la que se ha movido el robot durante la visita
   
   
   
   save_measurements();   // Funcion que guarda las medidas tomadas en el archivo "measurements.txt" cuando finaliza la visita.


} // fin funcion Run()



}  // fin namespace msr
