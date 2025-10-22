//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Marcos Barranco Montilla
// Last update: 11.04.2025
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// En este archivo se definen los metodos o funciones declarados en c_object_detection.h , y que son los que se van a usar en object_detection.cpp


// includes (dependencias) :

#include "c_object_detection.h"





// creo el namespace obj_det --> funciones obj_det::

namespace obj_det {


// Constructor (inicializacion de atributos) :

cObjectDetection::cObjectDetection(){

   idioma_recibido_visita = "";
   
   time_period = "";
   
   text_tts_playing = false;       // indica si hay texto reproduciendose (true) o ya ha terminado de reproducirse (false)
   
   linear_vel_x = 0.0; 
   linear_vel_y = 0.0;
   robot_isMoving = false;
   
   number_objects_detected = 0;
   number_persons_detected = 0;
   
   info_active_displayed = false;
   info1_disabled_displayed = false;
   info2_disabled_displayed = false;
   person_detected_to_greet = false;
   greeting_done = false;
   counter_person_greeted_out = 0;      
      
}




// Destructor :

cObjectDetection::~cObjectDetection(){

   std::cout << "cObjectDetection::~cObjectDetection(): shutting down ROS" << std::endl;
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


// 1) /visita/language :   ((( recibe mensaje [std_msgs/String] del topic /visita/language y lo guarda en variable local msg1)))

/* Esta funcion callback recibe un mensaje tipo std_msgs::String del nodo "visita" que contiene el idioma que se ha elegido en el nodo "visita" dentro de un string con 2 posibles valores : 

   si msg1.data = "es" --> Español  
   si msg1.data = "en" --> Ingles     */

void cObjectDetection::language_callback(const std_msgs::String msg1){

   idioma_recibido_visita = msg1.data;      // guardo en idioma_recibido_visita el idioma elegido en nodo visita publicado en topic /visita/language. 
                                            // 3 posibles valores tendra idioma_recibido_visita : "" , "es" o "en"
                                   
                                            // con esta variable ya tenemos asignado el idioma con el que se va a hablar

}




// 2) /tts/status :   ((( recibe mensaje [actionlib_msgs/GoalStatusArray] del topic /tts/status y lo guarda en variable local msg2)))

/*
Este mensaje que llega del topic /tts/status nos permite saber si el ultimo texto publicado en /tts/goal se ha terminado de reproducir o se esta reproduciendo todavia (es decir, nos permite saber si en este intante se esta reproduciendo un texto por los altavoces o no).

Para ello tenemos que mirar el valor del campo status del ultimo componente del vector msg2.status_list , que nos dice si el texto ha terminado de reproducirse o sigue reproduciendose (nos dice si hay texto reproduciendose o no).


Suponiendo que msg2.status_list[n-1] es el ultimo componente del vector msg2.status_list :

si msg2.status_list[n-1].status = 0 --> Texto se ha publicado, pero no ha empezado a reproducirse todavia (pero lo va a hacer)
si msg2.status_list[n-1].status = 1 --> Texto se esta reproduciendo y no ha terminado

si msg2.status_list[n-1].status = 3 --> Texto ha terminado de reproducirse (no hay nungun texto reproduciendose actualmente)



Atencion : caso n=0 (vector vacio sin componentes) lo tenemos que hace a parte, ya que si n=0 --> n-1=-1 , lo que dara problemas porque no existe la componente [-1] en un vector

*/

void cObjectDetection::tts_status_callback(const actionlib_msgs::GoalStatusArray msg2){
   
   //primero sacamos el numero de componentes del vector msg2.status_list y lo guardamos en una variable local n
   
   int n = msg2.status_list.size();   
   
   
   // para caso n=0 (vector vacio) , no se ha publicado ningun texto por lo que no hay texto reproduciendose.
   
   if (n == 0){
   
      text_tts_playing = false;   // no hay textos publicados (vector msg2.status_list esta vacio , no tiene componentes) , por lo que no hay texto reproduciendose
   
   }
   
   
   //para caso n>0 , ahora miramos el valor del campo status del ultimo componente [n-1] del vector msg2.status_list para saber si hay texto reproduciendose o no
   
   if (n > 0){
               
      if (msg2.status_list[n-1].status == 1 || msg2.status_list[n-1].status == 0){
   
         text_tts_playing = true;  // ultimo texto todavia se esta reproduciendo (hay texto reproduciendose)
   
      }
         
      
      else if (msg2.status_list[n-1].status == 3){
            
          text_tts_playing = false;   // ultimo texto se ha terminado de reproducir (no hay texto reproduciendose)
            
      }
   
   } // fin if (n>0)

}




// 3) /mobile_base_controller/cmd_vel_out :   ((( recibe mensaje [geometry_msgs/TwistStamped] del topic /mobile_base_controller/cmd_vel_out y lo guarda en variable local msg3)))

// En este topic se publican las velocidades REALES que controlan el movimiento del robot, una vez ajustadas por el local planner  (velocidades que la base realmente está usando para moverse)


/* En esta funcion callback se va a determinar si el robot esta parado o esta en movimiento en funcion de las velocidades indicadas en el mensaje que se recibe

   robot_isMoving = false  --> robot esta parado (Vx = Vy = 0.0)
   robot_isMoving = true   --> robot se esta moviendo (al menos una de sus velocidades lineales es != 0.0) 
   
   
   Hemos comprobado que el robot solo se mueve usando Vx (velocidad lineal en x) y Wz (velocidad angular en z) . El resto de las velocidades son siempre = 0.0.
   
   Los valores de las velocidades que se publican en el topic /mobile_base_controller/cmd_vel_out son valores tipo float64 , que en C++ equivale a valores tipo double , y su cero exacto es = 0.0.
   Por tanto , cuando las velocidades se anulan se publica en el topic un valor = 0.0 (comprobado con rostopic echo)
   
*/

void cObjectDetection::cmd_vel_out_callback(const geometry_msgs::TwistStamped msg3){

   // msg3.twist.linear.x y msg3.twist.linear.y son valores tipo float64, que en C++ equivalen a valores tipo double --> Su cero exacto se escribe = 0.0 (es lo que tengo que poner en los bloques if)

   linear_vel_x = msg3.twist.linear.x;    // Vx (lineal) de la base  
   linear_vel_y = msg3.twist.linear.y;    // Vy (lineal) de la base
   
   
   if (linear_vel_x == 0.0 && linear_vel_y == 0.0) {
     
     robot_isMoving = false;   // robot parado
     
   }
   
    
   else {      // si (linear_vel_x != 0.0 || linear_vel_y != 0.0)
      
      robot_isMoving = true;  // robot en movimiento
  
   }

}




// 4) /yolov8/BoundingBoxes :   ((( recibe mensaje [yolov8_ros_msgs/BoundingBoxes] del topic /yolov8/BoundingBoxes y lo guarda en variable local msg4))) 

/* En el topic /yolov8/BoundingBoxes se publica el vector de objetos detectados (vector bounding_boxes[]) con toda la informacion de los objetos detectados y las dimensiones de las cajas de deteccion */ 


/* IMPORTANTE : Vamos a establecer en el nodo que solo consideraremos como persona detectada a los objetos detectados de clase person con una probabilidad >= 0.5 (50 %) = Es decir , establecemos un umbral de confianza para las personas (objetos de clase persona) del 0.5 en este nodo. 
Como en el nodo yolov8_ros el umbral de confianza que usamos es muy pequeño (0.02 = 2 %) para que puedan detectarse las mesas, lo que vamos a hacer es que el umbral de confianza lo establecemos aquí dentro de este nodo para que se descarten los falsos positivos (que en este caso seran las personas detectadas con probabilidad < 0.5)*/


// En esta funcion callback vamos a guardar el mensaje publicado en el topic /yolov8/BoundingBoxes (que contiene el vector de objetos detectados bounding_boxes[]) y luego vamos a calcular el numero de objetos detectados = numero componentes vector bounding_boxes[] y el numero de personas detectadas

void cObjectDetection::bboxes_callback(const yolov8_ros_msgs::BoundingBoxes msg4){
   
   bb_msg = msg4; 
   
      
   // obtenemos el numero de objetos detectados = numero componentes del vector bounding_boxes[] : 
    
   number_objects_detected = bb_msg.bounding_boxes.size();
   
   
   
   // obtenemos el numero de personas detectadas : 
   
   /* Vamos a establecer que un objeto se cuenta como persona detectada cuando :
      - su clase es person (class = person)
      - su probailidad es >= 0.50 = PROB_MIN_PERSON  (probabilidad >= umbral confianza establecido para persona)
      - la altura de su caja de deteccion (ymax - ymin) es >= HEIGHT_MIN_PERSON_DETECTED  (>= 200 px)
      
      Por tanto, solo vamos a contar como personas detectadas a los objetos de clase "person" con probabilidad >= 0.5 y cuya caja de deteccion sea de altura >= HEIGHT_MIN_PERSON_DETECTED (px) 
   */
   
   int i = 0;
   number_persons_detected = 0;   // reiniciamos number_persons_detected a 0 antes de iniciar la cuenta
   
   for (i=0; i <= number_objects_detected - 1; i++){
   
      if (bb_msg.bounding_boxes[i].Class == "person" 
          && bb_msg.bounding_boxes[i].probability >= PROB_MIN_PERSON 
          && bb_msg.bounding_boxes[i].ymax - bb_msg.bounding_boxes[i].ymin >= HEIGHT_MIN_PERSON_DETECTED) {
          
             number_persons_detected ++;
          
      }
   
   }   // fin bucle for
   
   
   
   
   


}



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//Funcion (metodo) que define los Publishers y Subscribers del nodo object_detection (en esta funcion defino las variables Publisher y Subscriber que he declarado como atributos de la clase):     

void cObjectDetection::Topics(ros::NodeHandle& n){

   //Publishers:
   
   tts_pub = n.advertise<pal_interaction_msgs::TtsActionGoal>("/tts/goal", 1);        //publica en topic /tts/goal (para habla) mensajes tipo [pal_interaction_msgs/TtsActionGoal]
   
   
   
   //Subscribers:
   
   language_sub = n.subscribe("/visita/language", 1, &cObjectDetection::language_callback, this);     // se suscribe a topic /visita/language [std_msgs/String]
   
   tts_status_sub = n.subscribe("/tts/status", 1, &cObjectDetection::tts_status_callback, this);      // se suscribe a topic /tts/status [actionlib_msgs/GoalStatusArray]
   
   cmd_vel_out_sub = n.subscribe("/mobile_base_controller/cmd_vel_out", 1, &cObjectDetection::cmd_vel_out_callback, this);   // se suscribe a topic /mobile_base_controller/cmd_vel_out
   
   bboxes_sub = n.subscribe("/yolov8/BoundingBoxes", 1, &cObjectDetection::bboxes_callback, this);    // se suscribe a topic /yolov8/BoundingBoxes [yolov8_ros_msgs/BoundingBoxes]



}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




// Funcion que obtiene la hora del reloj del PC, la convierte a estructura tm (estructura hora local = horas : minutos : segundos) y determina el periodo del día en el que nos encontramos : "morning", "afternoon" o "evening". Finalmente muestra los resultados por pantalla.

/*  
hora actual < 13:00 (hasta 12:59) --> time_period = "morning" 
13:00 <= hora actual < 18:00 (hora actual >= 13:00 && hora actual < 18:00) --> time_period = "afternoon" 
hora actual >= 18:00 --> time_period = "evening" 
*/

void cObjectDetection::getTimePeriod(){
   
   // Obtener hora actual del sistema (del PC) y la guardamos en variable local std::time_t current_time
   auto clock_time_now = std::chrono::system_clock::now();
   std::time_t current_time = std::chrono::system_clock::to_time_t(clock_time_now);
   
   
   // Convertimos la hora a estructura tm (estructura hora local de horas:min:seg) y la guardamos en variable local std::tm time_info
   
   std::tm time_info = *std::localtime(&current_time);  // Copia del struct tm al que apunta el puntero devuelto por std::localtime
   
   
   // Extraer horas y minutos de time_info (variable que contiene la hora en estructura tm)
   int current_hour = time_info.tm_hour;
   int current_minutes = time_info.tm_min;
   
   
   // Determinar periodo del dia (se guarda en atributo privado std::string time_period)
      
   if (current_hour <= 12){     // hasta las 12:59
   
      time_period = "morning";
   }
   
   else if (current_hour < 18){   // current_hour está entre 13 y 17 (desde 13:00 hasta 17:59)
   
      time_period = "afternoon";   
   }
   
   else {  // current_hour >= 18 (desde 18:00 en adelante)
   
      time_period = "evening";   
   }
   
   
   
   // Mostrar resultados por pantalla en terminal
   
   std::cout << "Hora actual: " << std::put_time(&time_info, "%H:%M") << "\nPeríodo: " << time_period << std::endl;

}





//Funcion para reproducir un texto en el robot (en español o inglés). Despues de publicar el texto se detiene el codigo hasta que el texto termina de reproducirse.

void cObjectDetection::play_dialog(const std::string& dialog , const std::string& idioma_dialog){
   
   pal_interaction_msgs::TtsActionGoal goal_msg;
	
	
   goal_msg.goal.rawtext.text = dialog;      /* metemos en el campo goal.rawtext.text de goal_msgs (tipo pal_interaction_msgs::TtsActionGoal) el texto que 
	                                        le pasamos como argumento a la funcion y que se guarda en la variable local constante dialog (tipo std::string).
	                                        Este texto es el que queremos reproducir */
	
	
   if (idioma_dialog == "es"){
      
      goal_msg.goal.rawtext.lang_id = "es_ES";   //dialogo en español
   
   }
   
        
   else if (idioma_dialog == "en"){
      
      goal_msg.goal.rawtext.lang_id = "en_GB";  //dialogo en ingles
   
   }
        
         
            
   tts_pub.publish(goal_msg);       /* publico en el topic tts/goal el mensaje goal_msg tipo pal_interaction_msgs::TtsActionGoal, que contiene en sus campos el
	                               texto que quiero reproducir y el idioma en el que quiero que se reproduzca. 
	                               El texto comenzara a reproducirse automaticamente nada mas publicarse el mensaje. */
	                                   
	                              
   ROS_INFO("Published TTS goal: %s", goal_msg.goal.rawtext.text.c_str());
   
   
   
   
   // Detengo el codigo en este punto mientras el texto se reproduce. 
   
   text_tts_playing = true;  // actualizo valor de text_tts_playing a true para indicar que se esta reproduciendo un texto
   
   
   ros::Rate loop2_rate(10);
   
   while (text_tts_playing){   // == true
   
      ros::spinOnce();  // llamo a las funciones callback una vez
      
      loop2_rate.sleep(); 
        
   }    // Se sale del while cuando el texto termina de reproducirse
   
}





// Funcion para hablar (en español o ingles) cuando se detecta una persona que cumple las condiciones establecidas.

// El texto que se publica y reproduce va a depender del valor del atributo std::string time_period, donde se guarda el periodo del dia en el que nos encontramos cuando se ejecuta la funcion getTimePeriod() una unica vez al iniciar el nodo (en object_detection.cpp). 

void cObjectDetection::talk_person(const std::string idioma){

   if (idioma == "es"){
   
      if (time_period == "morning"){
   
         play_dialog("Hola, buenos días", idioma); 
      }
      
            
      else if (time_period == "afternoon" || time_period == "evening"){
      
         play_dialog("Hola, buenas tardes", idioma);
      }
           
   }
   
   
   
   else if (idioma == "en"){
      
      if (time_period == "morning"){
      
         play_dialog("Hello, good morning", idioma);
      }
      
      
      else if (time_period == "afternoon"){
      
         play_dialog("Hello, good afternoon", idioma);
      }
      
      
      else if (time_period == "evening"){
      
         play_dialog("Hello, good evening", idioma);
      }
      
   }

}





// Funcion para saludar a una persona detectada con la camara RGB del robot mediante YOLO mientras el robot se encuentra en movimiento (robot_isMoving == true) y pasa al lado de la persona.


/* Se saluda cuando se detecta a una persona (objeto con class = "person") que cumple las siguientes condiciones :

1) Probabilidad >= PROB_MIN_PERSON (>= 0.50)
2) Altura cuadro deteccion >= HEIGHT_MIN_GREET_PERSON (>= 370 px)
3) Cuadro deteccion en un extremo de la imagen : Xmax <= XMAX_GREET_PERSON (<= 240 px) ó Xmin >= XMIN_GREET_PERSON (>= 400 px)

Solo se va a saludar (publicar texto en /tts/goal) cuando el robot se encuentre avanzando (robot_isMoving = true), no se esté reproduciendo ningun texto en ese momento (text_tts_playing = false) y el numero de personas detectadas sea <= 3 (mas de 3 personas detectadas lo consideramos público)  */


/* Después de saludar se espera a que la persona se retire de la imágen y se resetean las flags. Para ello se establecen las siguientes directrices :

a) Solo vamos a considerar como persona detectada a los objetos detectados de clase "person" con Probabilidad >= PROB_MIN_PERSON (>= 0.50).
b) Mientras se detecte a una persona con un cuadro de deteccion de altura >= HEIGHT_SAME_PERSON (>= 300 px), vamos a suponer que es la misma persona.
c) Si se detecta a una persona con un cuadro de deteccion de altura < HEIGHT_SAME_PERSON (< 300 px), consideramos que es otra persona distinta. 

La función bboxes_callback() recibe nuevos mensajes continuamente con frecuencia <= 10 Hz, por lo que cada vez que se repite el loop de la funcion main() y se llama a bboxes_callback(), esta función se va a ejecutar y vamos a recibir un nuevo mensaje con los objetos detectados en ese momento.

Para resetear las flags y poder saludar de nuevo, ponemos como condicion que durante 15 repeticiones seguidas del loop principal (que equivale a 1.5 segundos) no se detecte a la misma persona a la que ya se ha saludado. Para ello vamos a usar un contador que se reinicia a 0 si se detecta a la misma persona y suma +1 si no se detecta, de forma que las flags se resetean cuando el contador es = 15 */


void cObjectDetection::greetPerson(const std::string language){
   
   int i = 0;
   

   // código para saludar (robot avanzando, NO hay texto reproduciendose (false) y numero personas detectadas <= 3)
   
   if (robot_isMoving && !text_tts_playing && number_persons_detected <= 3) {    
   
      info1_disabled_displayed = false;
      info2_disabled_displayed = false;
      
      if (!info_active_displayed){   // == false
      
         ROS_INFO("greetPerson en funcionamiento");
         
         info_active_displayed = true;     // mensaje solo se muestra una sola vez         
      }
      
      
      
      if (!person_detected_to_greet){   // == false
      
         for (i=0; i <= number_objects_detected - 1; i++){  
            
            // condiciones para saludar
            
            if (bb_msg.bounding_boxes[i].Class == "person" 
                && bb_msg.bounding_boxes[i].probability >= PROB_MIN_PERSON
                && bb_msg.bounding_boxes[i].ymax - bb_msg.bounding_boxes[i].ymin >= HEIGHT_MIN_GREET_PERSON
                && (bb_msg.bounding_boxes[i].xmax <= XMAX_GREET_PERSON || bb_msg.bounding_boxes[i].xmin >= XMIN_GREET_PERSON)){
                
                
                   person_detected_to_greet = true;   // se cumplen condiciones para saludar
                  
                   break;  // sal del bucle for                             
            }
         
         } // fin bucle for (se revisa uno a uno los componentes del vector bouding_boxes)
      
      } // fin if (!person_detected_to_greet)
      
      
      
      if (person_detected_to_greet && !greeting_done){  
      
         ROS_INFO("Persona cerca detectada, saludando ...");
         
         talk_person(language);    // saludamos a la persona detectada (código se detiene mientras se reproduce el texto)
         
         greeting_done = true;     
      
      }
     
   }
   
   
   
   else if (!robot_isMoving || text_tts_playing){    // robot sin avanzar ó hay texto reproduciendose (true)
   
      if (!info1_disabled_displayed){   // == false
      
         ROS_INFO("Robot se encuentra parado y/o reproduciendo un texto, greetPerson en pausa");
         ROS_INFO("Esperando...");
         
         info_active_displayed = false;
         
         info1_disabled_displayed = true;       
      }
   }
   
  
   
   else if (number_persons_detected > 3){
   
      if (!info2_disabled_displayed){   // == false
      
         // Si el numero de personas detectadas es > 3, se considera que es el público (robot no reproduce nada)
         
         std::cout << "Cuento un número de " << number_persons_detected << " personas en el público" << std::endl;
         
         info_active_displayed = false;
         
         info2_disabled_displayed = true;     
      }
   
   }

  
   
   
   // código para despues de saludar (greeting_done = true)
   
   if (greeting_done){
   
      for (i=0; i <= number_objects_detected - 1; i++){
      
         if (bb_msg.bounding_boxes[i].Class == "person" 
             && bb_msg.bounding_boxes[i].probability >= PROB_MIN_PERSON
             && bb_msg.bounding_boxes[i].ymax - bb_msg.bounding_boxes[i].ymin >= HEIGHT_SAME_PERSON){
             
                
                // persona no se ha retirado: se pone a 0 el contador y se sale del bucle for (deja de revisar los componentes del vector)
                
                counter_person_greeted_out = 0;
                break;            
         }
         
         
         else if (i == number_objects_detected - 1){    // comprobado ultimo componente vector
         
            counter_person_greeted_out ++;   // +1 al contador
         }   
          
      } // fin bucle for
      
      
      
      if (counter_person_greeted_out == 15){
         
         ROS_INFO("persona cercana ha dejado de detectarse");
         
         // reseteamos las flags para saludar y reiniciamos el contador a 0
         
         person_detected_to_greet = false;
         greeting_done = false;
         
         counter_person_greeted_out = 0;      
      }      
     
     
   } // fin if(greeting_done)
   
   
}





// Funcion Run() en la que se llevan a cabo las acciones de nodo cada vez que se repite el loop de la funcion main() y despues de llamar a las funciones callback

void cObjectDetection::Run(const std::string language_chosen){

   greetPerson(language_chosen);   

}


} // fin namespace obj_det

