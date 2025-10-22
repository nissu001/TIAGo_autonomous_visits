//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Óscar García Fernández
// Last update: 19.12.2024
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/visita/cvisita.h"
#include "pugixml.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "structs.h"
#include "ros/package.h"
#include <pal_interaction_msgs/TtsActionGoal.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"



namespace visita {

// Función para eliminar caracteres extra como saltos de línea al leer los archivos de diálogos
std::string cvisita::trim_end(const std::string& str) {
	size_t end = str.find_last_not_of(" \n\r\t");
	return (end == std::string::npos) ? "" : str.substr(0, end + 1);
}

// Función que lee los textos de los diálogos en la llamada a la función se da la ruta de la carpeta de diálogos y en el segundo parámetro el nombre del lugar del cual tiene que cargar el diálogo, la función reotorna el texto leido tras filtrarlo con trim_end para eliminar caracteres no desados. 
std::string cvisita::load_dialog(const std::string& base_path_dialog, const std::string& place) {
	std::string file_path_dialog = base_path_dialog + "/" + place;
	std::ifstream file(file_path_dialog);
	if (!file.is_open()) {
	        ROS_ERROR("No se pudo abrir el archivo: %s", file_path_dialog.c_str());
	        return "";
			        
	}
    
	std::stringstream buffer;
	buffer << file.rdbuf();
	std::string content = buffer.str();
	return trim_end(content);
}

// Esta función carga el programa de la visita, el programa de visita será un archivo de texto que se encuentre en la carpeta programas_visita. El archivo debe contener en cada línea primero la habitación de destino seguido de un espacio en blanco y el lugar de la habitación que se quiere visitar. Las habitaciones se guardan en un vector llamado rooms_visit y los lugares en otro vector llamado places_visit. 
void cvisita::load_visit(const std::string& filename) {
	std::string base_path_visit = ros::package::getPath("visita") + "/programas_visita";
	std::string file_path_visit = base_path_visit + "/" + filename;

	std::ifstream file(file_path_visit);
	if (!file.is_open()) {
	        ROS_ERROR("No se pudo abrir el archivo de la visita: %s", file_path_visit.c_str());
	        return;
	}

	std::string line;
	while (std::getline(file, line)) {
        if (!line.empty()) {
		size_t space_pos = line.find(' ');
		if (space_pos != std::string::npos) {
			std::string room = line.substr(0, space_pos);
			std::string place = line.substr(space_pos + 1);
			rooms_visit.push_back(room);
			places_visit.push_back(place);
		}
	}
	}

	file.close();
	ROS_INFO("Visita cargada con %lu lugares.", rooms_visit.size());
}

//función usada para publicar la habitación y el lugar al que se quiere ir, la habitación se publica en el topic room_name_target y el lugar en place_name_target, estos topics serán leidos en el nodo  places_interpretation, que buscará en el archivo xml el lugar y mandará sus coordenadas al nodo sending goals, para que las publique en move_base y se mueva el robot al destino buscado.  
void cvisita::publish_place(const std::string& room, const std::string& place) {
	
	std_msgs::String room_msg;
	room_msg.data = room;
	room_name_pub.publish(room_msg);
	ROS_INFO("Published room: %s", room.c_str());

	std_msgs::String place_msg;
	place_msg.data = place;
	place_name_pub.publish(place_msg);
	ROS_INFO("Published place: %s", place.c_str());
}

//Esta función publica en el topic goal de tts del robot. En raxtext.lang_id se proporciona el idioma es_ES es español en este caso, podría usarse también en_GB correpondiente al inglés. En rawtext.text se escribirá el texto que queremos que reproduzca, la función está preparada para reproducir el texto que se pasa por llamada a la función, que será el que haya cargado la función load_dialog.
void cvisita::publish_dialog(const std::string& dialog , const std::string& idioma_dialog) {
	pal_interaction_msgs::TtsActionGoal goal_msg;
	goal_msg.goal.rawtext.text = dialog;
	
	
	if (idioma_dialog == "es"){
	   goal_msg.goal.rawtext.lang_id = "es_ES";   //dialogo en español
        }
        
        else if (idioma_dialog == "en"){
           goal_msg.goal.rawtext.lang_id = "en_GB";  //dialogo en ingles
        }
        
         
            
	tts_pub.publish(goal_msg);
	ROS_INFO("Published TTS goal: %s", goal_msg.goal.rawtext.text.c_str());
}

//función para saber si quedan más lugares por visitar
bool cvisita::has_more_places() const {
	return current_place_index < rooms_visit.size();
}

//callback del suscriptor a goal_status que nos avisará cuando se hayan alcanzado las coordenadas mandadas 
void cvisita::goal_status_callback(const std_msgs::String::ConstPtr& msg) {
	if (msg->data == "reached") {
		goal_sent = false;
        	goal_reached = true;
        	ROS_INFO("Goal reached received.");
	}
}



// funcion callback del topic /tts/status , en la que recibe un mensaje tipo actionlib_msgs::GoalStatusArray que se guarda en la variable local msg2.

/*
Este mensaje que llega del topic /tts/status nos permite saber si el ultimo texto publicado en /tts/goal se ha terminado de reproducir o se esta reproduciendo todavia (es decir, nos permite saber si en este intante se esta reproduciendo un texto por los altavoces o no).

Para ello tenemos que mirar el valor del campo status del ultimo componente del vector msg2.status_list , que nos dice si el texto ha terminado de reproducirse o sigue reproduciendose (nos dice si hay texto reproduciendose o no).


Suponiendo que msg2.status_list[n-1] es el ultimo componente del vector msg2.status_list :

si msg2.status_list[n-1].status = 0 --> Texto se ha publicado, pero no ha empezado a reproducirse todavia (pero lo va a hacer)
si msg2.status_list[n-1].status = 1 --> Texto se esta reproduciendo y no ha terminado

si msg2.status_list[n-1].status = 3 --> Texto ha terminado de reproducirse (no hay nungun texto reproduciendose actualmente)



Atencion : caso n=0 (vector vacio sin componentes) lo tenemos que hace a parte, ya que si n=0 --> n-1=-1 , lo que dara problemas porque no existe la componente [-1] en un vector

*/

void cvisita::tts_status_callback(const actionlib_msgs::GoalStatusArray msg2){
   
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




//función donde se declaran los publishers y suscriptores
void cvisita::open(ros::NodeHandle& n) {
	// Publishers
	tts_pub = n.advertise<pal_interaction_msgs::TtsActionGoal>("/tts/goal", 10);
	room_name_pub = n.advertise<std_msgs::String>(ROOM_NAME_TARGET, 10);
	place_name_pub = n.advertise<std_msgs::String>(PLACE_NAME_TARGET, 10);
	
	language_pub = n.advertise<std_msgs::String>("/visita/language", 10);
	
	visita_status_pub = n.advertise<std_msgs::String>("/visita/status", 10);   
	

        
	// Subscribers
	goal_status_sub = n.subscribe(GOAL_STATUS, 10, &cvisita::goal_status_callback, this);
	
	tts_status_sub = n.subscribe("/tts/status", 10, &cvisita::tts_status_callback, this);    


	opened = true;
}

// Constructor con la inicialización de variables
cvisita::cvisita() {
	opened = false;
	bRun = true;
	finished = false; 
	goal_reached = false; 
	goal_sent = false;
	
	text_tts_playing = false;  
}

// Destructor
cvisita::~cvisita() {
	printf("cvisita::~cvisita(): shutting down ROS\n");
	usleep(100000);
	if (ros::isStarted()) {
	        ros::shutdown();
	        ros::waitForShutdown();
	}
	usleep(100000);
	printf(" - done\n");
}




// Funcion para publicar el idioma elegido para la visita (Español : "es" , Ingles : "en") en el topic /visita/language :

void cvisita::PublicarIdioma(const std::string& idioma){

       std_msgs::String msg;
       
       msg.data = idioma;
       
       
       language_pub.publish(msg);           /* publico en el topic /visita/language el mensaje "msg" de tipo std_msgs::String con 
                                               el idioma elegido en su campo data ("es" o "en")*/
       
}





// Bucle principal, aquí se desarrolla la ejecución de la visita. En primer lugar se comprueba que la bandera finished no esté activada, si lo está indica que acabó la visita y saldrá de la función. El funcionamiento prosigue de la siguiente manera, se comprueba si se ha alcanzado un punto mediante la bandera goal_reached que se activará en el callback del topic "goal_status" cuando se haya publicado en el topic un "reached". Si se ha llegado a un punto se reproduce el diálogo del lugar al que se ha llegado mediante la función publish_dialog, se aumenta en 1 el índice que gestiona los lugares y habitaciones de la visita. Se comprueba mediante la función has_more_places que queden lugares de visitar por delante, si es correcto publicará la siguiente habitación con su respectivo lugar para que el robot vaya a él y vuelva a comenzar el bucle. En caso de de no quedar más lugares publica la despedida y activa finished para finalizar la visita. Mediante el último else se gestiona que se mueva al primer lugar de la visita.
void cvisita::Run(const std::string& idioma_run) {
	
	std::string base_path;
	std::string place;
	
	std::string dialogo_final_es;
	std::string dialogo_final_en;
	int num_characters = 0;
	
	std_msgs::String status_visita_msg;  
	
	
	
	
	
	if (finished) {
	        return; 
	}

	if (goal_reached) {
	        if (has_more_places()) { 
			
			if (idioma_run == "es"){
			    base_path = ros::package::getPath("visita") + "/dialogos_ES";  /*defino una variable std::string donde se guarda la ruta de la carpeta /dialogo_ES del paquete "visita",
			                                                                                 donde están los diálogos en español */
			    place = places_visit[current_place_index];
                       }
                       
                       else if (idioma_run == "en"){
                            base_path = ros::package::getPath("visita") + "/dialogos_EN"; /*defino una variable std::string donde se guarda la ruta de la carpeta /dialogo_EN del paquete "visita",
			                                                                                 donde están los diálogos en Ingles */
			     place = places_visit[current_place_index];
                       }
			
			
			std::string dialog_text = load_dialog(base_path, place);
			publish_dialog(dialog_text,idioma_run);  //lanzamos la funcion para publicar y leer los dialogos en el idioma indicado
			
			

			
			num_characters = dialog_text.length();     // cuenta el numero de caracteres del dialogo guardado dentro de la variable dialogo_text (que es un std::string) 
			
			
			ros::Duration(num_characters * 0.06).sleep();      /* "dormimos" o detenemos el código en este punto un tiempo en segundos para que el robot diga el dialogo cargado.
			                                                       Le damos 0.06 segundos a cada caracter del dialogo cargado. 
			                                                       El valor que se le pasa a la funcion Duration como argumento es un valor tipo double con el tiempo en segundos 
			                                                       que queremos parar el codigo */
                       

			
			current_place_index++;
			goal_reached = false; 

			if (has_more_places()) {
				auto& next_room = rooms_visit[current_place_index];
				auto& next_place = places_visit[current_place_index];
				publish_place(next_room, next_place);
				goal_sent = true;
			} 
			
			else {
				
				
				
				ros::spinOnce();      /* primero llamamos a los callbacks para que se actualice el valor de text_tts_playing a true. Aqui llegamos sin llamar a las callbacks desde antes de 
				                         publicar el ultimo texto (cuando text_tts_playing era = false). Si no hacemos esto, text_tts_playing llega siempre con valor false y 
				                         NO ENTRA EN EL BUCLE WHILE, pasa de largo sin ejecutar ni una sola vez lo que hay dentro del bucle (por lo que no llama a las callbacks y no se 
				                         actualiza el valor de text_tts_playing)*/
				
				
				ros::Rate loop2_rate(10);
				
				ROS_INFO("entra en el bucle loop2");
				
				
				// mientras se este reproduciendo el texto (text_tts_playing == true) , paramos el codigo y esperamos
				
				while (text_tts_playing == true){
				
				   ros::spinOnce();
				   loop2_rate.sleep();
				
				}
				
							
				ROS_INFO("Se ha salido del bucle. Se espera lo programado y publica el texto final");
				
				
				
				
				
				ros::Duration(2.0).sleep();    // Cuando termine de reproducirse el texto del utlimo punto , paramos codigo y esperamos 2 segundos antes de lanzar el mensaje de despedida
				
				
				if (idioma_run == "es"){
				
				   dialogo_final_es = "Gracias por la atención, hasta pronto.";
				   publish_dialog(dialogo_final_es, idioma_run);     // se publica el dialogo de despedida (en español)
				   
				    
				   num_characters = dialogo_final_es.length();     // cuenta el numero de caracteres del dialogo guardado dentro de la variable dialogo_final_es (que es un std::string)   
				   
				   
				   ros::Duration(num_characters * 0.06).sleep();    // duermo el codigo num_characters*0.06 segundos)  
				   
				}
				
				
				else if (idioma_run == "en"){
				   
				   dialogo_final_en = "Thank you for your attention, see you soon.";
				   publish_dialog(dialogo_final_en, idioma_run);   //se publica el dialogo de despedida (en ingles)
				   
				   
				   num_characters = dialogo_final_en.length();     // cuenta el numero de caracteres del dialogo guardado dentro de la variable dialogo_final_en (que es un std::string)   
				   
				   
				   ros::Duration(num_characters * 0.06).sleep();    // duermo el codigo num_characters*0.06 segundos)  
				   
				}
				
				ROS_INFO("Completed all places.");
				finished = true;
				
				
				
				
				// Fin visita : publico mensaje "finished" en topic /visita/status
				
				status_visita_msg.data = "finished";
				visita_status_pub.publish(status_visita_msg); 
				
				
			}
		}
	} 

	else {
		if (current_place_index == 0 && !finished && !goal_sent) {
			auto& room = rooms_visit[current_place_index];
			auto& place = places_visit[current_place_index];
			publish_place(room, place);
			goal_sent = true;
			
			
			
				
			// Inicio visita (se publica el primer destino de todos) : publico mensaje "started" en topic /visita/status
				
			status_visita_msg.data = "started";
			visita_status_pub.publish(status_visita_msg); 
			
			
			
		}
	}
}

} // namespace visita


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Publisher for the places
