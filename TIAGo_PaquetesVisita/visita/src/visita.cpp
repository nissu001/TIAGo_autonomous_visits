//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Óscar García Fernández
// Last update: 19.12.2024
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/visita/cvisita.h"
#include "ros/package.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "visita");
	ros::NodeHandle n;

//	std::cout << "Starting visita..." << std::endl;
    
	visita::cvisita myvisita;
	myvisita.open(n); 
	
	


       std::string idioma_visita;
       bool idioma_elegido = false;
       
  //se solicita por terminal el idioma en que queremos realizar la visita hasta que se proporcione es (para Español) o en (para Inglés)     
       while(!idioma_elegido){
       
             std::cout << "En qué idioma quieres realizar la visita , Español (es) o Inglés (en) : ";
             std::cin >> idioma_visita;
             
             if (idioma_visita == "es" || idioma_visita == "en"){
             
                 std::cout << "Idioma elegido: " << idioma_visita << std::endl;
                 idioma_elegido = true;  //indicamos que el idioma ha sido elegido correctamente
             }
             
             else {
                 std::cout << "Comando incorrecto , por favor vuelva a introducir el Idioma" << std::endl;
             }
       }
       
       
       
       myvisita.PublicarIdioma(idioma_visita);           /* publico en el topic /visita/language el valor de idioma_visita (="es" para Español o ="en" para Ingles) ,
                                                            que nos indica el idioma que se ha elegido para realizar la visita*/
       
	

	
	
	
	std::string file_name;
	bool file_loaded = false;
		
	
	
//función de soliciud del programa de visita, se solicita por terminal el nombre del archivo, se queda en bucle hasta que se proporcione el nombre de uno de los programas de viita válidos.
	while (!file_loaded) {
	        std::cout << "Introduzca el nombre del programa de la visita: ";
	        std::cin >> file_name;
	        std::cout << "programa de visita: " << file_name << std::endl;
		myvisita.load_visit(file_name);
		if (!myvisita.rooms_visit.empty()) {
			file_loaded = true;
		}
		else {
			std::cout << "No se pudo cargar el archivo de la visita. Inténtelo de nuevo." << std::endl;
		}
	}
        
	ROS_INFO("Service ready");
	ros::Rate loop_rate(10);

	ROS_INFO("Init loop");
	// Loop
	while (ros::ok()) {
        
        	ros::spinOnce();
        	loop_rate.sleep();
        	myvisita.Run(idioma_visita);
	}

    return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


