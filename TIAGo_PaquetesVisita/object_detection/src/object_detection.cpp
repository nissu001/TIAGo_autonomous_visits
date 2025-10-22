//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Marcos Barranco Montilla
// Last update: 11.04.2025
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// archivo con la funcion main() del nodo object_detection , donde se realiza el bucle infinito o loop (con frecuencia de 10 Hz = 0.1 segundos)


// includes (dependencias) :

#include "c_object_detection.h"




// Funcion main() :  

int main(int argc, char **argv){


   ros::init(argc, argv, "object_detection");    // creo e inicializo el nodo "object_detection" (nombre del nodo que aparece en rostopic list si lo inicializo con rosrun)
   ros::NodeHandle n;                            // creo un NodeHandle n (solo 1 para todos los topics)
   
   
   obj_det::cObjectDetection my_detection;       /*creo un objeto "my_detection" de clase obj_det::cObjectDetection, con lo que se crean todos
                                                  los metodos y atributos de la clase para el objeto */
   
   
   my_detection.Topics(n);       // Llamo al metodo cObjectDetection::Topics() del objeto "my_detection" y le mando como argumento el NodeHandle n que acabo de crear.
                                 // Este metodo define los Publishers y Subscribers del nodo
                             
                                 /* no hay que usar el namespace obj_det:: para llamar al metodo del objeto creado.
                                    Esto es porque ya lo hemos usado para crear el objeto.*/
   
   
   
   my_detection.getTimePeriod();    // Llamo al metodo publico cObjectDetection::getTimePeriod() 
                                    // obtengo hora actual del PC, determino periodo del dia y se muestran por pantalla.
   
   
   
   
   std::string idioma_visita = "";      // declaro una variable LOCAL tipo string de la funcion main() en la que se guarda el idioma ("en" o "es") de la visita
   bool idioma_asignado = false;        // flag que nos indica si el idioma ha sido guardado en la variable idioma_visita o no
   
   
   
   
   // Descomentar esto y comentar lineas del loop indicadas mas abajo para ejecutar el nodo por su cuenta sin necesidad de ejecutar el nodo visita // 
   
   /*
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
   // se solicita por terminal el idioma en que queremos realizar la visita hasta que se proporcione "es" (para Español) o "en" (para Inglés)     
   
   while(!idioma_asignado){
       
        std::cout << "En qué idioma quieres realizar la visita , Español (es) o Inglés (en) : ";
        std::cin >> idioma_visita;
             
        if (idioma_visita == "es" || idioma_visita == "en"){
             
            std::cout << "Idioma elegido: " << idioma_visita << std::endl;
            
            idioma_asignado = true;                         //indicamos que el idioma ha sido elegido correctamente y salimos del bucle while           
        }
             
        else {
        
           std::cout << "Comando incorrecto , por favor vuelva a introducir el Idioma" << std::endl;
        }
   }
	
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
   */
   
   
   
   ROS_INFO("Ready to run");
   
   
   ros::Rate loop_rate(10);      // establezco la frecuencia del Loop o Bucle infinito = 10 Hz (0.1 s)
   
   ROS_INFO("Init loop");
   ROS_INFO("Waiting for language selection at visit node ... ");
   
   
   
   // loop (bucle infinito que se repite con una frecuencia de 10 Hz = 0.1 s hasta que se "mata" el nodo. Por tanto las callbacks se llaman cada 0.1 s ) :
   
   while (ros::ok()) {
   
      ros::spinOnce();         // se llama a las callbacks 1 sola vez (cada 0.1 s = 10Hz)
      loop_rate.sleep();       // Pausa para mantener la frecuencia de repetición deseada del loop (10 Hz)
      
      
      
      
      /////////////////// (comentar este tramo si queremos ejecutar el nodo por su cuenta sin necesidad de ejecutar al mismo tiempo el nodo "visita") /////////////////////
        
      if (idioma_visita != my_detection.idioma_recibido_visita){
        
         idioma_asignado = false;
        
      }    
        
      // Esto ultimo se hace para que si cambiamos de idioma en nodo "visita" sin desactivar el nodo "object_detection", el idioma se vuelva a asignar tambien en "object_detection":
      // Si el string idioma_visita es distinto del string idioma_recibido_visita que llega del nodo "visita", entonces 
      // pon idioma_asignado = false y vuelves a meterte en el if para asignar de nuevo el idioma.
        
        
        
        
      if (!idioma_asignado){   // == false
        
         idioma_visita = my_detection.idioma_recibido_visita;
           
           
         if (idioma_visita == "es" || idioma_visita == "en"){
           
            std::cout << "Selected language: " << idioma_visita << std::endl;
            ROS_INFO("object_detection running");
              
            idioma_asignado = true;                       //indicamos que el idioma ha sido elegido correctamente
         }  
      }
        
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      
      
      if (idioma_asignado) {   // (== true) . Aqui metemos el codigo que se va a ejecutar
           
      my_detection.Run(idioma_visita);    // hasta que no se asigna por primera vez el idioma, la funcion Run() no se ejecuta
        
      }
   
      
   }  // fin del loop (sale cuando se "mata" el nodo)
   
   
return 0;

} // fin funcion main()
