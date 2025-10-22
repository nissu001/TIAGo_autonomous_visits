//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Marcos Barranco Montilla
// Last update: 20.03.2025
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// archivo con la funcion main() del nodo measurements_visita , donde se realiza el bucle infinito o loop (con frecuencia de 10 Hz = 0.1 segundos)


// includes (dependencias) :

#include "cmeasurements_visita.h"




// Funcion main() :  

int main(int argc, char **argv){
   
   
   ros::init(argc, argv, "measurements_visita");    // creo e inicializo el nodo "measurements_visita"  (nombre del nodo que aparece en rostopic list si lo inicializo con rosrun)
   ros::NodeHandle n;                               // creo un NodeHandle n (solo 1 para todos los topics)
   
   msr::cMeasurementsVisita my_measurements;    /*creo un objeto "my_measurements" de clase msr::cMeasurementsVisita, con lo que se crean todos
                                                  los metodos y atributos de la clase para el objeto */
   
   
   
   my_measurements.Topics(n);    // Llamo al metodo cMeasurementsVisita::Topics() del objeto "my_measurements" y le mando como argumento el NodeHandle n que acabo de crear.
                                 // Este metodo define los Publishers y Subscribers del nodo
                             
                                 /* no hay que usar el namespace msr:: para llamar al metodo del objeto creado.
                                    Esto es porque ya lo hemos usado para crear el objeto.*/
   
   
   
   ROS_INFO("Ready to run");
      
   ros::Rate loop_rate(10);      // establezco la frecuencia del Loop o Bucle infinito = 10 Hz (0.1 s)
   
   ROS_INFO("Init loop");
   
   
   // loop (bucle infinito que se repite con una frecuencia de 10 Hz = 0.1 s hasta que se "mata" el nodo. Por tanto las callbacks se ejecutan cada 0.1 s ) :
   
   while (ros::ok()) {
   
      ros::spinOnce();         // se llama a las callbacks de los topics suscriptores 1 sola vez (cada 0.1 s = 10Hz)
      loop_rate.sleep();       // Pausa para mantener la frecuencia de repetición deseada del loop (10 Hz = 0.1 s)
      
      
      my_measurements.Run();   // ejecuto el metodo cMeasurementsVisita::Run() del objeto "my_measurements"
   
   
   } // fin del loop (sale cuando se "mata" el nodo)                                                                                     



return 0;

} // fin funcion main()

