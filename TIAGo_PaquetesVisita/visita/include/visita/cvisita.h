//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Óscar García Fernández
// Last update: 19.12.2024
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef CVISITA_H
#define CVISITA_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include "mensajes/RoomsVector.h"
#include "mensajes/Pose2DStamped.h"
#include "mensajes/PlacesVector.h"

#include "std_msgs/Float64.h"

#include "actionlib_msgs/GoalStatusArray.h"   
#include "actionlib_msgs/GoalStatus.h"         



#define PLACE_NAME_TARGET    "place_name_target"
#define ROOM_NAME_TARGET     "room_name_target"
#define PLACES_REQUESTED     "places_requested"
#define PLACES_IN_ROOM       "places_in_room"
#define CORRECT_ROOM_NAME    "correct_room_name"
#define CORRECT_PLACE_NAME   "correct_place_name"
#define ALL_PLACES           "all_places"
#define GOAL_POSE            "goal_pose"
#define STOP_REQUEST         "stop_request"
#define GOAL_STATUS          "goal_status"

namespace visita {

class cvisita {
public:
	cvisita();
	~cvisita();
	void open(ros::NodeHandle &n);
	void Run(const std::string& idioma_run);
    
	void publish_dialog(const std::string& dialog , const std::string& idioma_dialog);
    
	void publish_place(const std::string& room, const std::string& place);
    
	void load_visit(const std::string& filename);
	
	void PublicarIdioma(const std::string& idioma);
	
    
	bool has_more_places() const;

	std::vector<std::string> rooms_visit;
	std::vector<std::string> places_visit;
	size_t current_place_index = 0;
	
	bool text_tts_playing = false;   // El valor de este atributo nos dice si hay texto reproduciendose (true) o ya ha terminado de reproducirse (false)
	
    
private:
    
	std::string trim_end(const std::string& str);
    
	std::string load_dialog(const std::string& base_path_dialog, const std::string& place);  
    
	
	// funciones callback
	void goal_status_callback(const std_msgs::String::ConstPtr& msg);
	
	void tts_status_callback(const actionlib_msgs::GoalStatusArray msg2);    
	
	
    
	bool bRun;
	bool opened;
	bool finished; 
	bool goal_reached;
	bool goal_sent;
	
	// subscribers
    
	ros::Subscriber goal_status_sub;
	
	ros::Subscriber tts_status_sub;    
	
    
	// publishers
  
	ros::Publisher tts_pub;
	ros::Publisher room_name_pub;
	ros::Publisher place_name_pub;
	
	ros::Publisher language_pub;
	
	ros::Publisher visita_status_pub;   
};

} // namespace visita

#endif // CVISITA_H

