//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 28.4.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "../include/PlacesLearningGui/qnode.hpp"


#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include "structs.h"
#include "pugixml.hpp"
#include <iostream>
#include <fstream>

using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of the room name
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void QNode::roomNameRequested()
{
	//std::cout << "room name requested" << std::endl;
		for (unsigned int i=0;i<known_rooms.rooms_vector.size();i++)
		{
			std::vector<Point2D> room_vertices;
			for (unsigned int j=0; j < known_rooms.rooms_vector[i].vertices_vector.size(); j++)
			{
				Point2D vertex;
				vertex.x = known_rooms.rooms_vector[i].vertices_vector[j].x;
				//std::cout << "vertex_x" << vertex.x << std::endl;
				vertex.y = known_rooms.rooms_vector[i].vertices_vector[j].y;
				//std::cout << "vertex_y" << vertex.y << std::endl;
				room_vertices.push_back(vertex);
				//std::cout << "current_x" << current_x << std::endl;
				//std::cout << "current_y" << current_y << std::endl;
				room_vertices.push_back(vertex);
			}
			if (isInRoom(current_x,current_y,room_vertices)) //The vertices MUST be ordered
			{
				current_room_name = known_rooms.rooms_vector[i].room_name;
				std::cout << "current room " << current_room_name<< std::endl;
				break;
			}		

		}
		publish_room_name = true;

}




bool QNode::isInRoom(float x_pos, float y_pos, std::vector<Point2D> polygon_points)
{
	//std::cout << "isInroom" << std::endl;
	//The points MUST be ordered either clockwise or counterclockwise
	//int num_intersections = 0;

//Implementation of the crossing number algorithm. Inspired by the original article in Communications of the ACM by M. Shimrat and by the pseudocode provided by D. Eppstein, with some improvements. Points laying on the edges or the vertices are also considered.
	bool b = false;
	for (unsigned int i = 0; i < polygon_points.size()-1; i++)
	{
		Point2D vertex = polygon_points[i];
		Point2D next_vertex = polygon_points[i+1];
		if ((vertex.x < x_pos && x_pos < next_vertex.x) || (vertex.x > x_pos && x_pos > next_vertex.x))
		{
		    float xr = x_pos;
		    float yr = ((next_vertex.y-vertex.y)*(xr-vertex.x)/(next_vertex.x-vertex.x))+vertex.y;
		    if (yr == y_pos) 
			return true;
		    else 
			if (yr < y_pos) 
				b = !b; //num_intersections++;
		}
		if (vertex.x == x_pos && vertex.y <= y_pos) 
		{
		    if (vertex.y == y_pos) 
			return true;

		    if (next_vertex.x == x_pos)
		    {
			if (y_pos <= next_vertex.y)
			    return true;
		    } 
		    else 
			if (next_vertex.x > x_pos) b=!b; //num_intersections++;

		}
	}

	//last segment checking
	int n = polygon_points.size();
	if ((polygon_points[n-1].x < x_pos && x_pos < polygon_points[0].x) || (polygon_points[n-1].x > x_pos && x_pos > polygon_points[0].x))
	{
		float yr_last = (polygon_points[0].y-polygon_points[n-1].y)*(x_pos-polygon_points[n-1].x)/(polygon_points[0].x-polygon_points[n-1].x)+polygon_points[n-1].y;

		if (yr_last == y_pos) 
			return true;
		else if (yr_last < y_pos) 
			b = !b; //num_intersections++;
	}

	if (polygon_points[n-1].x == x_pos && polygon_points[n-1].y <= y_pos) 
	{
            //std::cout << "last case " << std::endl;
	    if (polygon_points[n-1].y == y_pos) //point is a vertex of the polygon
		return true;

	    if (polygon_points[0].x == x_pos)  //point lies on one side of the polygon
	    {
		if (y_pos <= polygon_points[0].y)
		    return true;
	    } 
	    else 
		if (polygon_points[0].x < x_pos) b=!b; //num_intersections++;

	}

	//std::cout << "num intersect " << num_intersections << std::endl;

	/*if (num_intersections % 2== 0 )
		return false;
	else 
		return true;*/

	return b;

}



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// function for the reception of new places, update
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::learnPlace(const std::string place_name, const std::string place_type)
{

	//cout << "current_x " << current_x << endl;
        //cout << "current_y " << current_y << endl;
	cout << "current_th " << current_theta*180/M_PI << endl;

        std_msgs::String req_msg;
	req_msg.data = "Request";
	currentRoomNameRequestPubl.publish(req_msg);
	roomNameRequested();
	cout << "sala actual"<< current_room_name<<endl;
        sleep(5);

	Place new_place;
	new_place.x = current_x;
	new_place.y = current_y;
	new_place.theta = current_theta;
	new_place.place_name = place_name.c_str();
	new_place.place_type = place_type.c_str();
	
	bool places_in_room = false;
	
	// add new place to the current room if any other places have already been added to the room
	for (unsigned int i=0;i<rooms.rooms_vector.size();i++)
	{
		std::string room_n = rooms.rooms_vector[i].room_name;
		if (current_room_name.compare(room_n) == 0)
		{
			std::cout << "Adding place to room " << current_room_name << std::endl;
			rooms.rooms_vector[i].places_vector.push_back(new_place);
			places_in_room = true;
			break;
		}
	}

	// initialize and add the current room if no other places have already been added to the room
	if(!places_in_room)
	{
		Room new_room;
		new_room.room_name = current_room_name;
		new_room.places_vector.push_back(new_place);	
		rooms.rooms_vector.push_back(new_room);	
		std::cout << "Adding new room " << current_room_name << std::endl;
	}

}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ROS callback function for the reception of the pose
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void QNode::getPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//void QNode::getPoseCallback(const geometry_msgs::pose2D::ConstPtr& msg)
{
	//should be updated with high frequency
	current_x = msg->pose.pose.position.x;
	current_y = msg->pose.pose.position.y;
	current_theta = tf::getYaw(msg->pose.pose.orientation);
	
	cout << "current pose " << current_x << " " << current_y << endl;
	//cout << "current pose " << current_x << " " << current_y << " " << current_theta*180/M_PI << endl;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Constructor of the class - allocated and configures resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv)
{
  opened = false;
  bRun = false; 
  publish_room_name = false;

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Destructor of the class - waits for the ROS node to shut down and
// releases the allocated resources.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
QNode::~QNode()
{
  printf("cPlacesLearningGui::~cPlacesLearningGui(): shutting down ROS\n");
  usleep(100000);
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  usleep(100000);
  printf(" - done\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Function that just starts the thread function.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::init()
{

// Start the thread function.Fcurre
  printf("Starting subscriber thread...\n");
  start();
  return true;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Thread function - this is where the ROS main loop runs.
//++++++++++++++++++++++++++++++++++++++++++++++F+++++++++++++++++++++
void QNode::run()
{
  if (bRun)
  {

    //cout << "here" << endl;

    // Init the ROS stuff and create subscribers and a publisher.
    ros::init(init_argc, init_argv, "PlacesLearningGui");
    ros::NodeHandle n;

    //////subscribers   

    // Subscriber for the current room name
    //currentRoomSubs = n.subscribe<std_msgs::String>(CURRENT_ROOM, 1, &QNode::currentRoomCallback, this);
    //std::cout << "subs " << currentRoomSubs.getNumPublishers() << " - " <<  currentRoomSubs.getTopic()<<std::endl;                                                            
    // Subscriber for the pose                                                             
    getPoseSubs = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(ESTIMATED_POSE, 1, &QNode::getPoseCallback, this);
    cout<<"get pose"<< endl;
    //getPoseSubs = n.subscribe<geometry_msgs::pose2D>(ESTIMATED_POSE, 1, &QNode::getPoseCallback, this);

   // Publisher to request the current room name
    currentRoomNameRequestPubl = n.advertise<std_msgs::String>(CURRENT_ROOM_REQUEST, 1);
    //currentRoomNameRequestPubl = n.advertise<std_msgs::String>("room_name_requested", 1);

    opened = true;
    if (publish_room_name)
	{
		current_room=current_room_name;
		publish_room_name = false;
	}
    // ROS main loop.
    ros::spin();

  }
  else
  {
	printf("cPlacesLearningGui::Run(): ignoring run command due to previous error\n");

  }

// Send the shutdown signal.
  printf("Ros shutdown, proceeding to close the gui.\n");
  Q_EMIT rosShutdown();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Save places to XML file
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool QNode::savePlaces(std::string fileName)
{

    //cout << "here" << endl;
    if (rooms.rooms_vector.size()==0)
    {
 		cout << "No places to be saved " << endl;
 		return false;
 
    }
	 

    //open the file

    pugi::xml_document doc;
    std::ifstream nameFile(fileName.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if (!result) //FIXME, pass as argument
    {
        std::cout << "Xml places file missing " << std::endl;
        //assert(0);
        return false;
    
    }

    //cout << "file open " << endl;

    /*****************************************************************************/

    // the rooms node
    pugi::xml_node rooms_node = doc.child("rooms");
    
    
    for (unsigned int i=0;i<rooms.rooms_vector.size();i++)
{
	// room name of places to be added
	string room_name = rooms.rooms_vector[i].room_name;

	//cout << "Room name of places to be added " << room_name << endl;
	
	//find node in file
	for(pugi::xml_node room = rooms_node.child("room"); room; room = room.next_sibling("room"))
	{
		std::string room_name_file = room.child_value("name");
		if(room_name.compare(room_name_file) == 0)
		{
			//cout << "Room found " << room_name << endl;
			if (!rooms.rooms_vector[i].places_vector.size() > 0) continue;
			
			// add places
			pugi::xml_node places_i = room.append_child();
			places_i.set_name("places");

			//add place_i_0
			pugi::xml_node place_i_0 = places_i.append_child();
			place_i_0.set_name("place");
			
			// add place name
			pugi::xml_node place_name_i_0 = place_i_0.append_child();
			place_name_i_0.set_name("name");
			string place_i_0_name = rooms.rooms_vector[i].places_vector[0].place_name;
			place_name_i_0.append_child(pugi::node_pcdata).set_value(place_i_0_name.c_str());

			// add place type
			pugi::xml_node place_type_i_0 = place_i_0.append_child();
			place_type_i_0.set_name("type");
			string place_i_0_type = rooms.rooms_vector[i].places_vector[0].place_type;
			place_type_i_0.append_child(pugi::node_pcdata).set_value(place_i_0_type.c_str());
			
			// add pose
			pugi::xml_node pose_i_0 = place_i_0.append_child();
			pose_i_0.set_name("pose");
			
			// add attributes to pose
			float place_i_0_x = rooms.rooms_vector[i].places_vector[0].x;
			char place_i_0_x_converted[1000];
			snprintf(place_i_0_x_converted, sizeof(place_i_0_x_converted), "%.6f", place_i_0_x);
			pose_i_0.append_attribute("x") = place_i_0_x_converted;
			
			float place_i_0_y = rooms.rooms_vector[i].places_vector[0].y;
			char place_i_0_y_converted[1000];
			snprintf(place_i_0_y_converted, sizeof(place_i_0_y_converted), "%.6f", place_i_0_y);
			pose_i_0.append_attribute("y") = place_i_0_y_converted;
			
			float place_i_0_theta = rooms.rooms_vector[i].places_vector[0].theta;
			char place_i_0_theta_converted[1000];
			snprintf(place_i_0_theta_converted, sizeof(place_i_0_theta_converted), "%.6f", place_i_0_theta);
			pose_i_0.append_attribute("theta") = place_i_0_theta_converted;

			// add other places
			pugi::xml_node prev_place_i = place_i_0;

			for (unsigned int j = 1; j < rooms.rooms_vector[i].places_vector.size(); j++)
			{
				pugi::xml_node place_i_j = places_i.insert_child_after(pugi::node_element, prev_place_i);
				place_i_j.set_name("place");

				// add place name
				pugi::xml_node place_name_i_j = place_i_j.append_child();
				place_name_i_j.set_name("name");
				string place_i_j_name = rooms.rooms_vector[i].places_vector[j].place_name;
				place_name_i_j.append_child(pugi::node_pcdata).set_value(place_i_j_name.c_str());

				// add place type
				pugi::xml_node place_type_i_j = place_i_j.append_child();
				place_type_i_j.set_name("type");
				string place_i_j_type = rooms.rooms_vector[i].places_vector[j].place_type;
				place_type_i_j.append_child(pugi::node_pcdata).set_value(place_i_j_type.c_str());

				// add pose
				pugi::xml_node pose_i_j = place_i_j.append_child();
				pose_i_j.set_name("pose");
				
				// add attributes to pose
				float places_i_j_x = rooms.rooms_vector[i].places_vector[j].x;
				char places_i_j_x_converted[1000];
				snprintf(places_i_j_x_converted, sizeof(places_i_j_x_converted), "%.6f", places_i_j_x);
				pose_i_j.append_attribute("x") = places_i_j_x_converted;
				
				float places_i_j_y = rooms.rooms_vector[i].places_vector[j].y;
				char places_i_j_y_converted[1000];
				snprintf(places_i_j_y_converted, sizeof(places_i_j_y_converted), "%.6f", places_i_j_y);
				pose_i_j.append_attribute("y") = places_i_j_y_converted;
				
				float places_i_j_theta = rooms.rooms_vector[i].places_vector[j].theta;
				char places_i_j_theta_converted[1000];
				snprintf(places_i_j_theta_converted, sizeof(places_i_j_theta_converted), "%.6f", places_i_j_theta);
				pose_i_j.append_attribute("theta") = places_i_j_theta_converted;

				prev_place_i = place_i_j;
			} // end places in room
		} // end this is the room
	} // end rooms in file
}// end rooms with places
    
	 
    //std::cout << "Saving places: " << doc.save_file(fileName.c_str()) << std::endl;
    //return true;
    
    return doc.save_file(fileName.c_str());
  
  
}


