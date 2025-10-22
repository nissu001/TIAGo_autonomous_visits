//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 14.1.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <string.h>
#include "../include/PlacesInterpretation/cPlacesInterpretation.h"
#include "pugixml.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "structs.h"


namespace PlacesInterpretation {

// Opens the file with the list of initial known places and stores them.
bool cPlacesInterpretation::LoadFile(std::string placesFileName) {
    // Open the file.
    pugi::xml_document doc;
    std::cout << "string " << placesFileName << std::endl;
    std::ifstream nameFile(placesFileName.c_str());
    if (!nameFile) {
        std::cerr << "Failed to open file: " << placesFileName << std::endl;
        return false;
    }
    pugi::xml_parse_result result = doc.load(nameFile);
    if (!result) {
        std::cerr << "Xml places file missing " << std::endl;
        return false;
    }
   
    pugi::xml_node rooms = doc.child("rooms");

    // known rooms 
    for (pugi::xml_node room = rooms.child("room"); room; room = room.next_sibling("room")) {
    
        mensajes::Room room_msg;

        // room name
        std::string room_name = room.child_value("name");
        room_msg.room_name = room_name;

        // Places
        pugi::xml_node places = room.child("places");
        for (pugi::xml_node place = places.child("place"); place; place = place.next_sibling("place")) {
           mensajes::Place place_msg;

            std::string place_name = place.child_value("name");
            place_msg.place_name = place_name;

            pugi::xml_node pose = place.child("pose");

            place_msg.x = atof(pose.attribute("x").value());
            place_msg.y = atof(pose.attribute("y").value());
            place_msg.theta = atof(pose.attribute("theta").value());

            room_msg.places_vector.push_back(place_msg);
        }
        
        known_rooms.rooms_vector.push_back(room_msg);
    }

    std::cout << "num rooms " << known_rooms.rooms_vector.size() << std::endl;

    all_places.header.frame_id = "map";
    all_places.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < known_rooms.rooms_vector.size(); i++) {
        for (unsigned int j = 0; j < known_rooms.rooms_vector[i].places_vector.size(); j++) {
            mensajes::Place place_ij = known_rooms.rooms_vector[i].places_vector[j];
            geometry_msgs::Pose p;
            p.position.x = place_ij.x;
            p.position.y = place_ij.y;
            p.orientation.x = 0;
            p.orientation.y = 0;
            p.orientation.z = sin(0.5 * place_ij.theta);
            p.orientation.w = cos(0.5 * place_ij.theta);

            all_places.poses.push_back(p);
        }
    }

    return true;
}

// Opening and initializing the node
void cPlacesInterpretation::open(ros::NodeHandle& n) {
    //// Topics ///

    //////subscribers   
        
    // Subscriber for going to a place
    goToPlaceSubs = n.subscribe<std_msgs::String>(PLACE_NAME_TARGET, 1, &cPlacesInterpretation::goToPlaceCallback, this);
                                                                
    // Subscriber for obtaining the room name
    getRoomNameSubs = n.subscribe<std_msgs::String>(ROOM_NAME_TARGET, 1, &cPlacesInterpretation::getRoomNameCallback, this);

    // Subscriber for the places request
    getPlacesSubs = n.subscribe<std_msgs::String>(PLACES_REQUESTED, 1, &cPlacesInterpretation::getPlacesCallback, this);

    //////Publishers  

    // Publisher for the goal                                                  
    goalPubl = n.advertise<mensajes::Pose2DStamped>(PLACE_GOAL, 1);
    // Publisher for the places in a room 
    placesPubl = n.advertise<mensajes::PlacesVector>(PLACES_IN_ROOM, 1);
    // Publishing if the room name is correct
    recognizedRoomPubl = n.advertise<std_msgs::Bool>(CORRECT_ROOM_NAME, 1);
    // Publishing if the place name is correct
    recognizedPlacePubl = n.advertise<std_msgs::Bool>(CORRECT_PLACE_NAME, 1);
    // Publisher for all the places
    knownPlacesPubl = n.advertise<geometry_msgs::PoseArray>(ALL_PLACES, 1);

    opened = true;
}

// ROS callback function for the reception of target place name
void cPlacesInterpretation::goToPlaceCallback(const std_msgs::String::ConstPtr& msg) {
    // examine the list of available known places and obtain the pose corresponding to the given place name, if it is correct
    bool correct_place_name = false;
    bool current_room_found = false;

    std::cout << "go to place " << msg->data << std::endl;

    for (unsigned int i = 0; i < known_rooms.rooms_vector.size(); i++) {
        if (known_rooms.rooms_vector[i].room_name.compare(current_room) == 0) {
            current_room_found = true;
            std::cout << "room found " << current_room << std::endl;
            for (unsigned int j = 0; j < known_rooms.rooms_vector[i].places_vector.size(); j++) {
                if (known_rooms.rooms_vector[i].places_vector[j].place_name == msg->data) {
                    current_place = msg->data;

                    goal.x = known_rooms.rooms_vector[i].places_vector[j].x;
                    goal.y = known_rooms.rooms_vector[i].places_vector[j].y;
                    goal.theta = known_rooms.rooms_vector[i].places_vector[j].theta;

                    std::cout << "goal pose " << goal.x << " " << goal.y << " " << goal.theta << std::endl;

                    correct_place_name = true;
                    new_place = true;
                }
            }
        }
    }

    room_isknown.data = current_room_found;
    recognizedRoomPubl.publish(room_isknown);

    place_isknown.data = correct_place_name;
    recognizedPlacePubl.publish(place_isknown);
}

// ROS callback function for the reception of the room name
void cPlacesInterpretation::getRoomNameCallback(const std_msgs::String::ConstPtr& msg) {
    current_room = msg->data;
    std::cout << "go to room " << current_room << std::endl;

    bool current_room_found = false;

    for (unsigned int i = 0; i < known_rooms.rooms_vector.size(); i++) {
        if (known_rooms.rooms_vector[i].room_name.compare(current_room) == 0) {
            current_room_found = true;
        }
    }

    room_isknown.data = current_room_found;
    recognizedRoomPubl.publish(room_isknown);
}

// ROS callback function for the reception of the places in the current room
void cPlacesInterpretation::getPlacesCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data.compare("Request") == 0) {
        bool publish_places = false;
        for (unsigned int i = 0; i < known_rooms.rooms_vector.size(); i++) {
            if (known_rooms.rooms_vector[i].room_name.compare(current_room) == 0) {
                places.places_vector = known_rooms.rooms_vector[i].places_vector;
                publish_places = true;
                break;
            }
        }
        if (publish_places) {
            placesPubl.publish(places);
        }
    }
}

/*
bool cPlacesInterpretation::getRooms(servicios::GetRooms::Request  &req, servicios::GetRooms::Response &res)
 {
	
	ROS_INFO("Rooms request received");

	res.rooms = known_rooms;

	ROS_INFO("sending back rooms response");
//	ROS_INFO("number of rooms: [%d]", res.rooms.rooms_vector.size());


	return true;

 }

*/

// publication of the next goal corresponding to the given place
int cPlacesInterpretation::goalPublication(mensajes::Pose2DStamped g) {
    if (!opened)
        return 0;

    goalPubl.publish(g);
    return 1;
}

// Constructor. Initialises member attributes and allocates resources.
cPlacesInterpretation::cPlacesInterpretation(int argc, char** argv) {
    opened = false;
    bRun = true; //FIXME, set via keyboard to start the node?
    new_place = false;
}

// Destructor. Shuts down ROS, ends the thread and released allocated resources.
cPlacesInterpretation::~cPlacesInterpretation() {
    printf("cPlacesInterpretation::~cPlacesInterpretation(): shutting down ROS\n");
    usleep(100000);
    if (ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
    usleep(100000);
    printf(" - done\n");
}

// ROS startup, resource allocation and ROS main loop.
void cPlacesInterpretation::Run(void) {
    if (bRun) {
        if (new_place) {
            goalPublication(goal);
            std::cout << "published new goal pose for place " << current_place << std::endl;
            new_place = false;
        }
        knownPlacesPubl.publish(all_places);
    } else {
        printf("cPlacesInterpretation::Run(): ignoring run command due to previous error\n");
    }
}

} // namespace PlacesInterpretation

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Publisher for the places
