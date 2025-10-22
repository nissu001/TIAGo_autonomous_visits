#ifndef STRUCTS_H
#define STRUCTS_H

#include <stdint.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include "std_msgs/String.h"




struct Place {
    float x;
    float y;
    float theta;
    std::string place_name;
    std::string place_type;
    int id;

};

struct Point2D {
    float x;
    float y;

};


struct PlacesVector {
    double time;
    std::vector<Place> places_vector;
};

struct Room {
    double time;
    std::string room_name;
    float x_ref;
    float y_ref;
    float theta_ref;
    std::vector<Place> places_vector;
    std::vector<Point2D> vertices_vector;
};

struct RoomsVector {
    double time;
    std::vector<Room> rooms_vector;
};

extern RoomsVector known_rooms;
#endif // STRUCTS_H
