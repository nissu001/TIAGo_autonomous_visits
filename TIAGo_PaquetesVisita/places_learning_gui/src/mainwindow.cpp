#include "../include/PlacesLearningGui/mainwindow.h"
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QGroupBox>
#include <QRadioButton>
#include <iostream>
#include <fstream>
#include "structs.h"
#include "../include/PlacesLearningGui/qnode.hpp"

#include "pugixml.hpp"
#include "../include/PlacesLearningGui/mydialog.h"

RoomsVector known_rooms;

using namespace std;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)  :
    QMainWindow(parent),
    ui(new Ui::MainWindow), qnode(argc, argv)
{
    ui->setupUi(this);
    bConnected = false;

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    ui->label->current_x = &(qnode.current_x);  
    ui->label->current_y = &(qnode.current_y);
    ui->label->current_theta = &(qnode.current_theta);

    ui->label->rooms = &qnode.rooms;

    ui->label->map_width_relation = 1;
    ui->label->map_height_relation = 1;
    ui->label->map_resolution = 1;
    ui->label->map_origin_x = 0;
    ui->label->map_origin_y = 0;
    ui->label->input_map_height = 400;
   
 
    num = 1;

}

MainWindow::~MainWindow()
{
    delete ui;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Load Map function to open the map file, display the image and get the map parameters
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float MainWindow::stringToFloat(const std::string& str) {
    float result = 0.0f;
    float fraction = 0.1f; // Usado para los decimales
    bool negative = false;
    bool decimal = false;
    int digitCount = 0;

    for (char c : str) {
        if (c == '-') {
            negative = true;
        } else if (c == '.') {
            decimal = true;
        } else if (std::isdigit(c)) {
            if (decimal) {
                result += (c - '0') * fraction;
                fraction *= 0.1f;
            } else {
                result = result * 10.0f + (c - '0');
                digitCount++;
            }
        } else {
            throw std::invalid_argument("Carácter no numérico encontrado");
        }
    }

    if (digitCount == 0) {
        throw std::invalid_argument("No se encontraron dígitos numéricos");
    }

    if (negative) {
        result *= -1.0f;
    }

    return result;
}


std::string MainWindow::replaceCommasWithDots(const std::string& str) {
    std::string result = str;
    std::replace(result.begin(), result.end(), ',', '.');
    return result;
}


void MainWindow::LoadMap()
{
    //
    map_file_name = QFileDialog::getOpenFileName(this,tr("Open map image"), "/home", tr("Image files (*.pgm)"));

    if (!map_file_name.isEmpty())
    {

            std::string map_file_name_str = map_file_name.toStdString();
            std::cout << "Map name " << map_file_name_str << std::endl;

            // open image from file
            QImage image(map_file_name);
            if (image.isNull())
            {
                QMessageBox::information(this, tr("Map loading"), tr("Cannot load %1.").arg(map_file_name));
                return;
            }

            //tell the label in the ui that the map is loaded
            ui->label->map_opened = true;

            //obtain label sizes
            float width = ui->label->width();
            cout << "width:" << width << endl; 
            float height = ui->label->height();

            //resize the input image to the size of the label, keeping the same aspect ratio
            QImage resized_image = image.scaled (width, height, Qt::KeepAspectRatio, Qt::FastTransformation);

            //show the image on the label
            ui->label->setPixmap(QPixmap::fromImage(resized_image));

            //obtain the map_size_relations, needed for the coordinates convertion from pixels to map coordinates
            map_width_relation = image.width()/width;
            //la relación de mapa está adaptado al ancho del mapa actual
            cout << "image width:" << image.width() << endl;
            //map_width_relation = 3.95;
            map_height_relation = image.height()/height;
            input_map_height = image.height();
            

	    cout << "map_width_relation " << map_width_relation << endl;
	    cout << "map_height_relation " << map_height_relation << endl;
	    cout << "input_map_height " << input_map_height << endl;

            //open .yaml file to obtain other parameters required for the coordinates convertion from pixels to map coordinates
            int ind = map_file_name_str.find_last_of(".");
            std::string fname = map_file_name_str.substr(0,ind);
            std::string conf_file_name_str = fname + ".yaml";

            std::cout << "Map conf file name " << conf_file_name_str << std::endl;
            QFile file(conf_file_name_str.c_str());

            //if the .yaml file is not found in the same directory as the map image, output message
            if(!file.open(QIODevice::ReadOnly))
            {
                QMessageBox msgBox;
                msgBox.setText(".yaml map configuration file missing!");
                msgBox.exec();
            }

            // reading the .yaml file
            QStringList stringList;
            QTextStream textStream(&file);
            while (true)
            {
                QString line = textStream.readLine();
                if (line.isNull())
                    break;
                else
                    stringList.append(line);
            }


            int count = stringList.count();
            for(int i=0;i<count;i++)
            {
                QString str = stringList.at(i);

                //cout << "string " << str.toStdString() << endl;

                std::string str_std = str.toStdString();

                if(str_std.find("resolution: ") != std::string::npos)
                {
                    string subs = str_std.substr(12); //FIXME
                    map_resolution = stringToFloat(subs);
                    cout << "Map resolution " << map_resolution << endl;
                }

                if(str_std.find("origin: ") != std::string::npos)
                {
                    int ind0 = string("origin: ").size(); //length of "origin: "
                    int ind1 = str_std.find_first_of(","); //find first comma
                    int len1 = ind1 - ind0; //length of x substring

                    string subs_x = str_std.substr(9,len1-1);
                    map_origin_x = stringToFloat(subs_x.c_str());
                    cout << "Origin x " << map_origin_x << endl;

                    int ind2 = str_std.find_last_of(","); //find second comma
                    int len2 = ind2 - ind1; //length of y substring

                    string subs_y = str_std.substr(ind1+2,len2-2); //string between two commas
                    map_origin_y = stringToFloat(subs_y);
                    cout << "Origin y " << map_origin_y << endl;

                }


            }

    	    ui->label->map_width_relation = map_width_relation;
	    ui->label->map_height_relation = map_height_relation;
            ui->label->map_resolution = map_resolution;
	    ui->label->input_map_height = input_map_height;
            ui->label->map_origin_x = map_origin_x;
            ui->label->map_origin_y = map_origin_y;

    }


}

bool MainWindow::LoadFile(std::string placesFileName)
{

    pugi::xml_document doc;
    std::ifstream nameFile(placesFileName.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

   if (!result) //FIXME, pass as argument
   {
        std::cout << "Xml places file missing " << std::endl;
        //assert(0);
        return false;
    
   }
   
   
   pugi::xml_node rooms = doc.child("rooms");

	// known rooms 
	
    for(pugi::xml_node room = rooms.child("room"); room; room = room.next_sibling("room"))
    {  
    	  Room room_msg;

	//room name
        std::string room_name = room.child_value("name");
        room_msg.room_name = room_name;
        std::cout << "Room name: " << room_name << std::endl;
        
         //Vertices
        pugi::xml_node vertices = room.child("vertices");
        for(pugi::xml_node vertex = vertices.child("vertex"); vertex; vertex = vertex.next_sibling("vertex"))
        {
	    Point2D vertex_msg;
	
	    /*std::string x_str = replaceCommasWithDots(vertex.attribute("x").value());
	    std::string y_str = replaceCommasWithDots(vertex.attribute("y").value());

            vertex_msg.x = atof(x_str.c_str());
            vertex_msg.y = atof(y_str.c_str());
            */
            vertex_msg.x = atof(vertex.attribute("x").value());
	    vertex_msg.y = atof(vertex.attribute("y").value());
           // std::cout << "vertex x " <<vertex_msg.x<< std::endl;
	   // std::cout << "vertex y " <<vertex_msg.y<< std::endl;

            room_msg.vertices_vector.push_back(vertex_msg);
        }
        
        
        qnode.rooms.rooms_vector.push_back(room_msg);
        known_rooms.rooms_vector.push_back(room_msg);
  }

  std::cout << "num rooms " << qnode.rooms.rooms_vector.size() << std::endl;

  return true;

}



float MainWindow::transform_x(float x)
{

    float map_x = map_origin_x + x*map_width_relation*map_resolution;
    return map_x;

}

float MainWindow::transform_y(float y)
{
    float map_y = map_origin_y + input_map_height*map_resolution - y*map_height_relation*map_resolution;
    return map_y;


}

void MainWindow::on_connect_clicked()
{

// Start the ROS worker thread.
  if (!bConnected)
  {
    qnode.bRun = true;
    qnode.init();
    bConnected = true;

    map_file_name = QFileDialog::getOpenFileName(this,tr("Open rooms file"), "/home", tr("All files (*)"));

    if (!map_file_name.isEmpty())
    {

            std::string map_file_name_str = map_file_name.toStdString();
            LoadFile(map_file_name_str);

    }

    else
	QMessageBox::information(this, tr("Rooms loading"), tr("Cannot open places.xml file"));
  }

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Load Map button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::on_load_map_clicked()
{
    LoadMap();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Save room button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::on_save_place_clicked()
{
    bool ok;  
    QString text_input = QInputDialog::getText(this,("Enter place name"),(""),QLineEdit::Normal,QString::null,&ok);
    std::cout << "Place name " << text_input.toStdString() << std::endl;

    MyDialog *subDialog = new MyDialog;
    subDialog->setWindowTitle("Sub Dialog");
    subDialog->show(); 
    subDialog->n = num;
    num++;

    subDialog->qnode = &qnode;
    std::string current_place_name = text_input.toStdString();
    subDialog->current_place_name = current_place_name;

    /*float x,y,th;
    //FIXME!! get pose from qnode
    MyQLabel::Place place(current_place_name, x, y, th);
    ui->label->places.push_back(place);*/

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Delete room button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::on_delete_place_clicked()
{
    //Open dialog for the user to enter the room name
    bool ok;
    QString text_input = QInputDialog::getText(this,("Enter room name"),(""),QLineEdit::Normal,QString::null,&ok);

    std::string room_to_be_deleted = text_input.toStdString();

    bool room_found = false;
    bool place_found = false;

    //look for the room name among the existing rooms
    for(unsigned int i=0; i<(*ui->label->rooms).rooms_vector.size();i++)
    {
        std::string room_name = (*ui->label->rooms).rooms_vector[i].room_name;
        if (room_name.compare(room_to_be_deleted) == 0)
        {
            room_found = true;
	    //cout << "room to be deleted found " << room_name << endl;

	    QString text_input_place = QInputDialog::getText(this,("Enter place name"),(""),QLineEdit::Normal,QString::null,&ok);
	    std::string place_to_be_deleted = text_input_place.toStdString();
            //if room is found look for the place name among the existing places
	    for(unsigned int j=0; j<(*ui->label->rooms).rooms_vector[i].places_vector.size();j++)
	    {
            	std::string place_name = (*ui->label->rooms).rooms_vector[i].places_vector[j].place_name;
        	if (place_name.compare(place_to_be_deleted) == 0)
		{
			(*ui->label->rooms).rooms_vector[i].places_vector.erase((*ui->label->rooms).rooms_vector[i].places_vector.begin()+j);
			place_found = true;
		  	//cout << "place to be deleted found " << place_name << endl;
			break;
		}
	    }
	    if (place_found)
		break;
        }

    }

    //if the requested room is not found, give output message so that any spelling errors can be corrected
    if(!room_found)
    {
        QMessageBox msgBox;
        msgBox.setText("That room does not exist");
        msgBox.exec();
	return;
    }

    if(!place_found)
    {
        QMessageBox msgBox;
        msgBox.setText("The place was not found within the given room");
        msgBox.exec();
	return;
    }

    {
        QMessageBox msgBox;
        msgBox.setText("The place was deleted");
        msgBox.exec();
	return;
    }

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Save file button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int MainWindow::on_save_file_clicked()
{

    // if no rooms were saved, show output message
    /*if (ui->label->places.size()==0)
    {
        //std::cout << "No places to be saved " << std::endl;
        QMessageBox msgBox;
        msgBox.setText("No places to be saved");
        msgBox.exec();
        return false;

    }*/

    // open dialog for the user to enter the file name
    QString file_name = QFileDialog::getSaveFileName(this,
             tr("Save to file"), "",
             tr("All Files (*)"));

    string file_name_str = file_name.toStdString();

    //create pugixml document to do the parsing to a xml file
    
    if(!qnode.savePlaces(file_name_str))
  		cout << "Places file was not saved " << endl;

    return 0;

}







