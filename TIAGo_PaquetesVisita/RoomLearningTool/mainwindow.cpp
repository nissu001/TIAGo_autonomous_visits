#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QTextStream>
#include <iostream>
#include <iomanip>
#include <string>

#include "pugixml.hpp"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    copy_files = true;
    maps_path = "opt/ros/hobbit_hydro/src/navigation/share/";

    create_sym_link = true;
    sym_link = "/opt/ros/hobbit_hydro/src/navigation/sym_link.sh ";  

    convert_maps = true; 

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


void MainWindow::LoadMap()
{
    //
    map_file_name = QFileDialog::getOpenFileName(this,tr("Open map image"), "/opt/ros/hobbit_hydro/src/navigation/share", tr("Image files (*.pgm)"));

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
            width = ui->label->width();
	    cout << "width " << width << endl;
            height = ui->label->height();
    	    cout << "height " << height << endl;

            //resize the input image to the size of the label, keeping the same aspect ratio
            QImage resized_image = image.scaled (width, height, Qt::KeepAspectRatio, Qt::FastTransformation);

	    float max_dim = max(image.width(), image.height());
	    if (max_dim == image.width())
	    {
		new_width = width;
		new_height = image.height()*width/image.width();
		cout << "new_width " << new_width << endl;
		cout << "new_height " << new_height << endl;
	    }
	    if (max_dim == image.height())
	    {
		new_width = image.width()*height/image.height();
		new_height = height;
		cout << "new_width " << new_width << endl;
		cout << "new_height " << new_height << endl;
	    }

            //show the image on the label
            ui->label->setPixmap(QPixmap::fromImage(resized_image));

            //obtain the map_size_relation, needed for the coordinates convertion from pixels to map coordinates
            map_width_relation = image.width()/width;
            map_height_relation = image.height()/height;
            input_map_height = image.height();

	    cout << "map_width_relation " << map_width_relation << endl;
	    cout << "map_height_relation " << map_height_relation << endl;

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
                    cout << str_std<< endl;
                    std::string subs = str_std.substr(12); //FIXME
                    cout << subs << endl;
                    cout << subs.c_str() << endl;
                    subs = "0.02500";
                    //map_resolution = std::stof(subs);
                    map_resolution = stringToFloat(subs);
                    cout << setprecision(5);
                    cout << "Map resolution " << map_resolution << endl;


                }

                if(str_std.find("origin: ") != std::string::npos)
                {
                    int ind0 = string("origin: ").size(); //length of "origin: "
                    int ind1 = str_std.find_first_of(","); //find first comma
                    int len1 = ind1 - ind0; //length of x substring

                    string subs_x = str_std.substr(9,len1-1);
                    //cout <<"subs_x " << subs_x << endl;
                    map_origin_x = atof(subs_x.c_str());
                    cout << "Origin x " << map_origin_x << endl;

                    int ind2 = str_std.find_last_of(","); //find second comma
                    int len2 = ind2 - ind1; //length of y substring

                    string subs_y = str_std.substr(ind1+1,len2-1); //string between two commas
                    //cout <<"subs_y " << subs_y << endl;
                    map_origin_y = atof(subs_y.c_str());
                    cout << "Origin y " << map_origin_y << endl;

                }


            }

	    if (copy_files) //FIXME
	    {
		    std::string cp_command1("cp "); 
		    cp_command1 += map_file_name_str.c_str();
		    cp_command1 += maps_path;
		    cp_command1 += " map.pgm";
		    system(cp_command1.c_str());

		    std::string cp_command2("cp ");
		    cp_command2 += conf_file_name_str.c_str();
		    cp_command2 += maps_path;
		    cp_command2 += " map.yaml";
		    system(cp_command2.c_str());
	    }


    }


}

float MainWindow::transform_x(float x)
{

    float map_x = map_origin_x + x*map_width_relation*map_resolution*width/new_width; //required for cases when the height is larger than the width
    return map_x;

}

float MainWindow::transform_y(float y)
{
    float map_y;
    map_y = map_origin_y + input_map_height*map_resolution - y*map_height_relation*map_resolution; //seems to work in all cases
    return map_y;


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
void MainWindow::on_save_room_clicked()
{
    bool ok;
    QString text_input = QInputDialog::getText(this,("Enter room name"),(""),QLineEdit::Normal,QString::null,&ok);
    std::string utf8_room_name = text_input.toUtf8().constData();
    std::cout << "Room name " << utf8_room_name << std::endl;

    //instantiate and save the room
    std::string current_room_name = utf8_room_name;
    MyQLabel::Room room(current_room_name, ui->label->corners);
    ui->label->rooms.push_back(room);

    //reset current corners
    ui->label->corners.clear();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Delete room button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::on_delete_room_clicked()
{
    //Open dialog for the user to enter the room name
    bool ok;
    QString text_input = QInputDialog::getText(this,("Enter room name"),(""),QLineEdit::Normal,QString::null,&ok);

    std::string room_to_be_deleted = text_input.toUtf8().constData();

    bool found = false;

    //look for the room name among the existing rooms
    for(unsigned int i=0; i<ui->label->rooms.size();i++)
    {
        std::string room_name = ui->label->rooms[i].room_name;
        if (room_name.compare(room_to_be_deleted) == 0)
        {
            found = true;

            ui->label->rooms.erase(ui->label->rooms.begin()+i);
            break;

        }

    }

    //if the requested room is not found, give output message so that any spelling errors can be corrected
    if(!found)
    {
        QMessageBox msgBox;
        msgBox.setText("That room does not exist");
        msgBox.exec();
    }

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Reset corners button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MainWindow::on_reset_clicked()
{
    //Remove current corners
    ui->label->corners.clear();
    ui->label->repaint();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Callback for the Save file button
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int MainWindow::on_save_file_clicked()
{

    // if no rooms were saved, show output message
    if (ui->label->rooms.size()==0)
    {
        //std::cout << "No rooms to be saved " << std::endl;
        QMessageBox msgBox;
        msgBox.setText("No rooms to be saved");
        msgBox.exec();
        return false;

    }

    // open dialog for the user to enter the file name
    QString file_name = QFileDialog::getSaveFileName(this,
             tr("Save to file"), "/opt/ros/hobbit_hydro/src/navigation/places_files",
             tr("All Files (*)"));

    string file_name_str = file_name.toStdString();

    //create pugixml document to do the parsing to a xml file
    //Qt has its own xml parsing functions, but pugixml is very light and implementation was faster
    pugi::xml_document doc;

    pugi::xml_node node_0 = doc.append_child();
    node_0.set_name("rooms");

    //add first room
    /*****************************************************************************/
    // add room
    pugi::xml_node room_0 = node_0.append_child();
    room_0.set_name("room");

    // add room_name
    pugi::xml_node room_name = room_0.append_child();
    room_name.set_name("name");
    string room_0_name = ui->label->rooms[0].room_name;
    room_name.append_child(pugi::node_pcdata).set_value(room_0_name.c_str());

    // add corners
    pugi::xml_node corners_0 = room_0.insert_child_after(pugi::node_element, room_name);
    corners_0.set_name("vertices");

    //add corner_0_0
    pugi::xml_node corner_0_0 = corners_0.append_child();
    corner_0_0.set_name("vertex");

    // add attributes to corner
    float corner_0_0_x = transform_x(ui->label->rooms[0].room_corners[0].x);
    char corner_0_0_x_converted[1000];
    snprintf(corner_0_0_x_converted,sizeof(corner_0_0_x_converted), "%f", corner_0_0_x);
    corner_0_0.append_attribute("x") = corner_0_0_x_converted;

    float corner_0_0_y = transform_y(ui->label->rooms[0].room_corners[0].y);
    char corner_0_0_y_converted[1000];
    snprintf(corner_0_0_y_converted,sizeof(corner_0_0_y_converted), "%f", corner_0_0_y);
    corner_0_0.append_attribute("y") = corner_0_0_y_converted;


    //add other corners
    pugi::xml_node prev_corner_0 = corner_0_0;

    for (unsigned int j=1;j<ui->label->rooms[0].room_corners.size();j++)
    {
         pugi::xml_node corner_0_j = corners_0.insert_child_after(pugi::node_element, prev_corner_0);
         corner_0_j.set_name("vertex");

         float  corners_0_j_x= transform_x(ui->label->rooms[0].room_corners[j].x);
         char corners_0_j_x_converted[1000];
         snprintf(corners_0_j_x_converted,sizeof(corners_0_j_x_converted), "%f", corners_0_j_x);
         corner_0_j.append_attribute("x") = corners_0_j_x_converted;

         float  corners_0_j_y= transform_y(ui->label->rooms[0].room_corners[j].y);
         char corners_0_j_y_converted[1000];
         snprintf(corners_0_j_y_converted,sizeof(corners_0_j_y_converted), "%f", corners_0_j_y);
         corner_0_j.append_attribute("y") = corners_0_j_y_converted;

         prev_corner_0 = corner_0_j;


      }


      /*****************************************************************************/

      //add other rooms
      pugi::xml_node prev_room = room_0;

      for (unsigned int i=1;i<ui->label->rooms.size();i++)
      {
           // add room
           pugi::xml_node room_i = node_0.insert_child_after(pugi::node_element,prev_room);
           room_i.set_name("room");

           // add room_name
           pugi::xml_node room_name_i = room_i.append_child();
           room_name_i.set_name("name");
           string room_i_name = ui->label->rooms[i].room_name;
           room_name_i.append_child(pugi::node_pcdata).set_value(room_i_name.c_str());

           // add vertices
           pugi::xml_node corners_i = room_i.insert_child_after(pugi::node_element, room_name_i);
           corners_i.set_name("vertices");

           //add vertex_i_0
           pugi::xml_node corner_i_0 = corners_i.append_child();
           corner_i_0.set_name("vertex");

           // add attributes to vertex
           float corner_i_0_x = transform_x(ui->label->rooms[i].room_corners[0].x);
           char corner_i_0_x_converted[1000];
           snprintf(corner_i_0_x_converted,sizeof(corner_i_0_x_converted), "%f", corner_i_0_x);
           corner_i_0.append_attribute("x") = corner_i_0_x_converted;

           float corner_i_0_y = transform_y(ui->label->rooms[i].room_corners[0].y);
           char corner_i_0_y_converted[1000];
           snprintf(corner_i_0_y_converted,sizeof(corner_i_0_y_converted), "%f", corner_i_0_y);
           corner_i_0.append_attribute("y") = corner_i_0_y_converted;

           //add other vertices
           pugi::xml_node prev_corner_i = corner_i_0;

           for (unsigned int j=1;j<ui->label->rooms[i].room_corners.size();j++)
           {
               pugi::xml_node corner_i_j = corners_i.insert_child_after(pugi::node_element, prev_corner_i);
               corner_i_j.set_name("vertex");

               // add attributes to corner
               float  corners_i_j_x= transform_x(ui->label->rooms[i].room_corners[j].x);
               char corners_i_j_x_converted[1000];
               snprintf(corners_i_j_x_converted,sizeof(corners_i_j_x_converted), "%f", corners_i_j_x);
               corner_i_j.append_attribute("x") = corners_i_j_x_converted;

               float  corners_i_j_y= transform_y(ui->label->rooms[i].room_corners[j].y);
               char corners_i_j_y_converted[1000];
               snprintf(corners_i_j_y_converted,sizeof(corners_i_j_y_converted), "%f", corners_i_j_y);
               corner_i_j.append_attribute("y") = corners_i_j_y_converted;

               prev_corner_i = corner_i_j;

           }

           prev_room = room_i;

      }


      //doc.print(std::cout);

      // save document to file
       //std::cout << "Saving places: " << doc.save_file("places.xml") << std::endl;
       std::cout << "Saving places: " << file_name_str.c_str() << std::endl;

       if (doc.save_file(file_name_str.c_str())) //FIXME
       {

	   if (create_sym_link)
	   {
		   sym_link += file_name_str.c_str();
		   system(sym_link.c_str());
	   }

	   if (convert_maps)
	   {
		   std::string command_string("/opt/ros/hobbit_hydro/src/navigation/start_map_conversion.sh ");

		   int ind_end = file_name_str.find_last_of(".");
		   std::string file_name = file_name_str.substr(0,ind_end); //remove the extension

		   std::cout << file_name << std::endl;
		   int ind = file_name_str.find_last_of("/");
		   file_name = file_name.substr(ind+1);

		   std::cout << file_name << std::endl;
		   int ind_ = file_name.find_first_of("_");
		   file_name = file_name.substr(ind_+1);

		   std::cout << file_name << std::endl;



		   command_string += file_name.c_str();
		   std::cout << command_string << std::endl;

		   system(command_string.c_str());
           }



           return true;
       }
       else
       {
           std::cout << "File " << file_name_str.c_str() << " could not be properly saved" << std::endl;
       }

}







