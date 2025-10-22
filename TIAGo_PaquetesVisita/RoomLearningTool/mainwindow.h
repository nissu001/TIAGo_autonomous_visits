#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPaintEvent>

#include "myqlabel.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private slots:
    
    void on_load_map_clicked();

    void on_save_room_clicked();

    void on_reset_clicked();

    int on_save_file_clicked();

    void on_delete_room_clicked();



private:

    Ui::MainWindow *ui;
    void LoadMap();
    float stringToFloat(const std::string& str);

    QString map_file_name;

    float input_map_height;
    float map_width_relation;
    float map_height_relation;
    float map_resolution;

    float map_origin_x;
    float map_origin_y;

    float transform_x(float x);
    float transform_y(float y);

    float width;
    float height;

    float new_width;
    float new_height;

    bool copy_maps;
    std::string maps_path;

    bool create_sym_link;
    std::string sym_link;

    bool convert_maps;
    bool copy_files;



};

#endif // MAINWINDOW_H
