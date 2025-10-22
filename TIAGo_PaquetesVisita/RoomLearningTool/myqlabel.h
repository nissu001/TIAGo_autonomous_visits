
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef MYQLABEL_H
#define MYQLABEL_H
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <QLabel>

class MyQLabel : public QLabel
{

public:
    MyQLabel (QWidget * parent = 0, Qt::WindowFlags f = 0):QLabel(parent,f){map_opened = false;}
    MyQLabel (const QString & text, QWidget * parent = 0, Qt::WindowFlags f = 0):QLabel(text,parent,f){;}
    virtual ~MyQLabel(){}   

    virtual void mousePressEvent(QMouseEvent *e);

    virtual void paintEvent(QPaintEvent * e);

    struct Point2D {
        float x;
        float y;
        Point2D(float x_val, float y_val) : x(x_val), y(y_val) {}
    };

    std::vector<Point2D> corners;

    struct Room {
        std::string room_name;
        std::vector<MyQLabel::Point2D> room_corners;
        Room(std::string rn, std::vector<MyQLabel::Point2D> rc) : room_name(rn), room_corners(rc) {}
    };


    std::vector<Room> rooms;

    bool map_opened;

};

#endif // MYQLABEL_H
