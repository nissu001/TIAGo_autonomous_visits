#ifndef MYPANEL_H
#define MYPANEL_H

#include <QGLWidget>

class MyPanel : public QGLWidget
{
    Q_OBJECT
public:
    explicit MyPanel(QWidget *parent = 0);
    
signals:
    
public slots:


protected:
    void initializeGL();
    void resizeGL(int x, int h);
    void paintGL();

private:

    
};

#endif // MYPANEL_H
