#ifndef VIDEO_PANEL_H
#define VIDEO_PANEL_H 

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#include <QLabel>
#include <QPushButton>
#endif

namespace video_panel_plugin
{

class VideoPanel: public rviz::Panel
{

Q_OBJECT
public:
	VideoPanel(QWidget* parent = 0);

	// ~VideoPanel();

private Q_SLOTS:
    void handle_button1();
    void handle_button2();
    void handle_button3();
	
private:
    QLabel *label1;
    QLabel *label2;
    QLabel *label3;
    QPushButton *button1;
    QPushButton *button2;
    QPushButton *button3;
};
}
#endif // VIDEO_PANEL_H
