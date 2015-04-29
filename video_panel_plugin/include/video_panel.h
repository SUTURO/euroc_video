#ifndef VIDEO_PANEL_H
#define VIDEO_PANEL_H 

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QCheckBox>
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
    void handlePullButton();
    void handleFailedRunsCheckBox(int state);
    void loadRun();
	
private:
    QPushButton *pullRunsButton;
    QListWidget *availableRunsList;
    QCheckBox *failedRuns;
};
}
#endif // VIDEO_PANEL_H
