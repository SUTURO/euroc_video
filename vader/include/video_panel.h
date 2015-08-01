#ifndef VIDEO_PANEL_H
#define VIDEO_PANEL_H 

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QCheckBox>
#include <QTableWidget>
#include <QWidget>
#include <QComboBox>
#include <ROSConnector.h>
#include <PlayLogs.h>
#include "suturo_video_msgs/Test.h"
#include "suturo_video_msgs/TestResult.h"
#endif

namespace video_panel_plugin
{
class PrefixLabel : public QLabel
{
public:
    PrefixLabel(std::string prefix);
    PrefixLabel(std::string prefix, std::string suffix);
    void updateSuffix(std::string suffix);

private:
    QLabel label;
    std::string prefix;
    std::string suffix;
};

class VideoPanel: public rviz::Panel
{

Q_OBJECT
public:
	VideoPanel(QWidget* parent = 0);

	// ~VideoPanel();

private Q_SLOTS:
    void handlePullButton();
    void handleFailedRunsCheckBox(int state);
    void loadRun(QListWidgetItem* item);
    void handleSelectedTest();
    void handlePlayButton();
	
private:
    ROSConnector connector;
    QPushButton *pullRunsButton;
    QListWidget *availableRunsList;
    QCheckBox *failedRuns;
    QListWidget *performedTestsList;
    QLabel *selectedRunLabel;
    QTabWidget *tab;
    QWidget *playerWidget;
    QWidget *testResultsWidget;
    QLabel *testLabel;
    PrefixLabel *testResultLabel;
    PrefixLabel *timePointsLabel;
    PrefixLabel *testListLabel;
    QComboBox *timePointsBox;

    // PlayLogs funktioniert nur, wenn der dbPlayer von Jannik läuft, ansonsten startet Rviz das Plugin nicht
    // PlayLogs playLogs;
    QWidget *playLogsWidget;
    QComboBox *selectTopicBox;
    QComboBox *selectStartTimeBox;
    QComboBox *selectEndTimeBox;
    QPushButton *startLogButton;

    int timePointsNumber;

    std::vector<suturo_video_msgs::Test> returnedTests;
    suturo_video_msgs::Test getTestFromList(QString name);

    /**
      creates a QString with surround html tags

      @param text text that should be bold
      @return the formatted QString
    */
    QString setBoldText(QString text);

    void setTestLabel(QString text);
};

}
#endif // VIDEO_PANEL_H
