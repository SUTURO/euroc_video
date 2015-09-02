#ifndef VIDEO_PANEL_H
#define VIDEO_PANEL_H 

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "mongo/client/dbclient.h"
#include <rviz/panel.h>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QCheckBox>
#include <QTableWidget>
#include <QWidget>
#include <QComboBox>
#include <ROSConnector.h>
#include <QResizeEvent>
#include <PlayLogs.h>
#include <dirent.h>
#include <stdio.h>
#include "suturo_video_msgs/Test.h"
#include "suturo_video_msgs/TestResult.h"
#endif

using mongo::BSONObj;
using mongo::BSONElement;

namespace video_panel_plugin
{
class ImageLabel : public QLabel
{
public:
	QPixmap *imagePixmap;
	void resizeEvent(QResizeEvent *);

};

class PrefixLabel : public QLabel
{
public:
    PrefixLabel(std::string prefix);
    PrefixLabel(std::string prefix, std::string suffix);
    void updateSuffix(std::string suffix);
    void updateSuffix(int suffix);
    std::string getSuffix();

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
    void handleAddTestsButton();
    void handleFailedRunsCheckBox(int state);
    void loadRun(QListWidgetItem* item);
    void handleSelectedTest();
    void handlePlayButton();
    void handleHighlightsTab();
    void handleHighlightsRefreshDataSetsButton();
    void handleHighlightsRefreshImagesButton();
    void handleSelectDataSet(QListWidgetItem* item);
    void handleSelectImage(QListWidgetItem* item);

	
private:
    mongo::DBClientConnection mongoClient;

    ROSConnector connector;
    QPushButton *pullRunsButton;
    PrefixLabel *availableTestsLabel;
    QPushButton *addTestsButton;
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
    PlayLogs playLogs;
    QWidget *playLogsWidget;
    QComboBox *selectTopicBox;
    QComboBox *selectStartTimeBox;
    QComboBox *selectEndTimeBox;
    QPushButton *startLogButton;
    std::string selectedDatabase;

    // Widget for highlights as images.
    const std::string dataDir;
    const bool imagesDebug;
    std::string selectedDataSet;
    std::string selectedImage;
    std::string selectedImageId;
    QLabel *highlightsSelectDataSetLabel;
    QListWidget *highlightsAvailableDataSetsList;
    QPushButton *highlightsRefreshDataSetsButton;
    QWidget *highlightsImagesWidget;
    QListWidget *highlightsAvailableImagesList;
    QPushButton *highlightsRefreshImagesButton;
    QLabel *highlightsSelectImageLabel;
    QLabel *highlightsSelectedImageLabel;
    ImageLabel *highlightsImageLabel;
//    QPixmap *imagePixmap;
//    QPainter *imageDisplay;

    void updateDataSets();
    void loadDataSet();
    void loadImage();
    bool fileHasExtension(std::string file, std::string extension);

    int timePointsNumber;
    std::vector<ros::Time> timePointList;

    std::vector<suturo_video_msgs::Test> returnedTests;
    std::vector<std::string> playableTopics;
    suturo_video_msgs::Test getTestFromList(QString name);

    void showMessage(std::string text);
    void showMessage(std::string text, std::string additionalText);

    void mongoInit();
    std::string timeToString(ros::Time t);


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
