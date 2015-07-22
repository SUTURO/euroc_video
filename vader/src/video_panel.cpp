#include "video_panel.h"
#include <stdio.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QListWidgetItem>
#include <boost/lexical_cast.hpp>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

namespace video_panel_plugin
{
	VideoPanel::VideoPanel( QWidget* parent)
	: rviz::Panel(parent)
    {
        timePointsNumber = 0;

        //QTABWIDGET
        // create the QTabWidget
        QVBoxLayout* tabLayout = new QVBoxLayout();
        tab = new QTabWidget();
        tab->setMinimumSize(300, 300);
        tabLayout->addWidget(tab);

        //OVERVIEW
        // create widget for overview tab
        QWidget* overviewWidget = new QWidget();
        QVBoxLayout* overviewLayout = new QVBoxLayout();
        overviewWidget->setLayout(overviewLayout);

        // create label
        QLabel* listLabel = new QLabel("Available runs:");
        overviewLayout->addWidget(listLabel);

        // create list for available runs
        availableRunsList = new QListWidget();
        overviewLayout->addWidget(availableRunsList);
        connect(availableRunsList, SIGNAL (itemDoubleClicked(QListWidgetItem*)), this, SLOT (loadRun(QListWidgetItem*)));

        // create button to pull available runs
        pullRunsButton = new QPushButton("Pull available runs", this);
        overviewLayout->addWidget(pullRunsButton);
        connect(pullRunsButton, SIGNAL (clicked()), this, SLOT (handlePullButton()));

        // add checkbox to filter for failed runs
//        failedRuns = new QCheckBox("Show only failed runs");
//        overviewLayout->addWidget(failedRuns);
//        connect(failedRuns, SIGNAL (stateChanged(int)), this, SLOT (handleFailedRunsCheckBox(int)));

        // add overviewWidget to QTabWidget
        tab->addTab(overviewWidget, "Overview");

        // PLAYER
        // create widget for player
        playerWidget = new QWidget();
        QVBoxLayout* playerLayout = new QVBoxLayout();
        playerWidget->setLayout(playerLayout);

        // create labels for player tab
        QLabel* runLabel = new QLabel("Selected simulation run:");
        playerLayout->addWidget(runLabel);
        selectedRunLabel = new QLabel("<b>No run selected!</b>");
        playerLayout->addWidget(selectedRunLabel);
        testListLabel = new PrefixLabel("Performed Tests: ");
        playerLayout->addWidget(testListLabel);

        // create list for performed Tests
        performedTestsList = new QListWidget();
        playerLayout->addWidget(performedTestsList);
        connect(performedTestsList, SIGNAL (itemSelectionChanged()), this, SLOT (handleSelectedTest()));

        // create widget to display testresults and additional data
        testResultsWidget = new QWidget();
        QVBoxLayout* testResultsLayout = new QVBoxLayout();
        testResultsWidget->setLayout(testResultsLayout);
        playerLayout->addWidget(testResultsWidget);

        // create label with testname
        testLabel = new QLabel("Please select a testcase from above");
        testResultsLayout->addWidget(testLabel);

        // create label for result
        testResultLabel = new PrefixLabel("Result: ");
        testResultsLayout->addWidget(testResultLabel);

        // create label for number of timepoints
        timePointsLabel = new PrefixLabel("Number of timepoints: ");
        testResultsLayout->addWidget(timePointsLabel);

        // add playerWidget to QTabWidget
        tab->addTab(playerWidget, "Player");

       // add QComboBox
        timePointsBox = new QComboBox();
        timePointsBox->addItem("No timepoints available");
        testResultsLayout->addWidget(timePointsBox);


        // set tabLayout as layout for parent
        setLayout(tabLayout);

        // create ROSConnector
        connector = ROSConnector();
	}


    void VideoPanel::handlePullButton()
    {
        availableRunsList->clear();
        try
        {
            std::vector<std::string> runs = connector.getSimulationRuns();
            if(runs.size() == 0)
            {
                availableRunsList->setEnabled(false);
                QListWidgetItem* item = new QListWidgetItem("No runs available", availableRunsList);
                return;
            }
            availableRunsList->setEnabled(true);
            for (std::vector<std::string>::iterator it = runs.begin() ; it != runs.end(); ++it)
            {
                QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(*it), availableRunsList);
            }
        }
        catch(ServiceUnavailableException &exc)
        {
            availableRunsList->setEnabled(false);
            ROS_ERROR_STREAM(exc.what());
            QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(exc.what()), availableRunsList);
        }
    }

    void VideoPanel::handleFailedRunsCheckBox(int state)
    {
        // TODO: Sort list. values are 0 / 2
    }

    void VideoPanel::loadRun(QListWidgetItem* item)
    {
        // change active tab
        tab->setCurrentWidget(playerWidget);

        // fill listwidget
        performedTestsList->clear();
        try
        {
            returnedTests = connector.getExecutedTests(item->text().toStdString());
            selectedRunLabel->setText(setBoldText(item->text()));
            for(std::vector<suturo_video_msgs::Test>::iterator it = returnedTests.begin(); it != returnedTests.end(); ++it){
                std::string name = (*it).name;
                QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(name), performedTestsList);
            }

            // create label for performed tests
            int availableTests = connector.getAvailableTests(item->text().toStdString()).capacity();
            std::ostringstream os;
            os << returnedTests.capacity() << " out of " << availableTests;
            std::string s = os.str();
            testListLabel->updateSuffix(s.c_str());

            // reset restResult and timePoints
            testResultLabel->updateSuffix("");
            timePointsLabel->updateSuffix("");
            timePointsBox->clear();
            testLabel->setText("Please select a testcase from above");

//            int height = 5 * performedTestsList->visualItemRect(performedTestsList->item(0)).height();
//            performedTestsList->setFixedHeight(height);
        }
        catch(ServiceUnavailableException &exc)
        {
            performedTestsList->setEnabled(false);
            ROS_ERROR_STREAM(exc.what());
            QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(exc.what()), performedTestsList);
        }
        // TODO: Implement real losaing of run
    }

    void VideoPanel::handleSelectedTest()
    {
        QListWidgetItem *selectedItem = performedTestsList->currentItem();
        // set label for current Test
        VideoPanel::setTestLabel(selectedItem->text());
        // get Testobject
        suturo_video_msgs::Test testCase = VideoPanel::getTestFromList(selectedItem->text());
        // set testresult label
        if(testCase.test_result.result){
            testResultLabel->setText(VideoPanel::setBoldText(QString("Testresult: success")));
        }
        else
        {
            testResultLabel->setText(VideoPanel::setBoldText(QString("Testresult: failure")));
        }

        // get number of timepoints and set label
        timePointsNumber = testCase.test_result.notableTimePoints.capacity();
        timePointsLabel->updateSuffix(boost::lexical_cast<std::string>(timePointsNumber));

        // insert timepoints if exist
        timePointsBox->clear();
        if(timePointsNumber == 0)
        {
            timePointsBox->addItem("No timepoints available");
        }
        else
        {
            for(std::vector<ros::Time>::iterator it = testCase.test_result.notableTimePoints.begin(); it != testCase.test_result.notableTimePoints.end(); ++it)
            {
                std::ostringstream os;
                os << *it;
                std::string s = os.str();
                timePointsBox->addItem(s.c_str());
            }
        }
//        std::cout << testcase.description << std::endl;
//        std::cout << testcase.expected << std::endl;
//        std::cout << testcase.query << std::endl;
//        std::cout << testcase.test_result.executionDate << std::endl;
//        std::cout << testCase.test_result.notableTimePoints.capacity() << std::endl;
//        std::cout << testcase.test_result.bindings.capacity() << std::endl;
    }

    QString VideoPanel::setBoldText(QString text)
    {
        QString* s = new QString();
        s->append("<b>");
        s->append(text);
        s->append("</b>");
        return *s;
    }

    void VideoPanel::setTestLabel(QString text)
    {
        QString* label = new QString("Testcase: ");
        label->append(text);
        testLabel->setText(*label);
    }

    suturo_video_msgs::Test VideoPanel::getTestFromList(QString name){
        for(std::vector<suturo_video_msgs::Test>::iterator it = returnedTests.begin(); it != returnedTests.end(); ++it){
            if (name.toStdString().compare((*it).name) == 0){
                return *it;
            }
        }
    }

    PrefixLabel::PrefixLabel(std::string prefix):QLabel()
    {
        this->prefix = prefix;
        this->setText(QString(this->prefix.c_str()));
    }

    PrefixLabel::PrefixLabel(std::string prefix, std::string suffix):QLabel()
    {
        this->prefix = prefix;
        PrefixLabel::updateSuffix(suffix);
    }

    void PrefixLabel::updateSuffix(std::string suffix)
    {
        this->suffix = suffix;
        this->setText(QString(this->prefix.c_str()).append(QString(this->suffix.c_str())));
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(video_panel_plugin::VideoPanel,rviz::Panel )
