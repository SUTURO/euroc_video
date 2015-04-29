#include "video_panel.h"
#include <stdio.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>
#include <QTabWidget>
#include <QListWidgetItem>

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
        // create the QTabWidget
        QVBoxLayout* tabLayout = new QVBoxLayout();
        QTabWidget* tab = new QTabWidget();
        tab->setMinimumSize(300, 300);
        tabLayout->addWidget(tab);

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
        connect(availableRunsList, SIGNAL (itemDoubleClicked(QListWidgetItem*)), this, SLOT (loadRun()));

        // create button to pull available runs
        pullRunsButton = new QPushButton("Pull available runs", this);
        overviewLayout->addWidget(pullRunsButton);
        connect(pullRunsButton, SIGNAL (clicked()), this, SLOT (handlePullButton()));

        // add checkbox to filter for failed runs
        failedRuns = new QCheckBox("Show only failed runs");
        overviewLayout->addWidget(failedRuns);
        connect(failedRuns, SIGNAL (stateChanged(int)), this, SLOT (handleFailedRunsCheckBox(int)));

        // add Widget to QTabWidget
        tab->addTab(overviewWidget, "Overview");

        // set tabLayout as layout for parent
        setLayout(tabLayout);

        // Old testing stuff below
        /*
        QVBoxLayout* vertical_layout = new QVBoxLayout;

        QHBoxLayout* label_layout = new QHBoxLayout;
        vertical_layout->addLayout(label_layout);
        label1 = new QLabel("Blub1");
        label2 = new QLabel("Blub2");
        label3 = new QLabel("Blub3");
        label_layout->addWidget(label1);
        label_layout->addWidget(label2);
        label_layout->addWidget(label3);

        QHBoxLayout* button_layout = new QHBoxLayout;
        vertical_layout->addLayout(button_layout);

        button2 = new QPushButton("Blub2", this);
        button3 = new QPushButton("Blub3", this);
        button_layout->addWidget(pullRunsButton);
        button_layout->addWidget(button2);
        button_layout->addWidget(button3);

        setLayout(vertical_layout);
        */
	}

    void VideoPanel::handlePullButton()
    {
        QListWidgetItem* item = new QListWidgetItem("New item", availableRunsList);
    }

    void VideoPanel::handleFailedRunsCheckBox(int state)
    {
        // TODO: Sort list. values are 0 / 2
        QListWidgetItem* item = new QListWidgetItem(QString::number(state), availableRunsList);
    }

    void VideoPanel::loadRun()
    {
        // TODO
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(video_panel_plugin::VideoPanel,rviz::Panel )
