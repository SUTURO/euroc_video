#include "video_panel.h"
#include <stdio.h>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace video_panel_plugin
{
	VideoPanel::VideoPanel( QWidget* parent)
	: rviz::Panel(parent)
	{
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
        button1 = new QPushButton("Blub1", this);
        button2 = new QPushButton("Blub2", this);
        button3 = new QPushButton("Blub3", this);
        button_layout->addWidget(button1);
        button_layout->addWidget(button2);
        button_layout->addWidget(button3);

        connect(button1, SIGNAL (clicked()), this, SLOT (handle_button1()));
        connect(button2, SIGNAL (clicked()), this, SLOT (handle_button2()));
        connect(button3, SIGNAL (clicked()), this, SLOT (handle_button3()));

        setLayout(vertical_layout);
	}

    void VideoPanel::handle_button1()
    {
        label1->setText("pushed");
    }

    void VideoPanel::handle_button2()
    {
        label2->setText("pushed");
    }

    void VideoPanel::handle_button3()
    {
        label3->setText("pushed");
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(video_panel_plugin::VideoPanel,rviz::Panel )
