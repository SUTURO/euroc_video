#include "video_panel.h"
#include <stdio.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QListWidgetItem>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
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
	void ImageLabel::resizeEvent(QResizeEvent *e) {
		int w = this->size().width();
		int h = this->size().height();
		QPixmap *p = this->imagePixmap;
		this->setPixmap(p->scaled(w, h, Qt::KeepAspectRatio));
	}

	VideoPanel::VideoPanel( QWidget* parent)
	: rviz::Panel(parent),
	  dataDir("/home/suturo/catkin_ws/src/euroc/sr_experimental_data"),
	  imagesDebug(false)
    {
        timePointsNumber = 0;
        mongoInit();

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

        // line for decoration
        QFrame* line = new QFrame();
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(320, 150, 118, 3));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        overviewLayout->addWidget(line);

        // create label for available tests
        availableTestsLabel = new PrefixLabel("Number of available Test: ", "0");
        overviewLayout->addWidget(availableTestsLabel);

        // create button to add tests
        addTestsButton = new QPushButton("Add a test");
        overviewLayout->addWidget(addTestsButton);
        connect(addTestsButton, SIGNAL (clicked()), this, SLOT (handleAddTestsButton()));


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

        // PLAY LOGS
        // Play Logs Tab
        playLogsWidget = new QWidget();
        QVBoxLayout* playLogsLayout = new QVBoxLayout();
        playLogsWidget->setLayout(playLogsLayout);

        // create Dropdown Menu for Collection / Topics
        QLabel* selectTopicLabel = new QLabel("Select topic:");
        playLogsLayout->addWidget(selectTopicLabel);

        selectTopicBox = new QComboBox();
        selectTopicBox->addItem("No topics available");
        playLogsLayout->addWidget(selectTopicBox);

        // create Dropdown Menu for start and end time
        QLabel* selectStartTimeLabel = new QLabel("Select start time:");
        playLogsLayout->addWidget(selectStartTimeLabel);

        selectStartTimeBox = new QComboBox();
        selectStartTimeBox->addItem("No time available");
        playLogsLayout->addWidget(selectStartTimeBox);

        QLabel* selectEndTimeLabel = new QLabel("Select end time:");
        playLogsLayout->addWidget(selectEndTimeLabel);

        selectEndTimeBox = new QComboBox();
        selectEndTimeBox->addItem("No time available");
        playLogsLayout->addWidget(selectEndTimeBox);

        // create button to pull available runs
        startLogButton = new QPushButton("Start", this);
        playLogsLayout->addWidget(startLogButton);
        connect(startLogButton, SIGNAL (clicked()), this, SLOT (handlePlayButton()));


        // add playlogs to QTabWidget
        tab->addTab(playLogsWidget, "Play Logs");


        // Widget for highlights as images.
        highlightsImagesWidget = new QWidget();
        QVBoxLayout* highlightsLayout = new QVBoxLayout();
        QVBoxLayout* highlightsDataSetLayout = new QVBoxLayout();
        QVBoxLayout* highlightsImagesLayout = new QVBoxLayout();
        QHBoxLayout* highlightsSelectionLayout = new QHBoxLayout();
        highlightsImagesWidget->setLayout(highlightsLayout);

        highlightsLayout->addLayout(highlightsSelectionLayout);
        highlightsSelectionLayout->addLayout(highlightsDataSetLayout);
        highlightsSelectionLayout->addLayout(highlightsImagesLayout);

        if (imagesDebug) {
        	highlightsSelectDataSetLabel = new QLabel("Select data set:");
        	highlightsDataSetLayout->addWidget(highlightsSelectDataSetLabel);
			highlightsAvailableDataSetsList = new QListWidget();
			highlightsDataSetLayout->addWidget(highlightsAvailableDataSetsList);
			connect(highlightsAvailableDataSetsList, SIGNAL (itemActivated(QListWidgetItem*)), this, SLOT (handleSelectDataSet(QListWidgetItem*)));
			highlightsRefreshDataSetsButton = new QPushButton("Refresh", this);
			highlightsDataSetLayout->addWidget(highlightsRefreshDataSetsButton);
			connect(highlightsRefreshDataSetsButton, SIGNAL (clicked()), this, SLOT (handleHighlightsRefreshDataSetsButton()));
        }

        highlightsSelectImageLabel = new QLabel("Select image (no data base loaded):");
        highlightsImagesLayout->addWidget(highlightsSelectImageLabel);
        highlightsAvailableImagesList = new QListWidget();
        highlightsImagesLayout->addWidget(highlightsAvailableImagesList);
        highlightsRefreshImagesButton = new QPushButton("Refresh", this);
        highlightsImagesLayout->addWidget(highlightsRefreshImagesButton);
		connect(highlightsRefreshImagesButton, SIGNAL (clicked()), this, SLOT (handleHighlightsRefreshImagesButton()));
        connect(highlightsAvailableImagesList, SIGNAL (itemActivated(QListWidgetItem*)), this, SLOT (handleSelectImage(QListWidgetItem*)));
//        imageDisplay = new QPainter();
//        highlightsLayout->addWidget(imageDisplay);
        highlightsImageLabel = new ImageLabel();
//        imageLabel->setScaledContents(true);
        highlightsImageLabel->setStyleSheet("border: 1px solid");
        //selectedDatabase = "exp-2015-08-01_11-03-48"; // TODO: Remove to actually use selected database
        //std::string image_name = "0__euroc_interface_node_cameras_scene_depth_cam.jpg";
        // TODO: get list of images with timestamps from prolog
        highlightsImageLabel->setMinimumHeight(200);
        highlightsImageLabel->imagePixmap  = new QPixmap();

        highlightsSelectedImageLabel = new QLabel("");
        highlightsLayout->addWidget(highlightsSelectedImageLabel);
        highlightsLayout->addWidget(highlightsImageLabel);
        //imageLabel->setPixmap(imagePixmap.scaled(imagesWidget->width(), imagesWidget->height(), Qt::KeepAspectRatio));
//        imageLabel->setPixmap(imagePixmap);
        tab->addTab(highlightsImagesWidget, "Highlights");

        // set tabLayout as layout for parent
        setLayout(tabLayout);

        // create ROSConnector
        connector = ROSConnector();
	}

	// Mongo stuff
	void VideoPanel::mongoInit() {
//		mongo::client::initialize();
		mongoClient.connect(getenv("MONGO_PORT_27017_TCP_ADDR"));
	}

	std::string VideoPanel::timeToString(ros::Time t) {
		std::stringstream ss;
		ss << t.sec << "." << t.nsec;
		return ss.str();
	}

	bool VideoPanel::fileHasExtension(std::string file, std::string extension) {
		const char *fspec = file.c_str();
		int len = file.length();
		if (len >= extension.length()) {
			if (strcmp(extension.c_str(), fspec + len - extension.length()) == 0) {
				return true;
			}
		}
		return false;
	}

	void VideoPanel::handleHighlightsTab() {
		if (!highlightsImagesWidget->isActiveWindow()) {
			showMessage("Updating!");
			updateDataSets();
		}
	}

	void VideoPanel::handleHighlightsRefreshDataSetsButton() {
		updateDataSets();
	}

	void VideoPanel::handleHighlightsRefreshImagesButton() {
		loadDataSet();
	}

	void VideoPanel::handleSelectDataSet(QListWidgetItem* item) {
		selectedDataSet = item->text().toStdString();
		loadDataSet();
	}

	void VideoPanel::handleSelectImage(QListWidgetItem* item) {
		selectedImage = item->text().toStdString();
		selectedImageId = item->data(Qt::UserRole).toString().toStdString();
		std::cout << "selectedImageId: " << selectedImageId << std::endl;
		loadImage();
	}

	void VideoPanel::loadImage() {
		highlightsSelectedImageLabel->setText(QString::fromStdString(selectedImage));

		std::string ns = selectedDatabase + ".logged_images_out_compressed";
		BSONObj queryObject = mongo::BSONObjBuilder().append("_id", mongo::OID(selectedImageId)).obj();

		BSONObj objectData = mongoClient.findOne(ns, queryObject);
				//query(selectedDatabase + ".logged_images_out_compressed", BSONObj());

		std::cout << "ns: " << ns << std::endl;
		std::cout << "queryObject: " << queryObject << std::endl;
		std::cout << "queryObject id : " << queryObject.getField("_id").toString(false) << std::endl;
		std::cout << selectedImageId << ": " << objectData.toString() << std::endl;

		BSONElement imageData = objectData.getField("data");
		int l = 1000;
		std::cout << "imageData: " << imageData.binData(l) << std::endl;


		/*
		std::string file = dataDir + "/" + selectedDataSet + "/" + selectedImage;
		//QPixmap tmpPix (file.c_str());
		//highlightsImageLabel->setPixmap(tmpPix);
		//highlightsImageLabel->imagePixmap  = new QPixmap("/home/suturo/catkin_ws/src/euroc/sr_experimental_data/exp-2015-08-01_11-03-48/0__euroc_interface_node_cameras_scene_depth_cam.jpg");
		highlightsImageLabel->imagePixmap  = new QPixmap(QString::fromStdString(file));
		highlightsImageLabel->repaint();
		highlightsImageLabel->resizeEvent(new QResizeEvent(highlightsImageLabel->size(), highlightsImageLabel->size()));
		//highlightsImageLabel->update();
		 */

		/*
		highlightsSelectedImageLabel->setText(QString::fromStdString(selectedImage));
		std::string file = dataDir + "/" + selectedDataSet + "/" + selectedImage;
		//QPixmap tmpPix (file.c_str());
		//highlightsImageLabel->setPixmap(tmpPix);
		//highlightsImageLabel->imagePixmap  = new QPixmap("/home/suturo/catkin_ws/src/euroc/sr_experimental_data/exp-2015-08-01_11-03-48/0__euroc_interface_node_cameras_scene_depth_cam.jpg");
		highlightsImageLabel->imagePixmap  = new QPixmap(QString::fromStdString(file));
		highlightsImageLabel->repaint();
		highlightsImageLabel->resizeEvent(new QResizeEvent(highlightsImageLabel->size(), highlightsImageLabel->size()));
		//highlightsImageLabel->update();
		 */
	}

	void VideoPanel::loadDataSet() {
		highlightsAvailableImagesList->clear();
		std::vector<std::string> objectIds;
		std::vector<ros::Time> timeStamps;

		mongo::auto_ptr<mongo::DBClientCursor> cursor = mongoClient.query(selectedDatabase + ".logged_images_out_compressed", BSONObj());
		while (cursor->more()) {
			BSONObj doc = cursor->next();
			BSONObj header = doc.getField("header").Obj();
			BSONElement date = header.getField("stamp");
			uint32_t secs = date.Date().millis / 1000;
			uint32_t nsecs = (date.Date().millis % 1000) * 1000000;
			ros::Time time(secs, nsecs);
			mongo::OID id(doc.getField("_id").OID());
			//std::string id = doc.getField("_id").toString(false);
			std::cout << "id: " << id.toString() << std::endl;
			objectIds.push_back(id.toString());
			timeStamps.push_back(time);
		}

		if (timeStamps.empty()) {
			highlightsAvailableImagesList->setEnabled(false);
			highlightsSelectImageLabel->setText("Select image (no data base loaded):");
			QListWidgetItem* item = new QListWidgetItem("No images available", highlightsAvailableImagesList);
			return;
		} else {
			highlightsAvailableImagesList->setEnabled(true);
			highlightsSelectImageLabel->setText(QString::fromStdString("Select image (data base: " + selectedDatabase + "):"));
			for (size_t i = 0; i<objectIds.size(); ++i) {
				QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(timeToString(timeStamps[i])), highlightsAvailableImagesList);
				QVariant objectId = QString::fromStdString(objectIds[i]);
				item->setData(Qt::UserRole, objectId);
			}
		}

		/*

		DIR *imagesDir = opendir((dataDir + "/" + selectedDataSet).c_str());
		if (imagesDir == NULL) {
			highlightsAvailableImagesList->setEnabled(false);
			highlightsSelectImageLabel->setText("Select image (no data set loaded):");
			showMessage("Error loading images.", "The directory containing the images could not be opened.");
			return;
		} else {
			struct dirent *entry = readdir(imagesDir);
			while (entry != NULL) {
				if (entry->d_type == DT_REG && fileHasExtension(entry->d_name, ".jpg")) {
					images.push_back(entry->d_name);
				}
				entry = readdir(imagesDir);
			}
			closedir(imagesDir);

			if (images.empty()) {
				highlightsAvailableImagesList->setEnabled(false);
				highlightsSelectImageLabel->setText("Select image (no data set loaded):");
				QListWidgetItem* item = new QListWidgetItem("No images available", highlightsAvailableImagesList);
				return;
			} else {
				highlightsAvailableImagesList->setEnabled(true);
				highlightsSelectImageLabel->setText(QString::fromStdString("Select image (data set: " + selectedDataSet + "):"));
				for (std::vector<std::string>::iterator it = images.begin() ; it != images.end(); ++it) {
					QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(*it), highlightsAvailableImagesList);
				}
			}
		}
		*/
	}

	void VideoPanel::updateDataSets() {
		highlightsAvailableDataSetsList->clear();
		std::vector<std::string> dataSets;

		DIR *dataSetsDir = opendir(dataDir.c_str());
		if (dataSetsDir == NULL) {
			highlightsAvailableDataSetsList->setEnabled(false);
			showMessage("Error loading data sets.", "The directory containing the data sets could not be opened.");
			return;
		} else {
			struct dirent *entry = readdir(dataSetsDir);
			while (entry != NULL) {
				if (entry->d_type == DT_DIR && strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) {
					dataSets.push_back(entry->d_name);
				}
				entry = readdir(dataSetsDir);
			}
			closedir(dataSetsDir);

			if (dataSets.empty()) {
				highlightsAvailableDataSetsList->setEnabled(false);
				QListWidgetItem* item = new QListWidgetItem("No data sets available", highlightsAvailableDataSetsList);
				return;
			} else {
				highlightsAvailableDataSetsList->setEnabled(true);
				for (std::vector<std::string>::iterator it = dataSets.begin() ; it != dataSets.end(); ++it) {
					QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(*it), highlightsAvailableDataSetsList);
				}
			}
		}
	}

    void VideoPanel::handlePlayButton()
    {
        playLogs = PlayLogs();
        // Derzeit geht nur TF
        // TODO: Mit anderen Msg_types arbeiten
        std::string msg_type = "tf2_msgs/TFMessage";
        std::string database = selectedDatabase;
        std::string collection = selectTopicBox->currentText().toStdString();
        std::string output_topic = selectTopicBox->currentText().toStdString();
//        ros::Time start = ros::Time::now();
        ros::Time start = timePointList.at(selectStartTimeBox->currentIndex());
//        ros::Time end = ros::Time::now() + ros::Duration(100);
        ros::Time end = ros::Time(0);

        if (selectEndTimeBox->currentText().toStdString().compare("Till end") != 0){
            end = timePointList.at(selectEndTimeBox->currentIndex());
        }


        std::string t = selectTopicBox->currentText().toStdString();
        std::cout << "Current Database: " << selectedDatabase << std::endl;
        std::cout << "Current Topic: " << t << std::endl;
        std::cout << "Current Startime: " << start << std::endl;
        std::cout << "Current Endtime: " << end << std::endl;

        playLogs.play_logs(msg_type, database, collection, output_topic, start, end);
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
            VideoPanel::showMessage(exc.what(), "Please see console for further information");
        }
    }

    void VideoPanel::handleAddTestsButton()
    {
        QString testfile = QFileDialog::getOpenFileName(this, "Select a json file that contains a testcase", QDir::currentPath(), tr("json file (*.json)"));
        try
        {
            if(connector.addTests(testfile.toStdString()))
            {
                // get new number of available tests
                int n = connector.getAvailableTests("").capacity();
                availableTestsLabel->updateSuffix(n);
            }
            else
            {
                VideoPanel::showMessage("Unable to upload file", "Please see console for further information");
            }
        }
        catch(ServiceUnavailableException &exc)
        {
            ROS_ERROR_STREAM(exc.what());
            VideoPanel::showMessage(exc.what(), "Please see console for further information");
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
            int nrAvTest = connector.getAvailableTests(item->text().toStdString()).capacity();
            returnedTests = connector.getExecutedTests(item->text().toStdString());
            if (nrAvTest > returnedTests.capacity())
            {
                connector.executeTests(item->text().toStdString());
                returnedTests = connector.getExecutedTests(item->text().toStdString());
            }
            selectedRunLabel->setText(setBoldText(item->text()));
            selectedDatabase = item->text().toStdString();

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
            selectStartTimeBox->clear();
            selectEndTimeBox->clear();
            selectTopicBox->clear();
            testLabel->setText("Please select a testcase from above");

            // Get available Topics to play
            playableTopics = connector.getPlayableTopics(selectedDatabase);

            std::cout << selectedDatabase << std::endl;
            std::cout << playableTopics.at(0) << std::endl;

            if (playableTopics.size() > 0)
            {
                for(int i = 0; i < playableTopics.size(); ++i)
                {
                    std::cout << "Add Topic Nr: " << i << std::endl;
                    selectTopicBox->addItem(QString(playableTopics.at(i).c_str()));
                }
            }
            else
            {
                selectTopicBox->addItem("No Playable Topics found!");
            }
        }
        catch(ServiceUnavailableException &exc)
        {
            performedTestsList->setEnabled(false);
            ROS_ERROR_STREAM(exc.what());
            QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(exc.what()), performedTestsList);
        }
    }

    // TODO: Testen, wenn wir Tests auswählen können
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
        timePointList = testCase.test_result.notableTimePoints;
        timePointsNumber = testCase.test_result.notableTimePoints.capacity();
        timePointsLabel->updateSuffix(timePointsNumber);

        // insert timepoints if exist
        timePointsBox->clear();
        selectStartTimeBox->clear();
        selectEndTimeBox->clear();

        if(timePointsNumber == 0)
        {
            timePointsBox->addItem("No timepoints available");
            selectStartTimeBox->addItem("No timepoints available");
            selectEndTimeBox->addItem("No timepoints available");
        }
        else
        {
            for(std::vector<ros::Time>::iterator it = testCase.test_result.notableTimePoints.begin(); it != testCase.test_result.notableTimePoints.end(); ++it)
            {
                std::ostringstream os;
                os << *it;
                std::string s = os.str();
                timePointsBox->addItem(s.c_str());
                selectStartTimeBox->addItem(s.c_str());
                selectEndTimeBox->addItem(s.c_str());
            }
            std::string tillEnd = "Till end";
            selectEndTimeBox->addItem(tillEnd.c_str());
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

    void PrefixLabel::updateSuffix(int suffix)
    {
        this->suffix = boost::lexical_cast<std::string>(suffix);
        this->setText(QString(this->prefix.c_str()).append(QString(this->suffix.c_str())));
    }

    void VideoPanel::showMessage(std::string text)
    {
        QMessageBox msgBox;
        msgBox.setText(QString::fromStdString(text));
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();
    }

    void VideoPanel::showMessage(std::string text, std::string additionalText)
    {
        QMessageBox msgBox;
        msgBox.setText(QString::fromStdString(text));
        msgBox.setInformativeText(QString::fromStdString(additionalText));
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(video_panel_plugin::VideoPanel,rviz::Panel)
