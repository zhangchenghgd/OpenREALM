#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "UAVImageTransDialog.h"
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QMessageBox>
#include <QFileDialog>
#include <QProcess>
#include <QApplication>
#include "OsgWidget.h"
#include "FrameWidget.h"
#include "DataManagerWidget.h"
#include "UAVConDialog.h"
#include <osg/Camera>
#include <osgViewer/Viewer>
#include <osgDB/ReaderWriter>
#include <osgDB/ReadFile>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include "StageNode.h"
#include "Scene3DNode.h"
#include "FrameViewNode.h"
#include "grabber_exiv2_node.h"
#include "JonFmvTKNode.h"
#include <QDebug>


using namespace std;


MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent), ui(new Ui::MainWindow),
	m_terrain_manipulator(nullptr), 
	m_trackball_manipulator(nullptr),
	exivNode(nullptr), vtkNode(nullptr),
	pose_estimation_stageNode(nullptr),
	densification_stageNode(nullptr),
	surface_generation_stageNode(nullptr),
	ortho_rectification_stageNode(nullptr),
	mosaicing_stageNode(nullptr),
	scene3DNode(nullptr),
	frameViewNode(nullptr),
	m_followFrameView(0),
	m_stageDirDateTime(QString())
{
	ui->setupUi(this);

	ui->action_start->setEnabled(false);
	ui->action_pause->setEnabled(false);
	ui->action_resume->setEnabled(false);
	ui->action_stop->setEnabled(false);
	ui->action_openProduction->setEnabled(false);
	ui->action_flowFrameCenter->setEnabled(false);
	ui->action_flowFrameView->setEnabled(false);

	m_frameWidget = new FrameWidget(this);
	m_dataMgrWidget = new DataManagerWidget(this);

	m_frameDockWidget = new QDockWidget(QStringLiteral("视频帧"), this);
	m_frameDockWidget->setWidget(m_frameWidget);

	m_dataMgrDockWidget = new QDockWidget(QStringLiteral("数据"), this);
	m_dataMgrDockWidget->setWidget(m_dataMgrWidget);


	this->addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, m_dataMgrDockWidget);
	this->addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, m_frameDockWidget);

	this->setupStatusBar();


	m_uavConDialog = new UAVConDialog(this);

	m_flightLineRoot = new osg::Group;
	m_flightGnssLine = new osg::Group;
	m_flightVisualLine = new osg::Group;
	m_sparseCloud = new osg::Group;
	m_denseCloud = new osg::Group;
	m_faces = new osg::Group;
	m_arImageViewNode = new osg::Group;
	m_dataMgrWidget->setFlightGnssLine(m_flightGnssLine);
	m_dataMgrWidget->setFlightVisualLine(m_flightVisualLine);
	m_dataMgrWidget->setSparse(m_sparseCloud);
	m_dataMgrWidget->setDense(m_denseCloud);
	m_dataMgrWidget->setFaces(m_faces);


	m_osgWidget = new OsgWidget(this);

	this->setCentralWidget(m_osgWidget);

	connect(m_osgWidget, SIGNAL(initialized()), this, SLOT(initOsg()));

	connect(this, SIGNAL(joUAVConnectStatusChangedSignal(int)),
		this, SLOT(on_joUAVConnectStatusChanged(int)),
		Qt::ConnectionType::QueuedConnection);

	connect(m_uavConDialog, SIGNAL(disconnect()),
		this, SLOT(on_connDialog_disconnect()));

	connect(m_uavConDialog, SIGNAL(minimized()),
		this, SLOT(on_connDialog_minimized()));
}

MainWindow::~MainWindow()
{
	delete ui;
	if (exivNode)
	{
		delete exivNode;
	}
	if (vtkNode)
	{
		delete vtkNode;
	}
	if (pose_estimation_stageNode)
	{
		delete pose_estimation_stageNode;
	}
	if (densification_stageNode)
	{
		delete densification_stageNode;
	}
	if (surface_generation_stageNode)
	{
		delete surface_generation_stageNode;
	}
	if (ortho_rectification_stageNode)
	{
		delete ortho_rectification_stageNode;
	}
	if (mosaicing_stageNode)
	{
		delete mosaicing_stageNode;
	}
	if (scene3DNode)
	{
		delete scene3DNode;
	}
	if (frameViewNode)
	{
		delete frameViewNode;
	}
}

void MainWindow::displayTrackedImage(const cv::Mat& img)
{
	QImage image;
	cv::Mat img_copy = img.clone();
	const unsigned char* data = img_copy.data;
	int width = img_copy.cols;
	int height = img_copy.rows;
	int bytesPerLine = static_cast<int>(img_copy.step);
	switch (img_copy.type())
	{
		//8 bit , ARGB
	case CV_8UC4:
		image = QImage(data, width, height, bytesPerLine, QImage::Format_ARGB32);
		break;
		//8 bit BGR
	case CV_8UC3:
		image = QImage(data, width, height, bytesPerLine, QImage::Format_RGB888).rgbSwapped();
		//swap blue and red channel
		//return image.rgbSwapped();
		break;
		//8 bit Gray shale
	case CV_8UC1:
		image = QImage(data, width, height, bytesPerLine, QImage::Format_Grayscale8);
		break;
	default:
		//qWarning() << "Unsupported cv::Mat type:" << img.type() << ", Empty QImage will be returned!";
		break;
	}

	if (!image.isNull())
	{
		//m_frameWidget->setFrameImage(QPixmap::fromImage(image));
		m_frameWidget->setFrameImage(image);
	}
}

void MainWindow::joUAVConnectStatusChanged(int status)
{
	emit joUAVConnectStatusChangedSignal(status);
}

void MainWindow::followFrameViewChanged(const osg::Matrix& view_mat,
	int cam_w, int cam_h, double f, double cx, double cy)
{
	if (m_followFrameView == 1)
	{
		osg::Vec3d eye, center, up;
		view_mat.getLookAt(eye, center, up);
		m_trackball_manipulator->setTransformation(eye, center, up);


		{
			osg::Camera* vcam = this->m_osgWidget->getOsgViewer()->getCamera();
			osg::Viewport* viewport = vcam->getViewport();

			// 计算投影矩阵
			osg::Matrix proj_mat = osg::Matrix::identity();
			// 计算投影矩阵
			double view_ratio = 1.0 * viewport->width() / viewport->height();
			double cam_ratio = 1.0 * cam_w / cam_h;
			double fovy, fovx;
			if (view_ratio >= cam_ratio)
			{
				fovy = 2.0 * atan((cam_h / 2.0) / f);
				fovx = fovy * view_ratio;
			}
			else
			{
				fovx = 2.0 * atan((cam_w / 2.0) / f);
				fovy = fovx / view_ratio;
			}

			double zNear = 0.0001;
			double zFar = 10000.0;

			double left = -zNear * tan(fovy / 2.0) * view_ratio;
			double right = zNear * tan(fovy / 2.0) * view_ratio;
			double bottom = -zNear * tan(fovy / 2.0);
			double top = zNear * tan(fovy / 2.0);
			proj_mat.makeFrustum(left * 1.2, right * 1.2, bottom * 1.2, top * 1.2, zNear, zFar);
			vcam->setProjectionMatrix(proj_mat);
		}

	}
	else if (m_followFrameView == 2)
	{
		osg::Vec3d eye0, center0, up0;
		m_terrain_manipulator->getTransformation(eye0, center0, up0);
		osg::Vec3 vec0 = center0 - eye0;
		vec0.normalize();

		osg::Vec3d eye, center, up;
		view_mat.getLookAt(eye, center, up);
		osg::Vec3 vec = center - eye;
		vec.normalize();
		center = eye;

		double dist = (eye0 - eye).length();
		static double MAX_LOOK_DISTANCE = 10000;
		if (dist > MAX_LOOK_DISTANCE) dist = MAX_LOOK_DISTANCE;

		eye = center - (vec0 * dist);

		m_terrain_manipulator->setTransformation(eye, center, up0);
	}
}

void MainWindow::on_action_connectDrone_triggered()
{
	UAVImageTransDialog UAVDialog;
	if (UAVDialog.exec() == QDialog::Accepted)
	{
		FrameWorkConfig frameCfg = UAVDialog.getFrameWorkConfig();

		if (frameCfg.valid())
		{
			m_frameWorkCfg = frameCfg;

			this->m_connAddrLabel->setText(m_frameWorkCfg.url);
			this->m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/no_connection_16px.png"));
			this->m_nodeStatus->setText(QStringLiteral("就绪"));
			this->setWindowTitle(m_frameWorkCfg.url);

			ui->action_start->setEnabled(true);
			ui->action_pause->setEnabled(false);
			ui->action_resume->setEnabled(false);
			ui->action_stop->setEnabled(false);

			m_stageDirDateTime = QString();
			ui->action_openProduction->setEnabled(false);
		}
	}
}

void MainWindow::on_action_start_triggered()
{
	FrameWorkConfig::createProfiles(m_frameWorkCfg);

	//create_exivNode();
	create_vtkNode();
	create_scene3DNode();
	create_frameViewNode();
	create_pose_estimation_stageNode();
	create_densification_stageNode();
	create_surface_generation_stageNode();
	create_ortho_rectification_stageNode();
	create_mosaicing_stageNode();

	//if(exivNode) exivNode->startThread();
	if (vtkNode) vtkNode->startThread();
	if (pose_estimation_stageNode) pose_estimation_stageNode->startThread();
	if (densification_stageNode) densification_stageNode->startThread();
	if (surface_generation_stageNode) surface_generation_stageNode->startThread();
	if (ortho_rectification_stageNode) ortho_rectification_stageNode->startThread();
	if (mosaicing_stageNode) mosaicing_stageNode->startThread();
	if (scene3DNode) scene3DNode->startThread();
	if (frameViewNode) frameViewNode->startThread();


	ui->action_start->setEnabled(false);
	ui->action_pause->setEnabled(true);
	ui->action_resume->setEnabled(false);
	ui->action_stop->setEnabled(true);
	ui->action_flowFrameCenter->setEnabled(true);
	ui->action_flowFrameView->setEnabled(true);

	m_uavConDialog->setWindowModality(Qt::NonModal);
	m_uavConDialog->showNormal();
}

void MainWindow::on_action_pause_triggered()
{
	//if(exivNode) exivNode->startThread();
	if (vtkNode) vtkNode->pause();
	if (pose_estimation_stageNode) pose_estimation_stageNode->pause();
	if (densification_stageNode) densification_stageNode->pause();
	if (surface_generation_stageNode) surface_generation_stageNode->pause();
	if (ortho_rectification_stageNode) ortho_rectification_stageNode->pause();
	if (mosaicing_stageNode) mosaicing_stageNode->pause();

	this->m_nodeStatus->setText(QStringLiteral("暂停"));

	ui->action_start->setEnabled(false);
	ui->action_pause->setEnabled(false);
	ui->action_resume->setEnabled(true);
	ui->action_stop->setEnabled(true);
}

void MainWindow::on_action_resume_triggered()
{
	//if(exivNode) exivNode->resume();
	if (vtkNode) vtkNode->resume();
	if (pose_estimation_stageNode) pose_estimation_stageNode->resume();
	if (densification_stageNode) densification_stageNode->resume();
	if (surface_generation_stageNode) surface_generation_stageNode->resume();
	if (ortho_rectification_stageNode) ortho_rectification_stageNode->resume();
	if (mosaicing_stageNode) mosaicing_stageNode->resume();

	this->m_nodeStatus->setText(QStringLiteral("正在处理..."));

	ui->action_start->setEnabled(false);
	ui->action_pause->setEnabled(true);
	ui->action_resume->setEnabled(false);
	ui->action_stop->setEnabled(true);
}

void MainWindow::on_action_stop_triggered()
{
	/*if (exivNode)
	{
		exivNode->cancel();
		delete exivNode;
		exivNode = nullptr;
	};*/
	if (vtkNode)
	{
		vtkNode->cancel();
		delete vtkNode;
		vtkNode = nullptr;
	};
	if (pose_estimation_stageNode)
	{
		pose_estimation_stageNode->cancel();
		delete pose_estimation_stageNode;
		pose_estimation_stageNode = nullptr;
	}
	if (densification_stageNode) {
		densification_stageNode->cancel();
		delete densification_stageNode;
		densification_stageNode = nullptr;
	}
	if (surface_generation_stageNode) {
		surface_generation_stageNode->cancel();
		delete surface_generation_stageNode;
		surface_generation_stageNode = nullptr;
	}
	if (ortho_rectification_stageNode) {
		ortho_rectification_stageNode->cancel();
		delete ortho_rectification_stageNode;
		ortho_rectification_stageNode = nullptr;
	}
	if (mosaicing_stageNode) {
		mosaicing_stageNode->cancel();
		delete mosaicing_stageNode;
		mosaicing_stageNode = nullptr;
	}
	if (scene3DNode) {
		scene3DNode->cancel();
		delete scene3DNode;
		scene3DNode = nullptr;
	}
	if (frameViewNode) {
		frameViewNode->cancel();
		delete frameViewNode;
		frameViewNode = nullptr;
	}


	ui->action_start->setEnabled(true);
	ui->action_pause->setEnabled(false);
	ui->action_resume->setEnabled(false);
	ui->action_stop->setEnabled(false);
	ui->action_flowFrameCenter->setEnabled(false);
	ui->action_flowFrameView->setEnabled(false);

	ui->action_homeView->trigger();
}

void MainWindow::create_exivNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["config/id"] = "camera_1";
	node_paras["config/input"] = "F:/OpenREALM2/data/edm_big_overlap_50p";
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/rate"] = "10";
	node_paras["config/opt/working_directory"] = "F:/OpenREALM2/data";

	exivNode = new MyREALM::Exiv2GrabberNode(node_paras);
}

void MainWindow::create_vtkNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["config/id"] = "camera_1";
	node_paras["config/input"] = m_frameWorkCfg.url.toLocal8Bit().toStdString();
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/rate"] = QString::number(m_frameWorkCfg.rate).toStdString();
	node_paras["config/gps_delay_ms"] = QString::number(m_frameWorkCfg.gps_delay_ms).toStdString();
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	vtkNode = new MyREALM::JonFmvTKNode(node_paras);
	MyREALM::JoUAVConnectCLB joUAVConCLB = std::bind(&MainWindow::joUAVConnectStatusChanged,
		this, std::placeholders::_1);
	vtkNode->bindJoUAVConnectCLB(joUAVConCLB);
	//joUAVConnectStatusChanged
}

void MainWindow::create_scene3DNode()
{
	this->m_flightGnssLine->removeChildren(0, m_flightGnssLine->getNumChildren());
	this->m_flightVisualLine->removeChildren(0, m_flightVisualLine->getNumChildren());
	this->m_sparseCloud->removeChildren(0, m_sparseCloud->getNumChildren());
	this->m_faces->removeChildren(0, m_faces->getNumChildren());

	MyREALM::NodeParas node_paras;
	node_paras["config/id"] = "camera_1";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	scene3DNode = new MyREALM::Scene3DNode(node_paras);

	osg::Node* visualTraj = scene3DNode->visualTrajNode();
	osg::Node* gnssTraj = scene3DNode->gnssTrajNode();
	osg::Node* sparse = scene3DNode->sparseNode();
	osg::Node* dense = scene3DNode->denseNode();
	osg::Node* mesh = scene3DNode->meshNode();
	osg::Node* visualPoseFrustum= scene3DNode->visualPoseFrustumNode();
	osg::Node* gnssPoseFrustum = scene3DNode->gnssPoseFrustumNode();
	osg::Node* arNode = scene3DNode->arNode();


	this->m_arImageViewNode->addChild(arNode);
	this->m_flightVisualLine->addChild(visualTraj);
	this->m_flightGnssLine->addChild(gnssTraj);
	this->m_flightVisualLine->addChild(visualPoseFrustum);
	this->m_flightGnssLine->addChild(gnssPoseFrustum);
	this->m_sparseCloud->addChild(sparse);
	this->m_denseCloud->addChild(dense);
	this->m_faces->addChild(mesh);
	
	//MyREALM::FollowFrameViewCLB followCLB = std::bind(&MainWindow::followFrameViewChanged,
	//	this, std::placeholders::_1);
	//scene3DNode->bindFollowFrameViewCLB(followCLB);
}

void MainWindow::create_frameViewNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["config/id"] = "camera_1";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	frameViewNode = new MyREALM::FrameViewNode(node_paras);
	MyREALM::SubImageFun subImageFunc = std::bind(&MainWindow::displayTrackedImage, this, std::placeholders::_1);
	frameViewNode->bindDisplayTrackedImageFunc(subImageFunc);
}

void MainWindow::create_pose_estimation_stageNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["stage/type"] = "pose_estimation";
	node_paras["stage/master"] = "true";
	node_paras["topics/input/frame"] = "/realm/camera_1/input";
	node_paras["topics/output"] = "/realm/camera_1/pose_estimation/frame";
	node_paras["config/id"] = "camera_1";
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/method"] = "open_vslam";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	pose_estimation_stageNode = new  MyREALM::StageNode(node_paras);

	m_stageDirDateTime = QDir::toNativeSeparators(QString(pose_estimation_stageNode->outputDirDatetime().c_str()));
	ui->action_openProduction->setEnabled(true);
}

void MainWindow::create_densification_stageNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["stage/type"] = "densification";
	node_paras["topics/input/frame"] = "/realm/camera_1/pose_estimation/frame";
	node_paras["topics/output"] = "/realm/camera_1/densification/frame";
	node_paras["config/id"] = "camera_1";
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/method"] = "psl";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	densification_stageNode = new  MyREALM::StageNode(node_paras);
}

void MainWindow::create_surface_generation_stageNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["stage/type"] = "surface_generation";
	node_paras["topics/input/frame"] = "/realm/camera_1/densification/frame";
	node_paras["topics/output"] = "/realm/camera_1/surface_generation/frame";
	node_paras["config/id"] = "camera_1";
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	surface_generation_stageNode = new  MyREALM::StageNode(node_paras);
}

void MainWindow::create_ortho_rectification_stageNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["stage/type"] = "ortho_rectification";
	node_paras["topics/input/frame"] = "/realm/camera_1/surface_generation/frame";
	node_paras["topics/output"] = "/realm/camera_1/ortho_rectification/frame";
	node_paras["config/id"] = "camera_1";
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	ortho_rectification_stageNode = new  MyREALM::StageNode(node_paras);
}

void MainWindow::create_mosaicing_stageNode()
{
	MyREALM::NodeParas node_paras;
	node_paras["stage/type"] = "mosaicing";
	node_paras["topics/input/frame"] = "/realm/camera_1/ortho_rectification/frame";
	node_paras["config/id"] = "camera_1";
	node_paras["config/profile"] = "tjh_reco";
	node_paras["config/method"] = "open_vslam";
	node_paras["config/opt/working_directory"] = m_frameWorkCfg.workDir.toLocal8Bit().toStdString();

	mosaicing_stageNode = new  MyREALM::StageNode(node_paras);
}

void MainWindow::createWorkDirectory()
{
}

void MainWindow::setupStatusBar()
{
	m_nodeStatus = new QLabel(QStringLiteral("就绪"), this);
	m_connStatusLabel = new QLabel(QStringLiteral("无连接"), this);
	m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/no_connection_16px.png"));
	m_connStatusLabel->setToolTip(QStringLiteral("无连接"));
	m_connAddrLabel = new QLabel(QStringLiteral("未连接图传"), this);


	this->statusBar()->addWidget(m_nodeStatus);
	this->statusBar()->addWidget(m_connStatusLabel);
	this->statusBar()->addWidget(m_connAddrLabel);
}

void MainWindow::on_action_flowFrameView_triggered(bool check)
{
	ui->action_flowFrameCenter->setChecked(false);
	m_followFrameView = check ? 1 : 0;
	m_flightLineRoot->setNodeMask(1);

	if (!scene3DNode) { return; }

	if (m_followFrameView == 1)
	{
		MyREALM::FollowFrameViewCLB followCLB = std::bind(&MainWindow::followFrameViewChanged,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
			std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
		scene3DNode->bindFollowFrameViewCLB(followCLB);
		m_osgWidget->getOsgViewer()->setCameraManipulator(m_trackball_manipulator.get());
		m_flightLineRoot->setNodeMask(0);
	}
	else
	{
		scene3DNode->bindFollowFrameViewCLB(NULL);
		m_osgWidget->getOsgViewer()->setCameraManipulator(m_terrain_manipulator.get());
	}

}

void MainWindow::on_action_flowFrameCenter_triggered(bool check)
{
	ui->action_flowFrameView->setChecked(false);
	m_followFrameView = check ? 2 : 0;
	m_flightLineRoot->setNodeMask(1);

	if (!scene3DNode) { return; }

	if (m_followFrameView == 2)
	{
		MyREALM::FollowFrameViewCLB followCLB = std::bind(&MainWindow::followFrameViewChanged,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
			std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
		m_osgWidget->getOsgViewer()->setCameraManipulator(m_terrain_manipulator.get());
		scene3DNode->bindFollowFrameViewCLB(followCLB);
	}
	else
	{
		scene3DNode->bindFollowFrameViewCLB(NULL);
		m_osgWidget->getOsgViewer()->setCameraManipulator(m_terrain_manipulator.get());
	}
}

void MainWindow::on_action_homeView_triggered()
{
	ui->action_flowFrameView->setChecked(false);
	ui->action_flowFrameCenter->setChecked(false);
	m_followFrameView = 0;
	m_flightLineRoot->setNodeMask(1);
	m_osgWidget->getOsgViewer()->setCameraManipulator(m_terrain_manipulator.get());
	osg::BoundingSphere bs = m_sceneRoot->computeBound();
	osg::Vec3 center = bs.center();
	float r =  bs.radius();

	osg::Vec3 vec(0, 0, r * 2.0);
	osg::Vec3 eye = center + vec;
	osg::Vec3 up(0.0, 1.0, 0.0);

	m_terrain_manipulator->setHomePosition(eye, center, up);


	m_terrain_manipulator->home(0);
}

void MainWindow::on_action_openProduction_triggered()
{
	if (!this->m_stageDirDateTime.isEmpty())
	{
		const QString explorer = "explorer";
		QStringList param;
		param << this->m_stageDirDateTime;
		QProcess::startDetached(explorer, param);
	}
}

void MainWindow::on_action_about_triggered()
{
	QMessageBox::about(this, QStringLiteral("关于"),
		QStringLiteral("%1\n\n版本: %2\n\n版权所有: Copyright (C) 2022 %3")
		.arg(qApp->applicationDisplayName())
		.arg(qApp->applicationVersion())
		.arg(qApp->organizationName()));
}

void MainWindow::on_joUAVConnectStatusChanged(int status)
{
	static QMap<int, QString> status_labels = {
		{0,QStringLiteral("未连接.")},
		{1,QStringLiteral("连接中...")},
		{2,QStringLiteral("连接成功，准备接收航摄视频图像...")},
		{3,QStringLiteral("连接失败!")},
		{4,QStringLiteral("连接成功，接收到航摄视频图像!")},
	};

	if (status == MyREALM::JonFmvTKNode::ConnectSuccess)
	{

		this->m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/signal_16px.png"));
		this->m_connStatusLabel->setToolTip(QStringLiteral("已连接"));
		this->m_nodeStatus->setText(QStringLiteral("正在处理..."));
	}
	else if (status == MyREALM::JonFmvTKNode::ConnectFailed)
	{
		this->m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/no_connection_16px.png"));
		this->m_connStatusLabel->setToolTip(QStringLiteral("连接失败"));
		this->m_nodeStatus->setText(QStringLiteral("无处理"));
	}
	else if (status == MyREALM::JonFmvTKNode::Disconnect)
	{
		this->m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/no_connection_16px.png"));
		this->m_connStatusLabel->setToolTip(QStringLiteral("连接断开"));
		this->m_nodeStatus->setText(QStringLiteral("未开始"));
	}
	else if (status == MyREALM::JonFmvTKNode::Connecting)
	{
		this->m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/signal_medium_16px.png"));
		this->m_connStatusLabel->setToolTip(QStringLiteral("连接中"));
		this->m_nodeStatus->setText(QStringLiteral("待处理..."));
	}
	else if (status == MyREALM::JonFmvTKNode::FrameRecieved)
	{
		m_uavConDialog->hideAfter(2);
		this->m_connStatusLabel->setPixmap(QPixmap(":/images/resource/images/signal_medium_16px.png"));
		this->m_connStatusLabel->setToolTip(QStringLiteral("已连接"));
		this->m_nodeStatus->setText(QStringLiteral("正在处理..."));
	}
	
	QString sta = status_labels.value(status, QStringLiteral("未知状态"));
	m_uavConDialog->setText(sta);

}

void MainWindow::on_connDialog_disconnect()
{
	this->on_action_stop_triggered();
	m_uavConDialog->hide();
}

void MainWindow::on_connDialog_minimized()
{
	this->showMinimized();
}

void MainWindow::initOsg()
{
	osgViewer::Viewer* pViewer = m_osgWidget->getOsgViewer();

	pViewer->getCamera()->getGraphicsContext()->getTraits();

	m_sceneRoot = new osg::Group;
	m_flightLineRoot->addChild(m_flightGnssLine);
	m_flightLineRoot->addChild(m_flightVisualLine);
	m_sceneRoot->addChild(m_flightLineRoot);
	m_sceneRoot->addChild(m_sparseCloud);
	m_sceneRoot->addChild(m_denseCloud);
	m_sceneRoot->addChild(m_faces);
	m_sceneRoot->addChild(m_arImageViewNode);
	pViewer->setSceneData(m_sceneRoot.get());
	pViewer->addEventHandler(new osgViewer::StatsHandler());
	pViewer->addEventHandler(new osgViewer::WindowSizeHandler());
	pViewer->addEventHandler(new osgViewer::ThreadingHandler());
	pViewer->addEventHandler(new osgGA::StateSetManipulator(
		pViewer->getCamera()->getOrCreateStateSet()));

	m_osgWidget->getOsgViewer()->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

	m_trackball_manipulator = new osgGA::TrackballManipulator;
	m_terrain_manipulator = new osgGA::TerrainManipulator;

	m_osgWidget->getOsgViewer()->setCameraManipulator(m_terrain_manipulator.get(), true);

	osg::Vec3 eye(50, -50, 150);
	osg::Vec3 center(0, 0, 90);
	osg::Vec3 up(0, 0, 1);

	m_terrain_manipulator->setHomePosition(eye, center, up);

	m_terrain_manipulator->home(0);
}
