#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QLabel>
#include <osg/Group>
#include <osgGA/TerrainManipulator>
#include <osgGA/TrackballManipulator>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "WorkProfiles.h"

namespace Ui {
class MainWindow;
}

namespace MyREALM {
    class Exiv2GrabberNode;
    class JonFmvTKNode;
    class StageNode;
    class Scene3DNode;
    class FrameViewNode;
}

class OsgWidget;

class FrameWidget;

class DataManagerWidget;

class UAVConDialog;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void displayTrackedImage(const cv::Mat& img);

    void joUAVConnectStatusChanged(int status);

    void followFrameViewChanged(const osg::Matrix& view_mat, 
        int width, int height, double focal, double cx, double cy);

signals:
    void joUAVConnectStatusChangedSignal(int status);

private slots:
    void initOsg();
    void on_action_connectDrone_triggered();
    void on_action_start_triggered();
    void on_action_pause_triggered();
    void on_action_resume_triggered();
    void on_action_stop_triggered();
    void on_action_homeView_triggered();
    void on_action_flowFrameView_triggered(bool check);
    void on_action_flowFrameCenter_triggered(bool check);
    void on_action_openProduction_triggered();
    void on_action_about_triggered();
    void on_joUAVConnectStatusChanged(int status);
    void on_connDialog_disconnect();
    void on_connDialog_minimized();

protected:
    void create_exivNode();
    void create_vtkNode();
    void create_scene3DNode();
    void create_frameViewNode();
    void create_pose_estimation_stageNode();
    void create_densification_stageNode();
    void create_surface_generation_stageNode();
    void create_ortho_rectification_stageNode();
    void create_mosaicing_stageNode();
    void createWorkDirectory();
    void setupStatusBar();

private:
    Ui::MainWindow *ui;
    OsgWidget* m_osgWidget;
    FrameWidget* m_frameWidget;
    DataManagerWidget* m_dataMgrWidget;
    UAVConDialog* m_uavConDialog;

    QDockWidget* m_dataMgrDockWidget;
    QDockWidget* m_frameDockWidget;

    QLabel* m_nodeStatus;
    QLabel* m_connStatusLabel;
    QLabel* m_connAddrLabel;
    int m_followFrameView;
    
    osg::ref_ptr<osgGA::TerrainManipulator> m_terrain_manipulator;
    osg::ref_ptr<osgGA::TrackballManipulator> m_trackball_manipulator;
    osg::ref_ptr<osg::Group> m_sceneRoot;
    osg::ref_ptr<osg::Group> m_flightLineRoot;
    osg::ref_ptr<osg::Group> m_flightGnssLine;
    osg::ref_ptr<osg::Group> m_flightVisualLine;
    osg::ref_ptr<osg::Group> m_arImageViewNode;
    osg::ref_ptr<osg::Group> m_sparseCloud;
    osg::ref_ptr<osg::Group> m_denseCloud;
    osg::ref_ptr<osg::Group> m_faces;
    MyREALM::Exiv2GrabberNode* exivNode;
    MyREALM::JonFmvTKNode* vtkNode;
    MyREALM::StageNode* pose_estimation_stageNode;
    MyREALM::StageNode* densification_stageNode;
    MyREALM::StageNode* surface_generation_stageNode;
    MyREALM::StageNode* ortho_rectification_stageNode;
    MyREALM::StageNode* mosaicing_stageNode;
    MyREALM::Scene3DNode* scene3DNode;
    MyREALM::FrameViewNode* frameViewNode;

    FrameWorkConfig m_frameWorkCfg;
    QString m_stageDirDateTime;

};

#endif // MAINWINDOW_H
