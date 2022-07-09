#include <iostream>
#include <vector>
#include <string>
#include <gdal_priv.h>
#include <osg/Node>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>
#include <osgUtil/Optimizer>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Layer>
#include <QCoreApplication>
#include "createHeightField.h"
#include "FacesSceneNode.h"

using namespace std;


int main(int argc, char* argv[])
{
	QCoreApplication app(argc, argv);

	std::string dom_path = "F:/OpenREALM2/osgTerrainMesh/data/dom_10cm.jpg";
	std::string dem_path = "F:/OpenREALM2/osgTerrainMesh/data/dem_10cm.tif";

	GDALAllRegister();
	OGRRegisterAll();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");   //设置支持中文路径 

	std::string gdal_data_path = QString("%1/gdal")
		.arg(app.applicationDirPath()).toLocal8Bit().toStdString();
	CPLSetConfigOption("GDAL_DATA", gdal_data_path.c_str());

	const char* newprojs[1] = {
		QString("%1/proj4").arg(app.applicationDirPath()).toLocal8Bit().toStdString().c_str() };

	char** proj_paths = NULL;
	proj_paths = CSLAddString(proj_paths,
		QString("%1/proj4").arg(app.applicationDirPath()).toLocal8Bit().toStdString().c_str());

	OSRSetPROJSearchPaths(proj_paths);
	CSLDestroy(proj_paths);

	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> group = new osg::Group;

	//group->addChild(createHeightField2(dem_path, dom_path));

	MyREALM2::FacesReceiverThread* meshThread = new MyREALM2::FacesReceiverThread;
	osg::ref_ptr<MyREALM2::FacesDrawCallback> meshCLB = new  MyREALM2::FacesDrawCallback(meshThread);
	osg::ref_ptr<osg::Geode> mesh = MyREALM2::createFacesGeode(meshCLB);

	group->addChild(mesh);

	viewer.setSceneData(group);
	viewer.addEventHandler(new osgViewer::ThreadingHandler);
	viewer.addEventHandler(new osgViewer::HelpHandler);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.setCameraManipulator(new osgGA::TerrainManipulator);

	viewer.setUpViewInWindow(100, 100, 800, 600);
	viewer.getCamera()->setClearColor(osg::Vec4(0., 0., 0., 1.0));

	osgUtil::Optimizer optimizer;
	optimizer.optimize(group.get());
	viewer.realize();

	meshThread->startThread();

	return viewer.run();

	//return app.exec();
}
