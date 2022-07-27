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
#include "UpdateGridMapThread.h"
#include "TileMesh.h"

using namespace std;

int main(int argc, char* argv[])
{
	QCoreApplication app(argc, argv);

	std::string dom_path = "F:/OpenREALM2/lzd/dom/ortho.jpg";
	std::string dem_path = "F:/OpenREALM2/lzd/dsm/elevation.tif";
	std::string lod_dirname = "F:/OpenREALM2/lzd/lod";

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

	MyREALM::TileMesh::Ptr tile_mesh = std::make_shared<MyREALM::TileMesh>(
		osg::Vec2(1023.0, 1023.0), 500.0, 0.8, 3, lod_dirname);

	/*MyREALM::TileMeshUpdateThread* tileThread = new MyREALM::TileMeshUpdateThread(
		osg::Vec2(0, 0), 3940.0, 0.8, 5, lod_dirname);*/

	MyREALM::UpdateGridMapThread* updateGridThread = new MyREALM::UpdateGridMapThread(
		dem_path, dom_path, tile_mesh.get());

	/*osg::ref_ptr<MyREALM::TileMeshUpdateCallback> tileCLB = new  MyREALM::TileMeshUpdateCallback(tileThread);
	
	group->setUpdateCallback(tileCLB);*/

	group->addChild(tile_mesh->meshNode());

	viewer.setSceneData(group);
	viewer.addEventHandler(new osgViewer::ThreadingHandler);
	viewer.addEventHandler(new osgViewer::HelpHandler);
	viewer.addEventHandler(new osgViewer::StatsHandler);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
	viewer.setCameraManipulator(new osgGA::TerrainManipulator);

	viewer.setUpViewInWindow(100, 100, 800, 600);
	//viewer.getCamera()->setClearColor(osg::Vec4(1., 1., 1., 1.0));

	//osgUtil::Optimizer optimizer;
	//optimizer.optimize(group.get());
	//viewer.realize();

	//tileThread->startThread();
	updateGridThread->startThread();
	

	viewer.run();

	return EXIT_SUCCESS;
}
