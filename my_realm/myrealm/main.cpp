#include "MyREALM.h"
#include <QApplication>
#include <QSplashScreen>
#include <QFile>
#include <QTextStream>
#include <QTextCodec>
#include <QTranslator>
#include <QDir>
#include <QDirIterator>
#include <QDebug>
#include <QPixmap>
#include <QStandardPaths>
#include "MainWindow.h"

#include <iostream>
#include <thread>
#include <glog/logging.h>
#include "grabber_exiv2_node.h"
#include "StageNode.h"
#include "Scene3DNode.h"


void setupLogDir(const QString& log_dirname)
{
	QDir log_dir(log_dirname);
	if (!log_dir.exists())
	{
		log_dir.mkpath(log_dirname);
	}
}

int main(int argc, char* argv[])
{
	QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
	QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);


	Q_INIT_RESOURCE(myrealm);

	QTextCodec* xcodec = QTextCodec::codecForLocale();
	QString exePath = xcodec->toUnicode(QByteArray(argv[0]));
	QString EXE_DIR = QFileInfo(exePath).path();
	QString pluginsDir = QString("%1/plugins").arg(EXE_DIR);
	//QString resourcesDir = QString("%1/resources").arg(EXE_DIR);
	QStringList libpaths;
	libpaths << EXE_DIR;
	libpaths << pluginsDir;
	//libpaths << resourcesDir;
	libpaths << QApplication::libraryPaths();
	QApplication::setLibraryPaths(libpaths);

	

	QApplication a(argc, argv);

	QString appPath = a.applicationDirPath();
	QString pluginsPath = QString("%1/plugins").arg(appPath);
	a.addLibraryPath(pluginsPath);
	a.setApplicationName(QStringLiteral("TJH_REALM"));
	a.setApplicationDisplayName(QStringLiteral("无人机视频实时重建系统"));
	a.setOrganizationName(QStringLiteral("武汉天际航信息科技股份有限公司"));
	a.setApplicationVersion("1.0");
	a.setWindowIcon(QIcon(":/logo/resource/logo/logo.ico"));

	GDALAllRegister();
	OGRRegisterAll();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");   //设置支持中文路径 

	QString log_dirname = QString("%1/TJH_REALM/logs/")
		.arg(QStandardPaths
			::writableLocation(QStandardPaths::HomeLocation));
	setupLogDir(log_dirname);
	google::InitGoogleLogging(argv[0]);
	google::SetLogDestination(google::GLOG_INFO, log_dirname.toLocal8Bit().toStdString().c_str());
	google::SetStderrLogging(google::GLOG_INFO);

	std::string gdal_data_path = QString("%1/gdal").arg(EXE_DIR).toLocal8Bit().toStdString();
	CPLSetConfigOption("GDAL_DATA", gdal_data_path.c_str());

	const char* newprojs[1] = {
		QString("%1/proj4").arg(EXE_DIR).toLocal8Bit().toStdString().c_str() };

	char** proj_paths = NULL;
	proj_paths = CSLAddString(proj_paths,
		QString("%1/proj4").arg(EXE_DIR).toLocal8Bit().toStdString().c_str());

	OSRSetPROJSearchPaths(proj_paths);
	CSLDestroy(proj_paths);

	
	QSplashScreen splashScreen;
	splashScreen.setPixmap(QPixmap(":/images/resource/images/logo.png"));
	splashScreen.show();

	MainWindow w;
	w.showMaximized();
	splashScreen.finish(&w);

	return a.exec();
}
