#include "WorkProfiles.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDir>
#include <QStandardPaths>
#include <QApplication>

bool copyDirectoryFiles(const QString& fromDir,
	const QString& toDir, bool coverFileIfExist);

bool copyFileToPath(QString sourceDir, QString toDir, bool coverFileIfExist);

FrameWorkConfig::FrameWorkConfig()
{
	url = "UDP://127.0.0.1:55037";
	gps_delay_ms = 0;
	rate = 10;
	frameSize = QSize(0, 0);
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	distortMatrix = cv::Mat::zeros(5, 1, CV_64F);
	workDir = "F:\\TJH_Video_Recon_Workspace";
}

QString FrameWorkConfig::url_protocol() const
{
	QStringList spls = url.split("://", Qt::SkipEmptyParts);
	if (spls.count() > 0)
	{
		QString pro_str = spls[0].trimmed();
		if (pro_str.compare("UDP", Qt::CaseInsensitive) == 0)
		{
			return "UDP";
		}
		else if (pro_str.compare("TCP", Qt::CaseInsensitive) == 0)
		{
			return "TCP";
		}
	}

	return QString();
}

QString FrameWorkConfig::url_addr() const
{
	QStringList spls = url.split("://", Qt::SkipEmptyParts);
	if (spls.count() > 1)
	{
		QStringList addr_spl = spls[1].split(":", Qt::SkipEmptyParts);
		if (addr_spl.count() > 0)
		{
			return addr_spl[0];
		}
	}

	return QString();
}

int FrameWorkConfig::url_port() const
{
	QStringList spls = url.split(":", Qt::SkipEmptyParts);
	if (spls.count() == 3)
	{
		QString port_str = spls[2].trimmed();
		return port_str.toInt();
	}
	return 0;
}

bool FrameWorkConfig::importCaliFile(const QString& filename)
{
	cv::FileStorage fs_read(filename.toLocal8Bit().toStdString(), cv::FileStorage::READ);  // 读取文件数据
	if (!fs_read.isOpened())
	{
		return false;
	}

	std::string date;
	//second method:use FileNode::operator >>
	fs_read["calibration_Time"] >> date;

	int w = fs_read["image_Width"];
	int h = fs_read["image_Height"];

	frameSize = QSize(w, h);

	fs_read["Camera_Matrix"] >> this->cameraMatrix;
	fs_read["Distortion_Coefficients"] >> this->distortMatrix;

	fs_read.release();

}

bool FrameWorkConfig::valid() const
{
	return !this->url_addr().isEmpty()
		&& this->url_port() > 0
		&& !frameSize.isEmpty()
		&& !workDir.isEmpty();
}

QString FrameWorkConfig::getWorkspaceDir()
{
	QString workspace = QString("%1/TJH_REALM/configs/%2")
		.arg(QStandardPaths
			::writableLocation(QStandardPaths::HomeLocation))
		.arg(qApp->applicationVersion());

	QDir workspace_dir(workspace);
	if (!workspace_dir.exists())
	{
		workspace_dir.mkpath(workspace);
	}

	return workspace;
}

bool FrameWorkConfig::load(const QString& cfg_name,
	FrameWorkConfig& work_cfg)
{
	QString filename = QString("%1/%2.json")
		.arg(getWorkspaceDir()).arg(cfg_name);
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << QString("Open project file \"%1\" failed!")
			.arg(filename);
		return false;
	}

	QByteArray content = file.readAll();
	file.close();

	QJsonParseError* error = new QJsonParseError;
	QJsonDocument doc = QJsonDocument::fromJson(content, error);

	//判断文件是否完整
	if (error->error != QJsonParseError::NoError)
	{
		qDebug() << "Parse json error: " << error->errorString();
		return false;
	}
	QJsonObject obj = doc.object();        //获取对象

	work_cfg.url = obj.value("url").toString();
	work_cfg.gps_delay_ms = obj.value("gps_delay_ms").toInt();
	work_cfg.rate = obj.value("rate").toInt(10);
	int w = obj.value("frame_size_width").toInt();
	int h = obj.value("frame_size_height").toInt();
	work_cfg.frameSize = QSize(w, h);

	QJsonArray cam_mat_arr = obj.value("camera_matrix").toArray();
	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			work_cfg.cameraMatrix.at<double>(r, c) =
				cam_mat_arr[r * 3 + c].toDouble();
		}
	}

	QJsonArray distort_arr = obj.value("distort_matrix").toArray();
	for (int r = 0; r < 5; r++)
	{
		work_cfg.distortMatrix.at<double>(r, 0) =
			distort_arr[r].toDouble();
	}

	work_cfg.workDir = obj.value("work_dir").toString();

	return true;
}

bool FrameWorkConfig::save(const FrameWorkConfig& work_cfg,
	const QString& cfg_name)
{
	QString filename = QString("%1/%2.json")
		.arg(getWorkspaceDir()).arg(cfg_name);
	QJsonObject obj;
	obj.insert("url", work_cfg.url);
	obj.insert("gps_delay_ms", work_cfg.gps_delay_ms);
	obj.insert("rate", work_cfg.rate);
	obj.insert("frame_size_width", work_cfg.frameSize.width());
	obj.insert("frame_size_height", work_cfg.frameSize.height());
	QJsonArray cam_mat_arr;
	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			cam_mat_arr.append(
				work_cfg.cameraMatrix.at<double>(r, c));
		}
	}
	obj.insert("camera_matrix", cam_mat_arr);

	QJsonArray distort_arr;
	for (int r = 0; r < 5; r++)
	{
		distort_arr.append(
			work_cfg.distortMatrix.at<double>(r, 0));
	}
	obj.insert("distort_matrix", distort_arr);

	obj.insert("work_dir", work_cfg.workDir);

	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly))
	{
		qDebug() << QString("Write project file \"%1\" failed!")
			.arg(filename);
		return false;
	}

	QJsonDocument doc(obj);

	// 文本保存
	QTextStream ts(&file);
	ts.setCodec("utf-8");
	ts << doc.toJson(QJsonDocument::Indented);
	ts.flush();

	file.close();

	return true;
}

QList<QString> FrameWorkConfig::listFrameWorkConfigFilenames()
{
	QString workspace = getWorkspaceDir();

	QDir workspace_dir(workspace);
	QStringList filters = { "*.json" };
	QStringList wlist = workspace_dir.entryList(filters, QDir::Files, QDir::Time);
	QStringList wlist2;
	QStringListIterator lit(wlist);
	while (lit.hasNext())
	{
		wlist2 << QFileInfo(lit.next()).completeBaseName();
	}


	return wlist2;
}

FrameWorkConfig FrameWorkConfig::createFrameWorkConfig(
	const QString& config_name)
{
	FrameWorkConfig work_cfg;

	save(work_cfg, config_name);

	return work_cfg;
}

bool FrameWorkConfig::removeFrameWorkConfig(const QString& config_name)
{
	QString filename = QString("%1/%2.json")
		.arg(getWorkspaceDir()).arg(config_name);

	if (QFile::exists(filename))
	{
		return QFile::remove(filename);
	}

	return false;
}

bool FrameWorkConfig::renameFrameWorkConfig(
	const QString& old_config_name, const QString& new_config_name)
{

	QString old_filename = QString("%1/%2.json")
		.arg(getWorkspaceDir()).arg(old_config_name);


	QString new_filename = QString("%1/%2.json")
		.arg(getWorkspaceDir()).arg(new_config_name);

	if (QFile::exists(old_filename))
	{
		return QFile::rename(old_filename, new_filename);
	}

	return false;
}

bool FrameWorkConfig::createProfiles(const FrameWorkConfig& work_cfg)
{
	QString profiles_dirname = QString("%1/profiles").arg(work_cfg.workDir);

	QString src_folder = QString("%1/profiles").arg(qApp->applicationDirPath());

	copyDirectoryFiles(src_folder, profiles_dirname, true);

	QString calib_path = QString("%1/tjh_reco/camera/calib.yaml").arg(profiles_dirname);
	QFile calib_file(calib_path);
	if (!calib_file.open(QIODevice::WriteOnly))
	{
		qDebug() << QString("Write calib file \"%1\" failed!")
			.arg(calib_path);
		return false;
	}


	// 文本保存
	QTextStream ts(&calib_file);
	ts.setCodec("utf-8");
	ts << QString(
		"%YAML:1.0                                       \n"\
		"#--------------------------------------         \n"\
		"# Camera Parameters for pinhole camera          \n"\
		"#--------------------------------------         \n"\
		"# Type of the camera model                      \n"\
		"type: pinhole                                   \n"\
		"# Frames per second                             \n"\
		"fps: 10.0                                       \n"\
		"# Image size                                    \n"\
		"width: %1                                     \n"\
		"height: %2                                    \n"\
		"# Principal point                               \n"\
		"cx: %3                                  \n"\
		"cy: %4                                  \n"\
		"# Focal length                                  \n"\
		"fx: %5                                 \n"\
		"fy: %6                                 \n"\
		"# Distortion                                    \n"\
		"k1: %7                                   \n"\
		"k2: %8                                    \n"\
		"p1: %9                                   \n"\
		"p2: %10                                   \n"\
		"k3: %11                                         \n")
		.arg(work_cfg.frameSize.width())
		.arg(work_cfg.frameSize.height())
		.arg(work_cfg.cameraMatrix.at<double>(0, 2), 0, 'f', 8)
		.arg(work_cfg.cameraMatrix.at<double>(1, 2), 0, 'f', 8)
		.arg(work_cfg.cameraMatrix.at<double>(0, 0), 0, 'f', 8)
		.arg(work_cfg.cameraMatrix.at<double>(1, 1), 0, 'f', 8)
		.arg(work_cfg.distortMatrix.at<double>(0, 0), 0, 'f', 10)
		.arg(work_cfg.distortMatrix.at<double>(1, 0), 0, 'f', 10)
		.arg(work_cfg.distortMatrix.at<double>(2, 0), 0, 'f', 10)
		.arg(work_cfg.distortMatrix.at<double>(3, 0), 0, 'f', 10)
		.arg(work_cfg.distortMatrix.at<double>(4, 0), 0, 'f', 10);


	ts.flush();
	calib_file.close();


	return true;
}



//拷贝文件：  
bool copyFileToPath(QString sourceDir, QString toDir, bool coverFileIfExist)
{
	toDir.replace("\\", "/");
	if (sourceDir == toDir) {
		return true;
	}
	if (!QFile::exists(sourceDir)) {
		return false;
	}
	QDir* createfile = new QDir;
	bool exist = createfile->exists(toDir);
	if (exist) {
		if (coverFileIfExist) {
			createfile->remove(toDir);
		}
	}//end if  

	if (!QFile::copy(sourceDir, toDir))
	{
		return false;
	}
	return true;
}

//拷贝文件夹：  
bool copyDirectoryFiles(const QString& fromDir, const QString& toDir, bool coverFileIfExist)
{
	QDir sourceDir(fromDir);
	QDir targetDir(toDir);
	if (!targetDir.exists()) {    /**< 如果目标目录不存在，则进行创建 */
		if (!targetDir.mkpath(targetDir.absolutePath()))
			return false;
	}

	QFileInfoList fileInfoList = sourceDir.entryInfoList();
	foreach(QFileInfo fileInfo, fileInfoList) {
		if (fileInfo.fileName() == "." || fileInfo.fileName() == "..")
			continue;

		if (fileInfo.isDir()) {    /**< 当为目录时，递归的进行copy */
			if (!copyDirectoryFiles(fileInfo.filePath(),
				targetDir.filePath(fileInfo.fileName()),
				coverFileIfExist))
				return false;
		}
		else {            /**< 当允许覆盖操作时，将旧文件进行删除操作 */
			if (coverFileIfExist && targetDir.exists(fileInfo.fileName())) {
				targetDir.remove(fileInfo.fileName());
			}

			/// 进行文件copy  
			if (!QFile::copy(fileInfo.filePath(),
				targetDir.filePath(fileInfo.fileName()))) {
				return false;
			}
		}
	}
	return true;
}