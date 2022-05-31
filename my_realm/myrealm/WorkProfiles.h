#ifndef WORKPROFILES_H
#define WORKPROFILES_H

#include <QObject>
#include <QSize>
#include <QList>
#include <opencv2/opencv.hpp>

struct FrameWorkConfig
{
	QString url;
	int gps_delay_ms;
	int rate;
	QSize frameSize;
	cv::Mat cameraMatrix;
	cv::Mat distortMatrix;
	QString workDir;

	FrameWorkConfig();

	QString url_protocol() const;

	QString url_addr() const;

	int url_port() const;

	bool importCaliFile(const QString& filename);

	bool valid() const;

	static QString getWorkspaceDir();

	static bool load(const QString& cfg_name, 
		 FrameWorkConfig& work_cfg);

	static bool save(const FrameWorkConfig& work_cfg, 
		const QString& cfg_name);

	static QList<QString> listFrameWorkConfigFilenames();

	static FrameWorkConfig createFrameWorkConfig(const QString& config_name);

	static bool removeFrameWorkConfig(const QString& config_name);

	static bool renameFrameWorkConfig(const QString& old_config_name, 
		const QString& new_config_name);


	static bool createProfiles(const FrameWorkConfig& work_cfg);

};



#endif  // !WORKPROFILES_H