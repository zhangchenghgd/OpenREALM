#include "UAVImageTransDialog.h"
#include "ui_UAVImageTransDialog.h"
#include <QFileDialog>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QInputDialog>

UAVImageTransDialog::UAVImageTransDialog(QWidget *parent) :
	QDialog(parent), m_currentFramwWorkCfgName(),
	ui(new Ui::UAVImageTransDialog)
{
	ui->setupUi(this);

	QDoubleValidator* bdl_valid = new QDoubleValidator(this);
	QIntValidator* int_valid = new QIntValidator(1,10000, this);
	ui->lineEdit_width->setValidator(int_valid);
	ui->lineEdit_height->setValidator(int_valid);
	ui->lineEdit_fx->setValidator(bdl_valid);
	ui->lineEdit_fy->setValidator(bdl_valid);
	ui->lineEdit_cx->setValidator(bdl_valid);
	ui->lineEdit_cy->setValidator(bdl_valid);
	ui->lineEdit_fx->setValidator(bdl_valid);
	ui->lineEdit_k1->setValidator(bdl_valid);
	ui->lineEdit_k2->setValidator(bdl_valid);
	ui->lineEdit_k3->setValidator(bdl_valid);
	ui->lineEdit_p1->setValidator(bdl_valid);
	ui->lineEdit_p2->setValidator(bdl_valid);

	this->on_pushButton_resetCal_clicked();

	ui->lineEdit_address->setValidator(
		new QRegExpValidator
		(
			QRegExp("^((2[0-4]\\d|25[0-5]|[01]?\\d\\d?)\\.){3}(2[0-4]\\d|25[0-5]|[01]?\\d\\d?)$")
		));
	// 用于占位
	//ui->lineEdit_address->setInputMask("000.000.000.000;0");
	
	on_comboBox_frameWorkCfgNames_currentIndexChanged(QString());

	this->updateFrameWorkCfgNames();
	this->updateFrameWorkCfg();
}

UAVImageTransDialog::~UAVImageTransDialog()
{
	delete ui;
}

FrameWorkConfig UAVImageTransDialog::getFrameWorkConfig()
{
	return m_frameWork;
}

void UAVImageTransDialog::on_pushButton_importCal_clicked()
{
	QString filename = QFileDialog::getOpenFileName(
		this, QStringLiteral("选择opencv格式相机标定参数文件"), 
		QDir::homePath(), QStringLiteral("OpenCV 相机标定文件 (*.xml *.yml)"));

	if (!QFile::exists(filename))
	{
		return;
	}

	m_frameWork.importCaliFile(filename);

	connectTextChangedSlots(false);
	this->updateCal();
	connectTextChangedSlots(true);
	this->saveFrameWorkCfg();
}

void UAVImageTransDialog::on_pushButton_resetCal_clicked()
{
	connectTextChangedSlots(false);
	m_frameWork.frameSize = QSize(0, 0);
	m_frameWork.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	m_frameWork.distortMatrix = cv::Mat::zeros(5, 1, CV_64F);
	this->updateCal();
	connectTextChangedSlots(true);

	this->saveFrameWorkCfg();
}

void UAVImageTransDialog::on_pushButton_selectWorkDir_clicked()
{
	QString dir0 = ui->lineEdit_workDir->text().trimmed().isEmpty() ? 
		QDir::homePath() : ui->lineEdit_workDir->text();
	QString dirname = QFileDialog::getExistingDirectory(this, QStringLiteral("选择文件夹"), dir0);

	if (!dirname.isEmpty())
	{
		ui->lineEdit_workDir->setText(dirname);
	}
}

void UAVImageTransDialog::on_comboBox_frameWorkCfgNames_currentIndexChanged(QString text)
{
	if (!m_currentFramwWorkCfgName.isEmpty())
	{
		saveFrameWorkCfg();
		m_currentFramwWorkCfgName = QString();
		m_frameWork = FrameWorkConfig();
	}

	this->updateFrameWorkCfg();

	if (!text.isEmpty())
	{
		if(FrameWorkConfig::load(text, m_frameWork))
		{
			m_currentFramwWorkCfgName = text;
			this->updateFrameWorkCfg();
		}
	}

	bool enable = !m_currentFramwWorkCfgName.isEmpty();
	ui->groupBox_protocol->setEnabled(enable);
	ui->groupBox_cam->setEnabled(enable);
	ui->frame_workDir->setEnabled(enable);
	ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(enable);
}

void UAVImageTransDialog::on_pushButton_newFrameWorkCfg_clicked()
{
	QString text = QInputDialog::getText(this, this->windowTitle(),
		QStringLiteral("输入配置名称"), QLineEdit::Normal, QStringLiteral("新配置 - %1")
		.arg(ui->comboBox_frameWorkCfgNames->count() + 1));

	if (text.trimmed().isEmpty())
	{
		return;
	}
	
	for (int i = 0; i < ui->comboBox_frameWorkCfgNames->count(); ++i)
	{
		if (ui->comboBox_frameWorkCfgNames->itemText(i).compare(text, Qt::CaseInsensitive) == 0)
		{
			QMessageBox::information(this, this->windowTitle(),
				QStringLiteral("配置名称\"%1\"已经存在，请使用其它名称!").arg(text));
			return;
		}
	}
	
	m_frameWork = FrameWorkConfig::createFrameWorkConfig(text);

	ui->comboBox_frameWorkCfgNames->insertItem(0,text);

	ui->comboBox_frameWorkCfgNames->setCurrentIndex(0);
}

void UAVImageTransDialog::on_pushButton_delFrameWorkCfg_clicked()
{
	if (m_currentFramwWorkCfgName.isEmpty()) { return; }

	if (QMessageBox::question(this, this->windowTitle(), QStringLiteral("确定要删除配置\"%1\"?")
		.arg(m_currentFramwWorkCfgName)) != QMessageBox::Yes)
	{
		return;
	}

	int cur_item_idx = ui->comboBox_frameWorkCfgNames->currentIndex();

	FrameWorkConfig::removeFrameWorkConfig(m_currentFramwWorkCfgName);
	m_currentFramwWorkCfgName = QString();
	ui->comboBox_frameWorkCfgNames->removeItem(cur_item_idx);
	ui->comboBox_frameWorkCfgNames->setCurrentIndex(0);
}

void UAVImageTransDialog::on_pushButton_renameFrameWorkCfg_clicked()
{
	if (m_currentFramwWorkCfgName.isEmpty()) { return; }

	QString text = QInputDialog::getText(this, this->windowTitle(),
		QStringLiteral("重命名配置"), QLineEdit::Normal, m_currentFramwWorkCfgName);

	if (text.trimmed().isEmpty())
	{
		return;
	}
	int cur_item_idx = ui->comboBox_frameWorkCfgNames->currentIndex();
	FrameWorkConfig::renameFrameWorkConfig(m_currentFramwWorkCfgName, text);
	m_currentFramwWorkCfgName = text;
	ui->comboBox_frameWorkCfgNames->setItemText(cur_item_idx, text);
}

void UAVImageTransDialog::on_input_textChanged(const QString& txt)
{
	this->saveFrameWorkCfg();
}

void UAVImageTransDialog::accept()
{
	if (m_currentFramwWorkCfgName.isEmpty())
	{
		QMessageBox::information(this, this->windowTitle(), QStringLiteral("未选择预定义配置!"));
		return;
	}

	if (ui->lineEdit_address->text().isEmpty())
	{
		QMessageBox::information(this, this->windowTitle(), QStringLiteral("连接地址不能为空!"));
		return;
	}

	if (ui->lineEdit_width->text().isEmpty() || ui->lineEdit_height->text().isEmpty())
	{
		QMessageBox::information(this, this->windowTitle(), QStringLiteral("分辨率不能为空!"));
		return;
	}

	if (ui->lineEdit_fx->text().isEmpty() ||
		ui->lineEdit_fy->text().isEmpty() ||
		ui->lineEdit_cx->text().isEmpty() ||
		ui->lineEdit_cy->text().isEmpty() ||
		ui->lineEdit_k1->text().isEmpty() ||
		ui->lineEdit_k2->text().isEmpty() ||
		ui->lineEdit_k3->text().isEmpty() ||
		ui->lineEdit_p1->text().isEmpty() ||
		ui->lineEdit_p2->text().isEmpty())
	{
		QMessageBox::information(this, this->windowTitle(), QStringLiteral("相机标定参数不能为空!"));
		return;
	}

	if (ui->lineEdit_workDir->text().isEmpty())
	{
		QMessageBox::information(this, this->windowTitle(), QStringLiteral("工作文件夹不能为空!"));
		return;
	}


	m_frameWork.url = QString("%1://%2:%3").arg(ui->comboBox_protocol->currentText().toLower())
		.arg(ui->lineEdit_address->text()).arg(ui->spinBox_port->value());
	m_frameWork.gps_delay_ms = ui->spinBox_gpsDelayMs->value();
	m_frameWork.rate = ui->spinBox_rate->value();

	m_frameWork.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	m_frameWork.cameraMatrix.at<double>(0, 0) = ui->lineEdit_fx->text().toDouble();
	m_frameWork.cameraMatrix.at<double>(1, 1) = ui->lineEdit_fy->text().toDouble();
	m_frameWork.cameraMatrix.at<double>(0, 2) = ui->lineEdit_cx->text().toDouble();
	m_frameWork.cameraMatrix.at<double>(1, 2) = ui->lineEdit_cy->text().toDouble();

	m_frameWork.distortMatrix = cv::Mat::zeros(5, 1, CV_64F);
	m_frameWork.distortMatrix.at<double>(0, 0) = ui->lineEdit_k1->text().toDouble();
	m_frameWork.distortMatrix.at<double>(1, 0) = ui->lineEdit_k2->text().toDouble();
	m_frameWork.distortMatrix.at<double>(4, 0) = ui->lineEdit_k3->text().toDouble();
	m_frameWork.distortMatrix.at<double>(2, 0) = ui->lineEdit_p1->text().toDouble();
	m_frameWork.distortMatrix.at<double>(3, 0) = ui->lineEdit_p2->text().toDouble();

	m_frameWork.frameSize = QSize(
		ui->lineEdit_width->text().toInt(), 
		ui->lineEdit_height->text().toInt());

	
	m_frameWork.workDir = QDir::toNativeSeparators(ui->lineEdit_workDir->text());

	QDialog::accept();
}

void UAVImageTransDialog::reject()
{
	m_frameWork = FrameWorkConfig();
	QDialog::reject();
}

void UAVImageTransDialog::connectTextChangedSlots(bool conn)
{
	if (conn)
	{
		connect(ui->comboBox_protocol, SIGNAL(currentTextChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_address, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->spinBox_port, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->spinBox_gpsDelayMs, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->spinBox_rate, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_width, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_height, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_fx, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_fy, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_cx, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_cy, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_k1, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_k2, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_k3, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_p1, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_p2, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		connect(ui->lineEdit_workDir, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
	}
	else
	{
		disconnect(ui->comboBox_protocol, SIGNAL(currentTextChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_address, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->spinBox_port, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->spinBox_gpsDelayMs, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->spinBox_rate, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_width, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_height, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_fx, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_fy, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_cx, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_cy, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_k1, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_k2, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_k3, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_p1, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_p2, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
		disconnect(ui->lineEdit_workDir, SIGNAL(textChanged(const QString&)), this, SLOT(on_input_textChanged(const QString&)));
	}
}

void UAVImageTransDialog::updateFrameWorkCfgNames()
{
	QStringList names = FrameWorkConfig::listFrameWorkConfigFilenames();

	ui->comboBox_frameWorkCfgNames->clear();
	ui->comboBox_frameWorkCfgNames->addItems(names);
	
	if (names.count() > 0)
	{
		ui->comboBox_frameWorkCfgNames->setCurrentIndex(0);
	}
}

void UAVImageTransDialog::updateFrameWorkCfg()
{
	connectTextChangedSlots(false);
	if (m_currentFramwWorkCfgName.isEmpty())
	{
		ui->comboBox_protocol->setCurrentText("UDP");
		ui->lineEdit_address->setText("127.0.0.1");
		ui->spinBox_port->setValue(0);
		ui->spinBox_gpsDelayMs->setValue(0);
		ui->spinBox_rate->setValue(10);
		this->updateCal();
		ui->lineEdit_workDir->setText("");
	}
	else
	{
		ui->comboBox_protocol->setCurrentText(m_frameWork.url_protocol());
		ui->lineEdit_address->setText(m_frameWork.url_addr());
		ui->spinBox_port->setValue(m_frameWork.url_port());
		ui->spinBox_gpsDelayMs->setValue(m_frameWork.gps_delay_ms);
		ui->spinBox_rate->setValue(m_frameWork.rate);
		this->updateCal();
		ui->lineEdit_workDir->setText(m_frameWork.workDir);
	}
	connectTextChangedSlots(true);
}

void UAVImageTransDialog::updateCal()
{
	if (m_currentFramwWorkCfgName.isEmpty())
	{
		ui->lineEdit_width->setText("0");
		ui->lineEdit_height->setText("0");
		ui->lineEdit_fx->setText("0");
		ui->lineEdit_fy->setText("0");
		ui->lineEdit_cx->setText("0");
		ui->lineEdit_cy->setText("0");

		ui->lineEdit_k1->setText("0");
		ui->lineEdit_k2->setText("0");
		ui->lineEdit_k3->setText("0");
		ui->lineEdit_p1->setText("0");
		ui->lineEdit_p2->setText("0");
	}
	else
	{
		ui->lineEdit_width->setText(QString::number(m_frameWork.frameSize.width()));
		ui->lineEdit_height->setText(QString::number(m_frameWork.frameSize.height()));
		ui->lineEdit_fx->setText(QString::number(m_frameWork.cameraMatrix.at<double>(0, 0), 'f', 6));
		ui->lineEdit_fy->setText(QString::number(m_frameWork.cameraMatrix.at<double>(1, 1), 'f', 6));
		ui->lineEdit_cx->setText(QString::number(m_frameWork.cameraMatrix.at<double>(0, 2), 'f', 6));
		ui->lineEdit_cy->setText(QString::number(m_frameWork.cameraMatrix.at<double>(1, 2), 'f', 6));

		ui->lineEdit_k1->setText(QString::number(m_frameWork.distortMatrix.at<double>(0, 0), 'f', 10));
		ui->lineEdit_k2->setText(QString::number(m_frameWork.distortMatrix.at<double>(1, 0), 'f', 10));
		ui->lineEdit_k3->setText(QString::number(m_frameWork.distortMatrix.at<double>(4, 0), 'f', 10));
		ui->lineEdit_p1->setText(QString::number(m_frameWork.distortMatrix.at<double>(2, 0), 'f', 10));
		ui->lineEdit_p2->setText(QString::number(m_frameWork.distortMatrix.at<double>(3, 0), 'f', 10));
	}
}

void UAVImageTransDialog::saveFrameWorkCfg()
{
	if (m_currentFramwWorkCfgName.isEmpty()) { return; }
	m_frameWork.url = QString("%1://%2:%3").arg(ui->comboBox_protocol->currentText())
		.arg(ui->lineEdit_address->text()).arg(ui->spinBox_port->value());

	m_frameWork.gps_delay_ms = ui->spinBox_gpsDelayMs->value();
	m_frameWork.rate = ui->spinBox_rate->value();

	m_frameWork.cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	m_frameWork.cameraMatrix.at<double>(0, 0) = ui->lineEdit_fx->text().toDouble();
	m_frameWork.cameraMatrix.at<double>(1, 1) = ui->lineEdit_fy->text().toDouble();
	m_frameWork.cameraMatrix.at<double>(0, 2) = ui->lineEdit_cx->text().toDouble();
	m_frameWork.cameraMatrix.at<double>(1, 2) = ui->lineEdit_cy->text().toDouble();

	m_frameWork.distortMatrix = cv::Mat::zeros(5, 1, CV_64F);
	m_frameWork.distortMatrix.at<double>(0, 0) = ui->lineEdit_k1->text().toDouble();
	m_frameWork.distortMatrix.at<double>(1, 0) = ui->lineEdit_k2->text().toDouble();
	m_frameWork.distortMatrix.at<double>(4, 0) = ui->lineEdit_k3->text().toDouble();
	m_frameWork.distortMatrix.at<double>(2, 0) = ui->lineEdit_p1->text().toDouble();
	m_frameWork.distortMatrix.at<double>(3, 0) = ui->lineEdit_p2->text().toDouble();

	m_frameWork.frameSize = QSize(
		ui->lineEdit_width->text().toInt(),
		ui->lineEdit_height->text().toInt());


	m_frameWork.workDir = QDir::toNativeSeparators(ui->lineEdit_workDir->text());

	FrameWorkConfig::save(m_frameWork, m_currentFramwWorkCfgName);

}
