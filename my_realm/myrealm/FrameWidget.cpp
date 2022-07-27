#include "FrameWidget.h"
#include "ui_FrameWidget.h"
#include <QResizeEvent>
#include <QPainter>



FetchFrameThread::FetchFrameThread(QObject* parent)
	:QThread(parent), m_done(false), m_max_que_size(2)
{

}

void FetchFrameThread::updateImage(const QImage& img)
{
	QMutexLocker locker(&m_mutex);
	if (m_images.size() > m_max_que_size)
	{
		m_images.erase(m_images.begin(), m_images.begin() + 1);
	}

	m_images.push_back(img);
}


bool FetchFrameThread::getImage(QImage& img)
{
	QMutexLocker locker(&m_mutex);
	if (m_images.size() > 0)
	{
		img = m_images.front();
		m_images.pop_front();
		return true;
	}
	return false;
}

int FetchFrameThread::cancel()
{
	m_done = true;
	while (isRunning()) { yieldCurrentThread(); }
	return 0;
}

void FetchFrameThread::run()
{
	m_done = false;
	do
	{
		yieldCurrentThread();
		if (this->isInterruptionRequested())
		{
			m_done = true;
		}
	} while (!m_done);
}


FrameWidget::FrameWidget(QWidget* parent) :
	QWidget(parent), m_pixmap(0, 0),
	ui(new Ui::FrameWidget)
{
	ui->setupUi(this);
	ui->label_frame->setAlignment(Qt::AlignCenter);

	m_thread = new FetchFrameThread(this);
	m_timer = new QTimer(this);
	m_timer->setInterval(200);
	connect(m_timer, &QTimer::timeout, this, &FrameWidget::on_timer_timeout);
	m_thread->start();
	m_timer->start();
}

FrameWidget::~FrameWidget()
{
	m_thread->cancel();
	m_timer->stop();

	delete ui;
}

void FrameWidget::setFrameImage(const QImage& img)
{
	/*m_pixmap = pixmap.scaled(ui->label_frame->size() - QSize(4, 4),
		Qt::KeepAspectRatio, Qt::SmoothTransformation);

	ui->label_frame->setPixmap(m_pixmap);*/

	m_thread->updateImage(img);
}

QSize FrameWidget::sizeHint() const
{
	return QSize(422, 200);
}

void FrameWidget::on_timer_timeout()
{
	QImage img;
	if (m_thread->getImage(img))
	{
		QPixmap pixmap = QPixmap::fromImage(img);
		m_pixmap = pixmap.scaled(ui->label_frame->size() - QSize(4, 4),
			Qt::KeepAspectRatio, Qt::SmoothTransformation);

		ui->label_frame->setPixmap(m_pixmap);
	}
}
