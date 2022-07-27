#ifndef FRAMEWIDGET_H
#define FRAMEWIDGET_H

#include <QWidget>
#include <QPixmap>

#include <QImage>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <deque>

namespace Ui {
class FrameWidget;
}

class FetchFrameThread;

class FrameWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FrameWidget(QWidget *parent = nullptr);
    ~FrameWidget();

    void setFrameImage(const QImage& img);

    QSize sizeHint() const override;

public slots:
    void on_timer_timeout();

private:
    Ui::FrameWidget *ui;
    QPixmap m_pixmap;
    QTimer* m_timer;
    FetchFrameThread* m_thread;
};



class FetchFrameThread :public QThread
{
    Q_OBJECT
private:
    std::deque<QImage> m_images;
    int m_max_que_size;
    QMutex m_mutex;
    bool m_done;

public:
    FetchFrameThread(QObject* parent = Q_NULLPTR);

    void updateImage(const QImage& img);


    bool getImage(QImage& img);

    int cancel();

protected:
    void run() override;

};

#endif // FRAMEWIDGET_H
