#ifndef FRAMEWIDGET_H
#define FRAMEWIDGET_H

#include <QWidget>
#include <QPixmap>

namespace Ui {
class FrameWidget;
}

class FrameWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FrameWidget(QWidget *parent = nullptr);
    ~FrameWidget();

    void setFrameImage(const QPixmap& pixmap);

    QSize sizeHint() const override;

private:
    Ui::FrameWidget *ui;
    QPixmap m_pixmap;
};

#endif // FRAMEWIDGET_H
