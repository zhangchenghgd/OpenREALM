#include "FrameWidget.h"
#include "ui_FrameWidget.h"
#include <QResizeEvent>
#include <QPainter>

FrameWidget::FrameWidget(QWidget *parent) :
    QWidget(parent), m_pixmap(0, 0),
    ui(new Ui::FrameWidget)
{
    ui->setupUi(this);
    ui->label_frame->setAlignment(Qt::AlignCenter);
}

FrameWidget::~FrameWidget()
{
    delete ui;
}

void FrameWidget::setFrameImage(const QPixmap& pixmap)
{
    m_pixmap = pixmap.scaled(ui->label_frame->size() - QSize(4, 4), 
        Qt::KeepAspectRatio, Qt::SmoothTransformation);

    ui->label_frame->setPixmap(m_pixmap);
}

QSize FrameWidget::sizeHint() const
{
    return QSize(422, 200);
}
