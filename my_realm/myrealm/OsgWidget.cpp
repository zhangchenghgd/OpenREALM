#include "OsgWidget.h"

OsgWidget::OsgWidget(QWidget* parent):osgQOpenGLWidget(parent)
{
}

OsgWidget::~OsgWidget()
{
}

QSize OsgWidget::sizeHint() const
{
	return QSize(this->width(), this->height());
}
