#ifndef TEETH_VIEWER_OSG_WIDGET_HPP
#define TEETH_VIEWER_OSG_WIDGET_HPP

#include <QWidget>
#include <QSlider>
#include <QAction>
#include <QMap>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osgQOpenGL/osgQOpenGLWidget>

class OsgWidget : public osgQOpenGLWidget
{
	Q_OBJECT
public:
	OsgWidget(QWidget* parent = Q_NULLPTR);

	~OsgWidget();

	QSize sizeHint() const override;

};


#endif // !TEETH_VIEWER_OSG_WIDGET_HPP
