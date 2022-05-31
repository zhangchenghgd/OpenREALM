#ifndef DATAMANAGERWIDGET_H
#define DATAMANAGERWIDGET_H

#include <QWidget>
#include <QTreeWidgetItem>
#include <osg/Group>

namespace Ui {
class DataManagerWidget;
}

class DataManagerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DataManagerWidget(QWidget *parent = nullptr);
    ~DataManagerWidget();

    void setFlightGnssLine(osg::Node* node);
    void setFlightVisualLine(osg::Node* node);
    void setFaces(osg::Node* node);

public slots:
    void on_treeWidget_itemChanged(QTreeWidgetItem* item, int column);

private:
    void updateTreeItems();

private:
    Ui::DataManagerWidget *ui;
    osg::Node* m_flightGnssLine;
    osg::Node* m_flightVisualLine;
    osg::Node* m_faces;
};

#endif // DATAMANAGERWIDGET_H
