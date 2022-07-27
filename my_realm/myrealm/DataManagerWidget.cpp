#include "DataManagerWidget.h"
#include "ui_DataManagerWidget.h"

DataManagerWidget::DataManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataManagerWidget),
    m_flightGnssLine(nullptr),
    m_flightVisualLine(nullptr), m_sparse(nullptr),m_dense(nullptr),
    m_faces(nullptr)
{
    ui->setupUi(this);
    this->updateTreeItems();
    ui->treeWidget->expandAll();
}

DataManagerWidget::~DataManagerWidget()
{
    delete ui;
}

void DataManagerWidget::setFlightGnssLine(osg::Node* node)
{
    m_flightGnssLine = node;
}

void DataManagerWidget::setFlightVisualLine(osg::Node* node)
{
    m_flightVisualLine = node;
}

void DataManagerWidget::setSparse(osg::Node* node)
{
    m_sparse = node;
}

void DataManagerWidget::setDense(osg::Node* node)
{
    m_dense = node;
}

void DataManagerWidget::setFaces(osg::Node* node)
{
    m_faces = node;
}

void DataManagerWidget::on_treeWidget_itemChanged(
    QTreeWidgetItem* item, int column)
{
    QVariant item_data = item->data(0, Qt::UserRole);
    if (item_data.isNull()) { return; }

    int v = item_data.toInt();
    unsigned int msk = item->checkState(0) == Qt::Checked ? 1 : 0;

    if (v == 1)
    {
        m_flightGnssLine->setNodeMask(msk);
    }
    else if (v == 2)
    {
        m_flightVisualLine->setNodeMask(msk);
    }
    else if (v == 3)
    {
        m_sparse->setNodeMask(msk);
    }
    else if (v == 4)
    {
        m_dense->setNodeMask(msk);
    }
    else if (v == 5)
    {
        m_faces->setNodeMask(msk);
    }

}

void DataManagerWidget::updateTreeItems()
{
    ui->treeWidget->clear();

    QTreeWidgetItem* root_item = new QTreeWidgetItem(ui->treeWidget);
    root_item->setText(0, QStringLiteral("图层"));
    root_item->setIcon(0, QIcon(":/images/resource/images/layers_16px.png"));

    QTreeWidgetItem* gnss_line_item = new QTreeWidgetItem;
    gnss_line_item->setText(0, QStringLiteral("GNSS航线"));
    gnss_line_item->setIcon(0, QIcon(":/images/resource/images/green_plane_16px.png"));
    gnss_line_item->setCheckState(0, Qt::Checked);
    gnss_line_item->setData(0, Qt::UserRole, 1);

    QTreeWidgetItem* visual_line_item = new QTreeWidgetItem;
    visual_line_item->setText(0, QStringLiteral("重建航线"));
    visual_line_item->setIcon(0, QIcon(":/images/resource/images/red_plane_16px.png"));
    visual_line_item->setCheckState(0, Qt::Checked);
    visual_line_item->setData(0, Qt::UserRole, 2);

    QTreeWidgetItem* sparse_item = new QTreeWidgetItem;
    sparse_item->setText(0, QStringLiteral("点云(估算)"));
    sparse_item->setIcon(0, QIcon(":/images/resource/images/points_yellow_30px.png"));
    sparse_item->setCheckState(0, Qt::Checked);
    sparse_item->setData(0, Qt::UserRole, 3);

    QTreeWidgetItem* dense_item = new QTreeWidgetItem;
    dense_item->setText(0, QStringLiteral("点云(过滤)"));
    dense_item->setIcon(0, QIcon(":/images/resource/images/points_red_30px.png"));
    dense_item->setCheckState(0, Qt::Checked);
    dense_item->setData(0, Qt::UserRole, 4);

    QTreeWidgetItem* faces_item = new QTreeWidgetItem;
    faces_item->setText(0, QStringLiteral("重建模型"));
    faces_item->setIcon(0, QIcon(":/images/resource/images/earth_element_16px.png"));
    faces_item->setCheckState(0, Qt::Checked);
    faces_item->setData(0, Qt::UserRole, 5);

    root_item->addChild(gnss_line_item);
    root_item->addChild(visual_line_item);
    root_item->addChild(sparse_item);
    root_item->addChild(dense_item);
    root_item->addChild(faces_item);
    ui->treeWidget->addTopLevelItem(root_item);

}
