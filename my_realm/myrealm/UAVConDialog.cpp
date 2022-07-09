#include "UAVConDialog.h"
#include "ui_UAVConDialog.h"
#include <QMessageBox>

UAVConDialog::UAVConDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::UAVConDialog)
{
    ui->setupUi(this); 
    m_hide_secs = 0;
    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(on_timer_timeout()));
}

UAVConDialog::~UAVConDialog()
{
    delete ui;
}

void UAVConDialog::setText(const QString& text)
{
    ui->label_connStatus->setText(text);
}

void UAVConDialog::hideAfter(int seconds)
{
    m_hide_secs = seconds;
    m_timer->start(1000);
}

void UAVConDialog::on_pushButton_disConn_clicked()
{
    if (this->isVisible())
    {
        if (QMessageBox::question(this, this->windowTitle(),
            QStringLiteral("确定断开图传连接?")) == QMessageBox::Yes)
        {
            emit disconnect();
        }
    }
}

void UAVConDialog::on_pushButton_minWin_clicked()
{
    emit minimized();
}

void UAVConDialog::on_timer_timeout()
{
    m_hide_secs--;
    if (m_hide_secs <= 0)
    {
        m_timer->stop();
        this->hide();
    }
}

void UAVConDialog::closeEvent(QCloseEvent* event)
{
    if (this->isVisible())
    {
        if (QMessageBox::question(this, this->windowTitle(),
            QStringLiteral("确定断开图传连接?")) == QMessageBox::Yes)
        {
            emit disconnect();
        }
        else
        {
            event->ignore();
            return;
        }
    }
    QDialog::closeEvent(event);
}
