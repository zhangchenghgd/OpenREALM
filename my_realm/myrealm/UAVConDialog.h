#ifndef UAVCONDIALOG_H
#define UAVCONDIALOG_H

#include <QDialog>
#include <QCloseEvent>
#include <QTimer>

namespace Ui {
class UAVConDialog;
}

class UAVConDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UAVConDialog(QWidget *parent = nullptr);
    ~UAVConDialog();

    void setText(const QString& text);

    void hideAfter(int seconds);

signals:
    void disconnect();
    void minimized();

protected slots:
    void on_pushButton_disConn_clicked();
    void on_pushButton_minWin_clicked();
    void on_timer_timeout();

protected:
    void closeEvent(QCloseEvent* event);

private:
    Ui::UAVConDialog *ui;
    QTimer* m_timer;
    int m_hide_secs;
};

#endif // UAVCONDIALOG_H
