#ifndef UAVIMAGETRANSDIALOG_H
#define UAVIMAGETRANSDIALOG_H

#include <QDialog>
#include <opencv2/opencv.hpp>
#include "WorkProfiles.h"

namespace Ui {
class UAVImageTransDialog;
}

class UAVImageTransDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UAVImageTransDialog(QWidget *parent = nullptr);
    ~UAVImageTransDialog();

    FrameWorkConfig getFrameWorkConfig();

public slots:

    void on_pushButton_importCal_clicked();
    void on_pushButton_resetCal_clicked();
    void on_pushButton_selectWorkDir_clicked();

    void on_comboBox_frameWorkCfgNames_currentIndexChanged(QString text);
    void on_pushButton_newFrameWorkCfg_clicked();
    void on_pushButton_delFrameWorkCfg_clicked();
    void on_pushButton_renameFrameWorkCfg_clicked();

    void on_input_textChanged(const QString& txt);


    void accept();
    void reject();

protected:
    void connectTextChangedSlots(bool conn);

    void updateFrameWorkCfgNames();

    void updateFrameWorkCfg();

    void updateCal();

    void saveFrameWorkCfg();

private:
    Ui::UAVImageTransDialog *ui;
    FrameWorkConfig m_frameWork;
    QString m_currentFramwWorkCfgName;
};

#endif // UAVIMAGETRANSDIALOG_H
