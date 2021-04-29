#ifndef RMSE_H
#define RMSE_H

#include <QWidget>

namespace Ui {
class RMSE;
}

class RMSE : public QWidget
{
    Q_OBJECT

public:
    explicit RMSE(QWidget *parent = nullptr);
    ~RMSE();
    void setQComboBoxes(QStringList&);
    bool isShowPC();

signals:
    void calculateRMSE(int,int);

private slots:
    void on_pushButton_clicked();
private:
    Ui::RMSE *ui;


};

#endif // RMSE_H
