#ifndef DENSITYFORM_H
#define DENSITYFORM_H

#include <QWidget>

namespace Ui {
class DensityForm;
}

class DensityForm : public QWidget
{
    Q_OBJECT

public:
    explicit DensityForm(QWidget *parent = nullptr);
    ~DensityForm();

signals:
    void calculateDensity(double);

public slots:
    void showDensityForm();

private slots:
    void on_pushButton_clicked();


private:
    Ui::DensityForm *ui;
};

#endif // DENSITYFORM_H
