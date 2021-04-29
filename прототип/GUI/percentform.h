#ifndef PERCENTFORM_H
#define PERCENTFORM_H

#include <QWidget>

namespace Ui {
class PercentForm;
}

class PercentForm : public QWidget
{
    Q_OBJECT

public:
    explicit PercentForm(QWidget *parent = nullptr);
    ~PercentForm();

public slots:
    void showPercentForm();

signals:
    void simplify(int);

private slots:
    void on_pushButton_clicked();

private:
    Ui::PercentForm *ui;
};

#endif // PERCENTFORM_H
