#include "percentform.h"
#include "ui_percentform.h"

PercentForm::PercentForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PercentForm)
{
    ui->setupUi(this);
}

PercentForm::~PercentForm()
{
    delete ui;
}

void PercentForm::on_pushButton_clicked(){
    this->hide();
    emit simplify(this->ui->spinBox->value());
}


void PercentForm::showPercentForm(){
    this->show();
}
