#include "densityform.h"
#include "ui_densityform.h"

DensityForm::DensityForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DensityForm){
    ui->setupUi(this);
    ui->doubleSpinBox->setMinimum(0.000001);
}

DensityForm::~DensityForm()
{
    delete ui;
}

void DensityForm::on_pushButton_clicked(){

    this->hide();
    emit calculateDensity(ui->doubleSpinBox->value());
}

void DensityForm::showDensityForm(){
    this->show();

}
