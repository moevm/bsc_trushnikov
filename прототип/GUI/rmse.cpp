#include "rmse.h"
#include "ui_rmse.h"
#include <iostream>

RMSE::RMSE(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RMSE){
    ui->setupUi(this);

}

RMSE::~RMSE()
{
    delete ui;
}

void RMSE::setQComboBoxes(QStringList &list){
    ui->comboBox->clear();
    ui->comboBox_2->clear();
    ui->comboBox->insertItems(0,list);
    ui->comboBox_2->insertItems(0,list);
}

void RMSE::on_pushButton_clicked(){
    if(ui->comboBox->currentIndex() != ui->comboBox_2->currentIndex()){
        this->hide();
        emit calculateRMSE(ui->comboBox->currentIndex(),ui->comboBox_2->currentIndex());
    }
}

bool RMSE::isShowPC() {
    return ui->checkBox->isChecked();
}
