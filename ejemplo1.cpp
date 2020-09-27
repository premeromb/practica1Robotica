#include "ejemplo1.h"

ejemplo1::ejemplo1() : Ui_Counter() {
    setupUi(this);

    cont = 0;
    increment = 1;

    show();

    connect(button, SIGNAL(clicked()), this, SLOT(doButton()));
    connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(changeIncrement(int)));
    connect(periodButton, SIGNAL(clicked()), this, SLOT(doPeriodButton()));
    mytimer.connect(std::bind(&ejemplo1::cuenta, this));
    mytimer.start(1000);
}

ejemplo1::~ejemplo1() {}

void ejemplo1::doButton() {
    static bool stopped = false;
    stopped = !stopped;
    if (stopped) {
        mytimer.stop();
        button->setText("GO!");
        qDebug() << "  click on STOP";
    } else {
        mytimer.start(1000);
        button->setText("STOP!");
        qDebug() << "  click on GO";
    }
}


void ejemplo1::doPeriodButton() {
    QString period = QString::fromStdString(std::to_string(mytimer.getElapsedTime()));
    periodLabel->setText(period);
    qDebug() << "click on periodButton";

}

void ejemplo1::changeIncrement(int value) {
    mytimer.setPeriod(value * 1000);
    qDebug() << "   increment changed to " << increment;
}

void ejemplo1::cuenta() {
    cont = cont + increment;
    lcdNumber->display(cont);
    trick++;
    qDebug() << "time tick";
}

