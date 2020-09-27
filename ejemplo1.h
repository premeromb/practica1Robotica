#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include <chrono>
#include "timer.h"


class ejemplo1 : public QWidget, public Ui_Counter {
Q_OBJECT
public:
    ejemplo1();

    int count;
    bool stop;
    int increment;

    virtual ~ejemplo1();

public slots:
    void changeIncrement(int value);
    void doButton();
    void doPeriodButton();

private:
    Timer mytimer, mytimerLong;
    int cont = 0;
    // dos callbacks con diferente número de parámetros
    void cuenta();

    int trick = 5;
};

#endif // ejemplo1_H
