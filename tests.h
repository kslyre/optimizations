#ifndef TESTS_H
#define TESTS_H

#include <QObject>
#include <QTest>
#include "gradientdescent.h"
#include <QDebug>

class Tests : public QObject
{
    Q_OBJECT
public:
    explicit Tests(QObject *parent = 0);

public slots:
    void functor1Test();
signals:

private slots:
    void gradientTest();
};

#endif // TESTS_H
