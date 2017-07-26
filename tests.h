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

signals:

private slots:
    void gradientTest();
    void functor1Test();
    void functor2Test();
};

#endif // TESTS_H
