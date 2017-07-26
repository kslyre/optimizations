#ifndef FUNCTORS_H
#define FUNCTORS_H

#include "derivable.h"

#include <QVector2D>
#include <structs.h>

class SimpleFunctor {
    QVector2D p1;
    QVector2D p2;
    QVector2D center;
    ProblemVector probVector;


    Derivable funcVal;
public:
    SimpleFunctor();
    SimpleFunctor(QVector2D point1, QVector2D point2, QVector2D center, ProblemVector probVector);
    void operator()(ProblemVector pv);

    Derivable f(int index);
};

class Functors
{
    QVector<QVector2D> point1;
    QVector<QVector2D> point2;
    QVector2D center2;

    SimpleFunctor sf1;
public:
    Functors();

    double func();
    double innerFunc();
};





#endif // FUNCTORS_H
