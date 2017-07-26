#ifndef FUNCTORS_H
#define FUNCTORS_H

#include "derivable.h"

#include <QVector2D>
#include <structs.h>

class SimpleFunctor {
    QVector<QVector2D> point1;
    QVector<QVector2D> point2;
    QVector2D center;


    double value;
public:
    SimpleFunctor();
    SimpleFunctor(QVector<QVector2D> point1, QVector<QVector2D> point2, QVector2D center, ProblemVector probVector);
    void operator()(ProblemVector pv, int index);

    int elems() { return 2; }
    int lengthVector() { return point1.length(); }
    int lengthParams() { return probVector.count(); }


    ProblemVector grad(int index, int indexElement);

    ProblemVector probVector;
    ProblemVector resVector;

    Derivable f1(int index, int indexParam, ProblemVector pv);
    Derivable f2(int index, int indexParam, ProblemVector pv);
    double f(ProblemVector pv);
    double innerFnnn(int indexElement);

    double innerF(int index, int elem);
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
