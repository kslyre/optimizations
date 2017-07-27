#ifndef FUNCTORS_H
#define FUNCTORS_H

#include "derivable.h"

#include <QVector2D>
#include <QVector3D>
#include <structs.h>

class SimpleFunctor {
    QVector<QVector2D> point1;
    QVector<QVector2D> point2;
    QVector2D center;
public:
    SimpleFunctor();
    SimpleFunctor(QVector<QVector2D> point1, QVector<QVector2D> point2, QVector2D center, ProblemVector probVector);
    void operator()(ProblemVector pv, int index);

    int elems() { return 2; }
    int lengthVector() { return point1.length(); }
    int lengthParams() { return probVector.count(); }

    ProblemVector grad(int index, int indexElement);
    ProblemVector probVector;

    Derivable f1(int index, int indexParam, ProblemVector pv, int val);
    double f(ProblemVector pv);

    double innerF(int index, int elem);
};

class Functor
{
    QVector<QVector3D> points1;
    QVector<QVector3D> points2;
    QVector3D center2;
public:
    Functor();
    Functor(QVector<QVector3D> points1, QVector<QVector3D> point2, ProblemVector pv);

    int elems();
    int lengthVector();
    int lengthParams();

    double func();
    double innerFunc();
};





#endif // FUNCTORS_H
