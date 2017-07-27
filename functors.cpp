#include "functors.h"

Functor::Functor()
{

}

double Functor::func()
{

}

double Functor::innerFunc()
{

}

SimpleFunctor::SimpleFunctor()
{

}

SimpleFunctor::SimpleFunctor(QVector<QVector2D> point1, QVector<QVector2D> point2, QVector2D center, ProblemVector probVector)
{
    this->point1 = point1;
    this->point2 = point2;
    this->center = center;
    this->probVector = probVector;

    //funcVal = f(probVector);
}

void SimpleFunctor::operator()(ProblemVector pv, int index)
{

}

ProblemVector SimpleFunctor::grad(int index, int indexElement)
{
    ProblemVector pv;// = ProblemVector();
//    for(int pvElemIndex=0; pvElemIndex<pv.count(); pvElemIndex++){
//        pv[pvElemIndex] = f1(index, pvElemIndex, probVector, indexElement).getDerivative();
//    }

    switch (indexElement) {
    case 0:
        pv = ProblemVector(f1(index, 0, probVector, 0).getDerivative(), f1(index, 1, probVector, 0).getDerivative(), f1(index, 2, probVector, 0).getDerivative());
        break;
    case 1:
        pv = ProblemVector(f1(index, 0, probVector, 1).getDerivative(), f1(index, 1, probVector, 1).getDerivative(), f1(index, 2, probVector, 1).getDerivative());
        break;
    }

    return pv;
}

double SimpleFunctor::f(ProblemVector pv)
{
    double res = 0;
    for(int i=0; i<point1.length(); i++){
        res += qSqrt(qPow(f1(i, 0, pv, 0).getValue(), 2) + qPow(f1(i, 0, pv, 1).getValue(), 2));
    }

    return res;
}

double SimpleFunctor::innerF(int index, int elem)
{
    double res = 0;
    switch (elem) {
    case 0:
        res = center.x() + (point2[index].x() - center.x())*qCos(probVector[0])
                         - (point2[index].y() - center.y())*qSin(probVector[0])
                         + probVector[1] - point1[index].x();
        break;
    case 1:
        res = center.y() + (point2[index].x() - center.x())*qSin(probVector[0])
                         + (point2[index].y() - center.y())*qCos(probVector[0])
                         + probVector[2] - point1[index].y();
        break;
    }

    return res;
}

Derivable SimpleFunctor::f1(int index, int indexParam, ProblemVector pv, int val)
{
    QVector<Derivable> pvD;
    for(int i=0; i<pv.count(); i++){
        Derivable xD;
        if(i == indexParam)
            xD = Derivable::IndependendVariable(pv[i]);
        else
            xD = Derivable(pv[i]);
        pvD.append(xD);
    }

    Derivable ad = pvD[0];
    Derivable txd = pvD[1];
    Derivable tyd = pvD[2];

    Derivable res;

    switch (val) {
    case 0:
        res = (Derivable(center.x()) + Derivable(point2[index].x() - center.x())*ad.cos(ad) - Derivable(point2[index].y() - center.y())*ad.sin(ad) + txd - Derivable(point1[index].x()));
        break;
    case 1:
        res = (Derivable(center.y()) + Derivable(point2[index].x() - center.x())*ad.sin(ad) + Derivable(point2[index].y() - center.y())*ad.cos(ad) + tyd - Derivable(point1[index].y()));
        break;
    }

    return res;
}
