#include "functors.h"

Functors::Functors()
{

}

double Functors::func()
{

}

double Functors::innerFunc()
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
    ProblemVector pv;
    switch (indexElement) {
    case 0:
        pv = ProblemVector(f1(index, 0, probVector).getDerivative(), f1(index, 1, probVector).getDerivative(), f1(index, 2, probVector).getDerivative());
        break;
    case 1:
        pv = ProblemVector(f2(index, 0, probVector).getDerivative(), f2(index, 1, probVector).getDerivative(), f2(index, 2, probVector).getDerivative());
        break;
    }
    return pv;
}

double SimpleFunctor::f(ProblemVector pv)
{
    double res = 0;
    for(int i=0; i<point1.length(); i++){
        res += qSqrt(qPow(f1(i, 0, pv).getValue(), 2) + qPow(f2(i, 0, pv).getValue(), 2));
    }
    return res;
}

double SimpleFunctor::innerFnnn(int indexElement)
{
    double res = 0;
//    if(indexElement == 0){
//        res = innerF(fig2[index], vector).x() - fig1[index].x();
//    }else{
//        res = innerF(fig2[index], vector).y() - fig1[index].y();
//    }
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

Derivable SimpleFunctor::f1(int index, int indexParam, ProblemVector pv)
{
//    QVector<Derivable> pvD = probVector;
//    for(int i; i<probVector.length(); i++){
//        if(i == index)
//            pvD[i] = Derivable::IndependendVariable(probVector[i]);
//        else
//            pvD[i] = Derivable::NoVariable(probVector[i]);
//    }

//    Derivable ad = pvD[0];
//    Derivable txd = pvD[1];
//    Derivable tyd = pvD[2];

//    //Derivable ad = Derivable::IndependendVariable(a);
//    return center.x() + (p2.x() - center.x())*ad.cos(ad) - (p2.y() - center.y())*ad.sin(ad) + txd + tyd - p1.x();


    QVector<Derivable> pvD;
    for(int i=0; i<pv.count(); i++){
        Derivable xD;
        if(i == indexParam)
            xD = Derivable::IndependendVariable(pv[i]);
        else
            xD = Derivable(pv[i]);
            //xD = Derivable::NoVariable(probVector[i]);
        pvD.append(xD);
    }

    Derivable ad = pvD[0];
    Derivable txd = pvD[1];
    Derivable tyd = pvD[2];

    //return center.x() + (p2.x() - center.x())*ad.cos(ad) - (p2.y() - center.y())*ad.sin(ad) + txd + tyd - p1.x();
    return (Derivable(center.x()) + Derivable(point2[index].x() - center.x())*ad.cos(ad) - Derivable(point2[index].y() - center.y())*ad.sin(ad) + txd - Derivable(point1[index].x()));
}

Derivable SimpleFunctor::f2(int index, int indexParam, ProblemVector pv)
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

    return (Derivable(center.y()) + Derivable(point2[index].x() - center.x())*ad.sin(ad) + Derivable(point2[index].y() - center.y())*ad.cos(ad) + tyd - Derivable(point1[index].y()));
}
