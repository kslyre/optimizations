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

SimpleFunctor::SimpleFunctor(QVector2D point1, QVector2D point2, QVector2D center, ProblemVector probVector)
{
    p1 = point1;
    p2 = point2;
    this->center = center;
    this->probVector = probVector;

    funcVal = f(probVector);
}

void SimpleFunctor::operator()(ProblemVector pv)
{

}

Derivable SimpleFunctor::f(int index)
{
    QVector<Derivable> pvD = probVector;
    for(int i; i<probVector.length(); i++){
        if(i == index)
            pvD[i] = Derivable::IndependendVariable(probVector[i]);
        else
            pvD[i] = Derivable::NoVariable(probVector[i]);
    }

    Derivable ad = pvD[0];
    Derivable txd = pvD[1];
    Derivable tyd = pvD[2];

    //Derivable ad = Derivable::IndependendVariable(a);
    return center.x() + (p2.x() - center.x())*ad.cos(ad) - (p2.y() - center.y())*ad.sin(ad) + txd + tyd - p1.x();
}
