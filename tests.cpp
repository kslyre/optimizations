#include "tests.h"

Tests::Tests(QObject *parent) : QObject(parent)
{

}

void Tests::gradientTest()
{
    GradientDescent gd;
    gd.fig1.clear();
    gd.fig1.append(QVector2D(0, 0));
    gd.fig1.append(QVector2D(0, 1));
    gd.fig2.clear();
    gd.fig2.append(QVector2D(0, 1));
    gd.fig2.append(QVector2D(1, 0));
    gd.center2 = gd.massCenter(gd.fig2);

    ProblemVector x0 = ProblemVector(0,0,0);
    ProblemVector pv = gd.gradient(x0);


    MatrixXf j = gd.jacobianFixed(x0, gd.fig1.length());
    VectorXf f = gd.innerFuncFixed(x0, gd.fig1.length());
    MatrixXf b = j.transpose()*f;

    ProblemVector jtb = ProblemVector(b(0), b(1), b(2));

    qDebug() << pv.angle << pv.transf[0] << pv.transf[1];
    qDebug() << jtb.angle << jtb.transf[0] << jtb.transf[1];

    QCOMPARE(pv,jtb);
}

void Tests::functor1Test()
{
    SimpleFunctor sf = SimpleFunctor(QVector2D(0,0),
                                     QVector2D(0,1),
                                     QVector2D(0.5f,0.5f),
                                     ProblemVector(0,0,0));

    QCOMPARE(1,1);
}
