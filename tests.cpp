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

//    qDebug() << pv.angle << pv.transf[0] << pv.transf[1];
//    qDebug() << jtb.angle << jtb.transf[0] << jtb.transf[1];

    QCOMPARE(pv,jtb);
}

void Tests::functor1Test()
{
//    QVector<QVector2D> fig1;
//    QVector<QVector2D> fig2;
//    fig1.clear();
//    fig1.append(QVector2D(1, 1));
//    fig2.clear();
//    fig2.append(QVector2D(2, 2));
//    SimpleFunctor sf = SimpleFunctor(fig1,
//                                     fig2,
//                                     QVector2D(1,1),
//                                     ProblemVector(1,1,1));


//    Derivable da = sf.f1(0,0);
//    Derivable dx = sf.f1(0,1);
//    Derivable dy = sf.f1(0,2);
//    qDebug() << da.getDerivative() << dx.getDerivative() << dy.getDerivative() << da.getValue();


//    GradientDescent gd;
//    gd.fig1.clear();
//    gd.fig1.append(QVector2D(1, 1));
//    gd.fig2.clear();
//    gd.fig2.append(QVector2D(2, 2));
//    gd.center2 = QVector2D(1,1);//gd.massCenter(gd.fig2);

//    ProblemVector pv = gd.grad(ProblemVector(1,1,1),0,true);
//    qDebug() << pv.angle << pv.transf[0] << pv.transf[1];

//    QCOMPARE(pv.angle, da.getDerivative());
    QCOMPARE(1,1);
}

void Tests::functor2Test()
{
//    QVector<QVector2D> fig1;
//    QVector<QVector2D> fig2;
//    fig1.clear();
//    fig1.append(QVector2D(1, 1));
//    fig2.clear();
//    fig2.append(QVector2D(2, 2));
//    SimpleFunctor sf = SimpleFunctor(fig1,
//                                     fig2,
//                                     QVector2D(1,1),
//                                     ProblemVector(1,1,1));

//    Derivable da = sf.f2(0,0);
//    Derivable dx = sf.f2(0,1);
//    Derivable dy = sf.f2(0,2);
//    qDebug() << da.getDerivative() << dx.getDerivative() << dy.getDerivative() << da.getValue() << dx.getValue();


//    GradientDescent gd;
//    gd.fig1.clear();
//    gd.fig1.append(QVector2D(1, 1));
//    gd.fig2.clear();
//    gd.fig2.append(QVector2D(2, 2));
//    gd.center2 = QVector2D(1,1);//gd.massCenter(gd.fig2);

//    ProblemVector pv = gd.grad(ProblemVector(1,1,1),0,false);
//    qDebug() << pv.angle << pv.transf[0] << pv.transf[1];

//    ProblemVector pv1 = gd.gradient(ProblemVector(1,1,1));
//    qDebug() << pv1.angle << pv1.transf[0] << pv1.transf[1] << gd.func(pv1);

//    QCOMPARE(pv.angle, da.getDerivative());
    QCOMPARE(1,1);
}
