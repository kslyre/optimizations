#ifndef GRADIENTDESCENT_H
#define GRADIENTDESCENT_H

#include "functors.h"

#include <QVector>
#include <QVector2D>
#include <QtMath>
#include <Eigen/Eigen>
#include <structs.h>

using namespace Eigen;

class GradientDescent
{
public:
    GradientDescent();

    QVector<QVector2D> fig1;
    QVector<QVector2D> fig2;
    QVector2D center2;

    void exec();
    ProblemVector gradient(ProblemVector vector);
    double func(ProblemVector vector);
    QVector2D massCenter(QVector<QVector2D> list);
    QVector2D pointTransform(QVector2D inPoint, ProblemVector vector);
    QVector2D pointRotate(QVector2D input, double angle, QVector2D center);
    QVector2D pointTranslate(QVector2D input, QVector<double> transf);
    double pointLength(QVector2D p1, QVector2D p2);
    void newton();
    ProblemVector partialDerivative(ProblemVector vector);
    double secondPartDer(ProblemVector vector, int var1, int var2);
    double gradient2(ProblemVector vector, int var1, int var2);
    ProblemVector grad(ProblemVector vector, int index, bool axis);
    double funcP(ProblemVector vector, int index, bool x);
    MatrixXf jacobianFixed(ProblemVector iter, int leng);
    VectorXf innerFuncFixed(ProblemVector iter, int leng);

    MatrixXf jacobian(SimpleFunctor functor);
    VectorXf innerFunc(SimpleFunctor functor);
public slots:
    void gaussNewtonUniv(SimpleFunctor functor);
};

#endif // GRADIENTDESCENT_H
