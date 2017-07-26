#include "gradientdescent.h"
#include <QDebug>
#include <QList>

GradientDescent::GradientDescent()
{

}

void GradientDescent::exec()
{
    fig1.clear();
    fig1.append(QVector2D(0, 0));
    fig1.append(QVector2D(1, 0));
    fig1.append(QVector2D(1, 1));
    fig1.append(QVector2D(0, 1));
    //QPointF center1 = massCenter(fig1);

    fig2.clear();
    fig2.append(QVector2D(2.85, 0.44));   // 80*
    fig2.append(QVector2D(3.02, 1.43));
    fig2.append(QVector2D(2.04, 1.6));
    fig2.append(QVector2D(1.86, 0.62));

//    fig2.append(QPointF(3.4, 0.6));   // custom
//    fig2.append(QPointF(4.4, 3.3));
//    fig2.append(QPointF(  2, 5));
//    fig2.append(QPointF(1.2, 2.5));

//    fig2.append(QPointF(0.1, 0.1));   // t 0.1 0.1
//    fig2.append(QPointF(1.1, 0.1));
//    fig2.append(QPointF(1.1, 1.1));
//    fig2.append(QPointF(0.1, 1.1));

//    fig2.append(QPointF(1,0));  // 90*
//    fig2.append(QPointF(1,1));
//    fig2.append(QPointF(0,1));
//    fig2.append(QPointF(0,0));
    center2 = massCenter(fig2);

    double step = 1e-3;


    ProblemVector iter = ProblemVector(qDegreesToRadians(0.f), 1, 1);
    ProblemVector nextIter = ProblemVector(0,0,0);
    ProblemVector prevIter = iter;

    int ops = 0;
    do {
        // https://ru.wikipedia.org/wiki/%D0%93%D1%80%D0%B0%D0%B4%D0%B8%D0%B5%D0%BD%D1%82%D0%BD%D1%8B%D0%B9_%D1%81%D0%BF%D1%83%D1%81%D0%BA
        // http://www.machinelearning.ru/wiki/index.php?title=%D0%9C%D0%B5%D1%82%D0%BE%D0%B4_%D0%B3%D1%80%D0%B0%D0%B4%D0%B8%D0%B5%D0%BD%D1%82%D0%BD%D0%BE%D0%B3%D0%BE_%D1%81%D0%BF%D1%83%D1%81%D0%BA%D0%B0#.D0.A7.D0.B8.D1.81.D0.BB.D0.BE.D0.B2.D1.8B.D0.B5_.D0.BF.D1.80.D0.B8.D0.BC.D0.B5.D1.80.D1.8B

        nextIter = iter - gradient(iter)*step;
        prevIter = iter;
        iter = nextIter;
        ops++;
        qDebug() << ops << ": " << func(nextIter) << nextIter.length();
        if(func(nextIter) > func(prevIter))
        {
            qDebug() << "ascending!";
            break;
        }
    } while(qAbs(func(nextIter) - func(prevIter)) > 1e-6);

    qDebug() << func(nextIter) - func(prevIter) << ops;
    qDebug() << qRadiansToDegrees(nextIter.angle)
            << nextIter.transf[0] << nextIter.transf[1];
    foreach (QVector2D p, fig2) {
        qDebug() << pointTransform(p, nextIter);
    }
}

ProblemVector GradientDescent::gradient(ProblemVector vector)
{
//    double dx = 0.001;
//    double dfdx = ((vector.angle + dx) - vector.angle)/dx;

//    double dy = 0.01;
//    double dfdy1 = ((vector.transf[0] + dy) - vector.transf[0])/dy;
//    double dfdy2 = ((vector.transf[1] + dy) - vector.transf[1])/dy;

    double cx = center2.x();
    double cy = center2.y();

    double drda = 0;

    double dtdx = 0;
    double dtdy = 0;

    for(int i=0; i<fig1.length(); i++) {
        double px = fig2[i].x();
        double py = fig2[i].y();
        double p2x = fig1[i].x();
        double p2y = fig1[i].y();

        //double rdx = -(py - cy)*qCos(vector.angle) - (px - cx)*qSin(vector.angle) ;
        //double rdy = +(py - cy)*qSin(vector.angle) - (px - cx)*qCos(vector.angle);
        double rdx = -(px - cx)*qSin(vector.angle) -(py - cy)*qCos(vector.angle);
        double rdy = +(px - cx)*qCos(vector.angle) -(py - cy)*qSin(vector.angle);

        double t = 2*pointTransform(QVector2D(px,py), vector).distanceToPoint(QVector2D(p2x, p2y));
        //double t = 2*pointLength(pointTransform(QVector2D(px,py), vector), QVector2D(p2x, p2y));
        double t1 = cx + (px - cx)*qCos(vector.angle) - (py - cy)*qSin(vector.angle) + vector.transf[0] - p2x;
        double t2 = cy + (px - cx)*qSin(vector.angle) + (py - cy)*qCos(vector.angle) + vector.transf[1] - p2y;

        //drda += t*(rdx+rdy);
        //drda += (rdx+rdy)*2*(t1+t2);
        drda += rdx*2*t1+rdy*2*t2;
        //qDebug() << "r " << rdx << rdy << t << t1 << t2;

        QVector2D rp = pointRotate(QVector2D(px,py), vector.angle, center2);

        dtdx += 2*((rp.x() + vector.transf[0]) - p2x);
        dtdy += 2*((rp.y() + vector.transf[1]) - p2y);
    }

    return ProblemVector(drda, dtdx, dtdy);
}

ProblemVector GradientDescent::grad(ProblemVector vector, int index, bool axis)
{
    double cx = center2.x();
    double cy = center2.y();

    double px = fig2[index].x();
    double py = fig2[index].y();
    double p2x = fig1[index].x();
    double p2y = fig1[index].y();

    double da = 0;
    double dx = 0;
    double dy = 0;

    if(axis){
        //da = -(px - cx)*qSin(vector.angle) -(py - cy)*qCos(vector.angle);
        da = -(px - cx)*qSin(vector.angle) -(py - cy)*qCos(vector.angle);
        dx = 1;
    } else {
        da = +(px - cx)*qCos(vector.angle) -(py - cy)*qSin(vector.angle);
        dy = 1;
    }

//    double rdx = -(px - cx)*qSin(vector.angle) - (py - cy)*qCos(vector.angle);
//    double rdy = +(py - cy)*qSin(vector.angle) - (px - cx)*qCos(vector.angle);

//    //double t = 2*pointTransform(QVector2D(px,py), vector).distanceToPoint(QVector2D(p2x, p2y));

//    double drda = (rdx+rdy);

//    //QVector2D rp = pointRotate(QVector2D(px,py), vector.angle, center2);

//    double dtdx = 1;
//    double dtdy = 1;

    return ProblemVector(da, dx, dy);
}

ProblemVector GradientDescent::partialDerivative(ProblemVector vector)
{
    double cx = center2.x();
    double cy = center2.y();

    double da = 0.01;
    double dt = 0.01;

    double dr = 0;
    double dtx = 0;
    double dty = 0;

//    for(int i=0; i<fig1.length(); i++) {
//        double px = fig2[i].x();
//        double py = fig2[i].y();
//        double p2x = fig1[i].x();
//        double p2y = fig1[i].y();

    dr  = (func(ProblemVector(vector.angle + da, vector.transf[0], vector.transf[1])) - func(vector))/da;
    dtx = (func(ProblemVector(vector.angle, vector.transf[0] + dt, vector.transf[1])) - func(vector))/dt;
    dty = (func(ProblemVector(vector.angle, vector.transf[0], vector.transf[1] + dt)) - func(vector))/dt;
//    }

    return ProblemVector(dr, dtx, dty);
}

double GradientDescent::secondPartDer(ProblemVector vector, int var1, int var2)
{
    double res = 0;
    double dv = 0.001;
    if(var1==var2){
        ProblemVector pv1, pv2;
        switch(var1){
        case 0:
            pv1 = ProblemVector(vector.angle + dv, vector.transf[0], vector.transf[1]);
            pv2 = ProblemVector(vector.angle - dv, vector.transf[0], vector.transf[1]);
            break;
        case 1:
            pv1 = ProblemVector(vector.angle, vector.transf[0] + dv, vector.transf[1]);
            pv2 = ProblemVector(vector.angle, vector.transf[0] - dv, vector.transf[1]);
            break;
        case 2:
            pv1 = ProblemVector(vector.angle, vector.transf[0], vector.transf[1] + dv);
            pv2 = ProblemVector(vector.angle, vector.transf[0], vector.transf[1] - dv);
            break;
        }
        res = (func(pv1) + 2*func(vector) - func(pv2))/dv*dv;
    }
    else {
        double fM = 0;
        double fN = 0;
        if((var1 == 0 && var2 == 1) || (var1 == 1 && var2 == 0)){
            fM = func(ProblemVector(vector.angle + dv, vector.transf[0] + dv, vector.transf[1]))
               - func(ProblemVector(vector.angle + dv, vector.transf[0] - dv, vector.transf[1]));
            fN = func(ProblemVector(vector.angle - dv, vector.transf[0] + dv, vector.transf[1]))
               - func(ProblemVector(vector.angle - dv, vector.transf[0] - dv, vector.transf[1]));
        }
        if((var1 == 0 && var2 == 2) || (var1 == 2 && var2 == 0)){
            fM = func(ProblemVector(vector.angle + dv, vector.transf[0], vector.transf[1] + dv))
               - func(ProblemVector(vector.angle + dv, vector.transf[0], vector.transf[1] - dv));
            fN = func(ProblemVector(vector.angle - dv, vector.transf[0], vector.transf[1] + dv))
               - func(ProblemVector(vector.angle - dv, vector.transf[0], vector.transf[1] - dv));
        }
        if((var1 == 1 && var2 == 2) || (var1 == 2 && var2 == 1)){
            fM = func(ProblemVector(vector.angle, vector.transf[0] + dv, vector.transf[1] + dv))
               - func(ProblemVector(vector.angle, vector.transf[0] + dv, vector.transf[1] - dv));
            fN = func(ProblemVector(vector.angle, vector.transf[0] - dv, vector.transf[1] + dv))
               - func(ProblemVector(vector.angle, vector.transf[0] - dv, vector.transf[1] - dv));
        }
        fM /= 2*dv;
        fN /= 2*dv;
        res = (fM - fN)/(2*dv);
    }


    return res;


    /*//double dv1 = 0.01;
    //double dv2 = 0.001;
    double dv1, dv2, dv3;
    double dv = 0.01;

    switch (var2) {
    case 0:
        dv1 = dv;
        dv2 = 0;
        dv3 = 0;
        break;
    case 1:
        dv1 = 0;
        dv2 = dv;
        dv3 = 0;
        break;
    case 2:
        dv1 = 0;
        dv2 = 0;
        dv3 = dv;
        break;
    }
    //double dgrad = (gradient(ProblemVector(vector.angle + dv1, vector.transf[0], vector.transf[1])) - gradient(vector))/dv1;
    double res = 0;

    switch (var1) {
    case 0:
        res = (ProblemVector(vector.angle + dv1, vector.transf[0] + dv2, vector.transf[1] + dv3) - vector).angle/dv;
        break;
    case 1:
        res = (ProblemVector(vector.angle + dv1, vector.transf[0] + dv2, vector.transf[1] + dv3) - vector).transf[0]/dv;
        break;
    case 2:
        res = (ProblemVector(vector.angle + dv1, vector.transf[0] + dv2, vector.transf[1] + dv3) - vector).transf[1]/dv;
        break;
    default:
        break;
    }

//    double d2r = 0;
//    double d2tx = 0;
//    double d2ty = 0;
//    d2r = (ProblemVector(vector.angle + dv1, vector.transf[0], vector.transf[1]) - vector).angle/dv;
//    d2tx = (ProblemVector(vector.angle, vector.transf[0] + dv2, vector.transf[1]) - vector).transf[0]/dv;
//    d2ty = (ProblemVector(vector.angle, vector.transf[0], vector.transf[1] + dv2) - vector).transf[1]/dv;

    return res;*/




}

double GradientDescent::gradient2(ProblemVector vector, int var1, int var2)
{
    double cx = center2.x();
    double cy = center2.y();


    double res = 0;

    switch(var1){
    case 0:
        for(int i=0; i<fig1.length(); i++) {
            double px = fig2[i].x();
            double py = fig2[i].y();

            double p2x = fig1[i].x();
            double p2y = fig1[i].y();

            double r2dx = -(px - cx)*qCos(vector.angle) + (py - cy)*qSin(vector.angle);
            double r2dy =  (px - cx)*qSin(vector.angle) + (py - cy)*qCos(vector.angle);

            double rdx = -(py - cy)*qCos(vector.angle) - (px - cx)*qSin(vector.angle);
            double rdy =  (py - cy)*qSin(vector.angle) - (px - cx)*qCos(vector.angle);

            if(var1==var2)
            {
                double t = 2*pointTransform(QVector2D(px,py), vector).distanceToPoint(QVector2D(p2x, p2y));
                res += t*(r2dx+r2dy);// + 2*qPow(rdx+rdy, 2);
            }
            else
            {
                res += 2*(rdx+rdy);
            }
//            double t= 0;
//            if(var1 == var2)
//                t = 2*pointTransform(QVector2D(px,py), vector).distanceToPoint(QVector2D(p2x, p2y));
//            else
//                t = 1;
//            res += t*(rdx+rdy);
        }
        break;
    case 1:
    case 2:
        if(var2 != 0)
            res = 2;
        break;
    }


    return res;
}

double GradientDescent::func(ProblemVector vector)
{
    double res = 0;
    for(int i=0; i<fig1.length(); i++){
        res += qPow(pointTransform(fig2[i], vector).distanceToPoint(fig1[i]), 2);
        //res += qPow(pointLength(pointTransform(fig2[i], vector), fig1[i]), 2);
    }
    return res;
}

double GradientDescent::funcP(ProblemVector vector, int index, bool x)
{
    double res = 0;
    if(x){
        res = pointTransform(fig2[index], vector).x() - fig1[index].x();
    }else{
        res = pointTransform(fig2[index], vector).y() - fig1[index].y();
    }
    return res;
}

double GradientDescent::pointLength(QVector2D p1, QVector2D p2)
{
    return qSqrt(qPow(p2.x() - p1.x(), 2) + qPow(p2.y() - p1.y(), 2));
}

QVector2D GradientDescent::pointTransform(QVector2D inPoint, ProblemVector vector)
{
    return pointTranslate(pointRotate(inPoint, vector.angle, center2), vector.transf);
}

QVector2D GradientDescent::pointRotate(QVector2D input, double angle, QVector2D center)
{
    QVector2D res;
    double x = center.x() + (input.x() - center.x())*qCos(angle) - (input.y() - center.y())*qSin(angle);
    double y = center.y() + (input.x() - center.x())*qSin(angle) + (input.y() - center.y())*qCos(angle);
    res.setX( x < 1e-10 ? 0 : x );
    res.setY( y < 1e-10 ? 0 : y );
    return res;

    // center.x() + (input.x() - center.x())*qCos(angle) - (input.y() - center.y())*qSin(angle) + transf[0] - second.x()
    // center.y() + (input.x() - center.x())*qSin(angle) + (input.y() - center.y())*qCos(angle) + transf[1] - second.y()
}

QVector2D GradientDescent::pointTranslate(QVector2D input, QVector<double> transf)
{
    return QVector2D(input.x() + transf[0], input.y() + transf[1]);
}


QVector2D GradientDescent::massCenter(QVector<QVector2D> list)
{
    QVector2D res = QVector2D(0,0);
    foreach (QVector2D p, list) {
        res += p;
    }
    return res / list.length();
}

void GradientDescent::newton()
{
    MatrixXf a(3,3);
    ProblemVector pv;
    ProblemVector iter = ProblemVector(qDegreesToRadians(0.f), 0, 0);
    ProblemVector nextIter = ProblemVector(0,0,0);
    ProblemVector prevIter = iter;
    //ProblemVector b;


    bool min = false;
    while(!min){
        pv = partialDerivative(iter);
        for(int i=0; i<a.rows(); i++){
            for(int j=0; j<a.cols(); j++){
                a(j,i) = gradient2(pv,j,i);
                //a(j,i) = secondPartDer(pv,j,i);
            }
        }
        VectorXf b(3);
        b(0) = pv.angle;
        b(1) = pv.transf[0];
        b(2) = pv.transf[1];
        LLT<MatrixXf> lltA(a);

        Vector3f x = lltA.solve(b); //a.colPivHouseholderQr().solve(b);
        //MatrixXd llt = lltA.matrixL();

        nextIter = iter - partialDerivative(iter);
        //nextIter = iter - hesseInv(iter)*gradient(iter);
        // a = 2nd gradient
        // b = gradient
        // pv = -a-1*b;
    }

}

MatrixXf GradientDescent::jacobian(SimpleFunctor functor)
{
    int length = functor.lengthVector();
    int lengthParams = functor.lengthParams();
    int elems = functor.elems();

    MatrixXf j(elems*length, lengthParams);

    for(int index=0; index < length; index++){
        for(int indElem = 0; indElem < elems; indElem++){
            ProblemVector pv = functor.grad(index, indElem);
            for(int indParam = 0; indParam < lengthParams; indParam++){
                j(elems*index + indElem, indParam) = pv[indParam];
            }
        }
    }
    return j;
}

VectorXf GradientDescent::innerFunc(SimpleFunctor functor)
{
    int length = functor.lengthVector();
    int elems = functor.elems();

    VectorXf f(elems*length);
    for(int index=0; index < length; index++){
        for(int indElem = 0; indElem < elems; indElem++){
            f(elems*index + indElem) = functor.innerF(index, indElem);
        }
    }
    return f;
}


MatrixXf GradientDescent::jacobianFixed(ProblemVector iter, int leng)
{
    MatrixXf j(2*leng, 3);
    for(int row=0; row<leng; row++){
        ProblemVector pv1 = grad(iter, row, true);
        j(2*row+0,0) = pv1.angle;
        j(2*row+0,1) = pv1.transf[0];
        j(2*row+0,2) = pv1.transf[1];
        //f(2*row+0) = gd.funcP(iter, row, true);
        ProblemVector pv2 = grad(iter, row, false);
        j(2*row+1,0) = pv2.angle;
        j(2*row+1,1) = pv2.transf[0];
        j(2*row+1,2) = pv2.transf[1];
        //f(2*row+1) = gd.funcP(iter, row, false);
    }
    return j;
}

VectorXf GradientDescent::innerFuncFixed(ProblemVector iter, int leng)
{
    VectorXf f(2*leng);
    for(int row=0; row<leng; row++){
        f(2*row+0) = funcP(iter, row, true);
        f(2*row+1) = funcP(iter, row, false);
    }
    return f;
}

void GradientDescent::gaussNewtonUniv(SimpleFunctor functor)
{
    qDebug() << "!";

    //ProblemVector iter = functor.probVector;

    int ops = 0;
    bool min = false;
    while(!min){
        ops++;
        qDebug() << ops;

        MatrixXf j = jacobian(functor);
        VectorXf f = innerFunc(functor); //functor.innerFunc();

        MatrixXf b = j.transpose()*f;
        MatrixXf a = j.transpose()*j;

        MatrixXf x = a.llt().solve(-b);

        qDebug() << " " << b(0) << b(1) << b(2);

        ProblemVector nextIter = functor.probVector + ProblemVector(x(0), x(1), x(2));
        //double err = qAbs(functor.func(nextIter) - functor.func(iter));
        double err = qAbs(functor.f(nextIter) - functor.f(functor.probVector));
        min = err < 1e-6;

        if(err != err)
        {
            min = true;
        }
        functor.probVector = nextIter;
        if(ops >= 25)
            min = true;      
    }
    qDebug() << qRadiansToDegrees(functor.probVector[0])
            << functor.probVector[1] << functor.probVector[2];
}


