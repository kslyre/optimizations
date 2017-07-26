#ifndef STRUCTS_H
#define STRUCTS_H

#include <QVector>
#include <QtMath>


struct ProblemVector
{
    double angle;
    QVector<double> transf;

    ProblemVector(){}

    ProblemVector(double a, double tx, double ty)
    {
        angle = a;
        transf.append(tx);
        transf.append(ty);
    }

    double length()
    {
        return qPow(angle, 2) + qPow(transf[0], 2) + qPow(transf[1], 2);
    }

    int count()
    {
        return 3;
    }

    inline ProblemVector operator *(const double &d) const
    {
        return ProblemVector(angle*d, transf[0]*d, transf[1]*d);
    }

    inline ProblemVector operator -(const ProblemVector &v) const
    {
        return ProblemVector(angle - v.angle,
                transf[0] - v.transf[0],
                transf[1] - v.transf[1]);
    }

    inline ProblemVector operator +(const ProblemVector &v) const
    {
        return ProblemVector(angle + v.angle,
                transf[0] + v.transf[0],
                transf[1] + v.transf[1]);
    }

    inline double operator[](int n) const
    {
        double res;
        switch(n){
        case 0:
            res = angle;
            break;
        case 1:
        case 2:
            res = transf[n-1];
            break;
        }
        return res;
    }

    inline bool operator== (const ProblemVector &v) const
        {
            return ( (angle == v.angle) &
                     (transf[0] == v.transf[0]) &
                     (transf[1] == v.transf[1]) );
        }
};

#endif // STRUCTS_H
