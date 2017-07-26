#ifndef WINDOW_H
#define WINDOW_H

#include <QDoubleSpinBox>
#include <QWidget>
#include <gradientdescent.h>
#include <QApplication>
#include <QThread>
#include <QPainter>
#include <QDebug>
#include <QPushButton>
#include <QLayout>
#include <QMessageLogger>
#include <functors.h>

class Window : public QWidget
{
    Q_OBJECT

    GradientDescent gd;
    QList<QVector2D> fig2conv;

    double step;
    QDoubleSpinBox *sb;

    void gradientDescentExec();
    void newton();
public:
    Window(QWidget *parent = 0);
    ~Window();
    QPointF pconv(QVector2D p);
    void prepareData();

    MatrixXf jacobian(ProblemVector iter, int leng);
    VectorXf innerFunc(ProblemVector iter, int leng);
protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

public slots:
    void setStep(double val);
    void gaussNewtonUniv();
private slots:
    void gaussNewton();
};

#endif // WINDOW_H
