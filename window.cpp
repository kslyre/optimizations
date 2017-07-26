#include "window.h"

#include <QDebug>

Window::Window(QWidget *parent)
    : QWidget(parent)
{
    resize(1000, 800);

    QPushButton *b0 = new QPushButton("Gradient Descent");
    QPushButton *b1 = new QPushButton("Newton");
    QPushButton *b2 = new QPushButton("Gauss-Newton");
    QPushButton *b3 = new QPushButton("Gauss-Newton Universal");
    connect(b0, &QPushButton::clicked, this, gradientDescentExec);
    connect(b1, &QPushButton::clicked, this, newton);
    connect(b2, &QPushButton::clicked, this, gaussNewton);
    connect(b3, &QPushButton::clicked, this, gaussNewtonUniv);

    sb = new QDoubleSpinBox();
    sb->setDecimals(4);
    sb->setValue(1e-2);


    //connect(sb, SIGNAL(QDoubleSpinBox::valueChanged), this, SLOT(Window::setStep));
    //connect(sb, &QDoubleSpinBox::valueChanged, this, setStep);

    QGridLayout *l = new QGridLayout();
    l->addWidget(b0, 0, 0, 1, 1);
    l->addWidget(sb, 0, 3, 1, 1);
    l->addWidget(b1, 0, 6, 1, 1);
    l->addWidget(b2, 0, 12, 1, 1);
    l->addWidget(b3, 0, 16, 1, 1);
    l->setAlignment(Qt::AlignTop);
    setLayout(l);
}

Window::~Window()
{
    QApplication::exit();
}

void Window::setStep(double val)
{
    step = val;
}


void Window::paintEvent(QPaintEvent *event)
{
    if(fig2conv.length() < 1)
        return;

    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(0,0,1000,800, Qt::white);

    QPen graphPen = QPen(Qt::red);
    //graphPen.setColor(Qt::red);
    QVector<qreal> dash;
    dash.push_back(1);
    dash.push_back(3);
    graphPen.setDashPattern(dash);
    painter.setPen(graphPen);

    for(int i=0; i<gd.fig1.length(); i++){
        painter.drawLine(pconv(gd.fig1[i]), pconv(fig2conv[i]));
    }

    graphPen = QPen(Qt::black);
    //graphPen.setColor(Qt::black);
    painter.setPen(graphPen);
    foreach (QVector2D p, gd.fig1) {
        painter.drawEllipse(pconv(p), 3,3);
    }

    if(gd.fig1.length() > 0){
        for(int i=0;i<gd.fig1.length()-1; i++){
            painter.drawLine(pconv(gd.fig1[i]), pconv(gd.fig1[i+1]));
        }

        painter.drawLine(pconv(gd.fig1.last()), pconv(gd.fig1[0]));
    }


    graphPen.setColor(Qt::blue);
    painter.setPen(graphPen);
    foreach (QVector2D p, fig2conv) {
        painter.drawEllipse(pconv(p), 3,3);
    }     
    if(fig2conv.length() > 0){
        for(int i=0;i<fig2conv.length()-1; i++){
            painter.drawLine(pconv(fig2conv[i]), pconv(fig2conv[i+1]));
        }

        painter.drawLine(pconv(fig2conv.last()), pconv(fig2conv[0]));
    }
    graphPen.setWidthF(0.5);
    graphPen.setColor(Qt::gray);
    painter.setPen(graphPen);
    if(gd.fig2.length() > 0){
        for(int i=0;i<gd.fig2.length()-1; i++){
            painter.drawLine(pconv(gd.fig2[i]), pconv(gd.fig2[i+1]));
        }

        painter.drawLine(pconv(gd.fig2.last()), pconv(gd.fig2[0]));
    }

    painter.end();
}

QPointF Window::pconv(QVector2D p)
{
    QVector2D res = p;
    res = res * 80;
    res.setY(this->size().height()-100 - res.y());
    res.setX(res.x() + 200);
    return res.toPointF();
}

void Window::prepareData()
{
    step = sb->value();

    gd.fig1.clear();
//    gd.fig1.append(QVector2D(0, 0));
//    gd.fig1.append(QVector2D(0, 1));

    gd.fig1.append(QVector2D(0, 0));
    gd.fig1.append(QVector2D(1, 0));
    gd.fig1.append(QVector2D(1, 1));
    gd.fig1.append(QVector2D(0, 1));

    gd.fig2.clear();
//    gd.fig2.append(QVector2D(0, 1));
//    gd.fig2.append(QVector2D(1, 0));

//    gd.fig2.append(QVector2D(2.85, 0.44));   // 80*
//    gd.fig2.append(QVector2D(3.02, 1.43));
//    gd.fig2.append(QVector2D(2.04, 1.6));
//    gd.fig2.append(QVector2D(1.86, 0.62));

//    gd.fig2.append(QVector2D(0.1, 0.1));   // t 0.1 0.1
//    gd.fig2.append(QVector2D(1.1, 0.1));
//    gd.fig2.append(QVector2D(1.1, 1.1));
//    gd.fig2.append(QVector2D(0.1, 1.1));

    gd.fig2.append(QVector2D(3.4, 0.6));   // custom
    gd.fig2.append(QVector2D(4.4, 3.3));
    gd.fig2.append(QVector2D(  2, 5));
    gd.fig2.append(QVector2D(1.2, 2.5));

//    gd.fig2.append(QVector2D(1,0));  // 90*
//    gd.fig2.append(QVector2D(1,1));
//    gd.fig2.append(QVector2D(0,1));
//    gd.fig2.append(QVector2D(0,0));

    gd.center2 = gd.massCenter(gd.fig2);
}

void Window::gradientDescentExec()
{
    prepareData();

    //gd.newton();

    //double step = 1e-3;

    ProblemVector iter = ProblemVector(qDegreesToRadians(0.f), 0, 0);
    ProblemVector nextIter = ProblemVector(0,0,0);
    ProblemVector prevIter = iter;

    int ops = 0;
    do {
        //nextIter = iter - gd.gradient(iter)*step;
        nextIter = iter - gd.partialDerivative(iter)*step;
        prevIter = iter;
        iter = nextIter;
        ops++;

        fig2conv.clear();
        for(int i=0; i<gd.fig2.length(); i++){
            fig2conv.append(gd.pointTransform(gd.fig2[i], nextIter));
        }
        this->update();

        QApplication::processEvents();
        QThread::msleep(5);
        //qInfo() << ops << ": " << gd.func(nextIter) << gd.gradient(nextIter).length();
        qDebug() << ops << ": " << gd.func(nextIter) << gd.partialDerivative(nextIter).length();
        if(gd.func(nextIter) > gd.func(prevIter))
        {
            qDebug() << "ascending!";
            break;
        }
    } while(qAbs(gd.func(nextIter) - gd.func(prevIter)) > 1e-6);
    //} while(qAbs(gd.gradient(nextIter).length() - gd.gradient(prevIter).length()) > 1e-10);

    qDebug() << gd.func(nextIter) - gd.func(prevIter) << ops;
    qDebug() << qRadiansToDegrees(nextIter.angle)
            << nextIter.transf[0] << nextIter.transf[1];
    foreach (QVector2D p, gd.fig2) {
        qDebug() << gd.pointTransform(p, nextIter);
    }
}

void Window::newton()
{
    prepareData();

    MatrixXf a(3,3);
    ProblemVector pv;
    ProblemVector iter = ProblemVector(qDegreesToRadians(180.f), 10, 10);
    ProblemVector nextIter = ProblemVector(0,0,0);
    ProblemVector prevIter = iter;
    //ProblemVector b;

    int ops = 0;
    bool min = false;
    while(!min){
        ops++;
        //pv = gd.partialDerivative(iter);
        //pv = gd.gradient(iter);
        pv = iter;
        for(int i=0; i<a.rows(); i++){
            for(int j=0; j<a.cols(); j++){
                //a(j,i) = gd.gradient2(pv,j,i);
                a(j,i) = gd.secondPartDer(pv,j,i);
            }
        }
        //VectorXf b(3);
        Vector3f b;
        pv = gd.gradient(iter);
        b(0) = pv.angle;
        b(1) = pv.transf[0];
        b(2) = pv.transf[1];
        LLT<MatrixXf> lltA(a);

        Vector3f x = lltA.solve(-b);
        SelfAdjointEigenSolver<Matrix3f> es(a);
        Vector3f ev = es.eigenvalues();
        //qInfo() << "ev:" << ev(0) << ev(1) << ev(2);

        nextIter = iter + ProblemVector(x(0), x(1), x(2));
        qDebug() << "res" << nextIter.angle << nextIter.transf[0] << nextIter.transf[1];
        //Vector3f r = Vector3f();
        //nextIter = ProblemVector(-r(0), -r(1), -r(2));
        prevIter = iter;
        iter = nextIter;

        fig2conv.clear();
        for(int i=0; i<gd.fig2.length(); i++){
            fig2conv.append(gd.pointTransform(gd.fig2[i], nextIter));
        }
        this->update();

        QApplication::processEvents();
        QThread::msleep(50);

        qDebug() << ops << ": " << gd.func(nextIter);
        //qInfo() << r(0) << r(1) << r(2);

        double err = qAbs(gd.func(nextIter) - gd.func(prevIter));
        min = err < 1e-3;
        if(err != err)
        {
            min = true;
            QApplication::exit();
        }
        //qInfo() << qAbs(gd.func(nextIter) - gd.func(prevIter));

        //a.colPivHouseholderQr().solve(b);
        //MatrixXd llt = lltA.matrixL();

        //nextIter = iter - partialDerivative(iter);
        //nextIter = iter - hesseInv(iter)*gradient(iter);
        // a = 2nd gradient
        // b = gradient
        // pv = -a-1*b;
    }

    qDebug() << qRadiansToDegrees(nextIter.angle)
            << nextIter.transf[0] << nextIter.transf[1];
    foreach (QVector2D p, gd.fig2) {
        qDebug() << gd.pointTransform(p, nextIter);
    }

    // http://matlab.exponenta.ru/optimiz/book_2/2_3.php
    // http://scjournal.ru/articles/issn_1993-5552_2012_8_48.pdf
}

void Window::gaussNewton()
{
    prepareData();


    ProblemVector pv;
    ProblemVector iter = ProblemVector(qDegreesToRadians(0.f), 0, 0);
    ProblemVector nextIter = ProblemVector(0,0,0);
    ProblemVector prevIter = iter;

    int ops = 0;
    bool min = false;
    while(!min){
        ops++;


//        for(int row=0; row<gd.fig1.length(); row++){
//            //for(int col=0; col<3; col++){
//            ProblemVector pv1 = gd.grad(iter, row, true);
//            j(2*row+0,0) = pv1.angle;
//            j(2*row+0,1) = pv1.transf[0];
//            j(2*row+0,2) = pv1.transf[1];
//            f(2*row+0) = gd.funcP(iter, row, true);
//            ProblemVector pv2 = gd.grad(iter, row, false);
//            j(2*row+1,0) = pv2.angle;
//            j(2*row+1,1) = pv2.transf[0];
//            j(2*row+1,2) = pv2.transf[1];
//            f(2*row+1) = gd.funcP(iter, row, false);
//        }
        MatrixXf j = gd.jacobianFixed(iter, gd.fig1.length());
        VectorXf f = gd.innerFuncFixed(iter, gd.fig1.length());
        MatrixXf b = j.transpose()*f;
        MatrixXf a;
        a = j.transpose()*j;
        //LLT<MatrixXf> lltA(a);
        MatrixXf x = a.llt().solve(-b);  //lltA.solve(-b);
        //qDebug() << x(0) << x(1) << x(2);

        //nextIter = iter + ProblemVector(x(1,0), x(1,0), x(2,0));
        nextIter = iter + ProblemVector(x(0), x(1), x(2));
        //qDebug() << "res = " << nextIter.angle
        //        << nextIter.transf[0] << nextIter.transf[1];
        prevIter = iter;
        iter = nextIter;

        fig2conv.clear();
        for(int i=0; i<gd.fig2.length(); i++){
            fig2conv.append(gd.pointTransform(gd.fig2[i], nextIter));
        }
        this->update();

        QApplication::processEvents();
        QThread::msleep(50);

        qDebug() << ops << ": " << gd.func(nextIter);
        qDebug() << " " << gd.gradient(prevIter).angle << gd.gradient(prevIter).transf[0] << gd.gradient(prevIter).transf[1];
        qDebug() << " " << b(0) << b(1) << b(2);

        double err = qAbs(gd.func(nextIter) - gd.func(prevIter));
        min = err < 1e-6;
        if(err != err)
        {
            min = true;
            QApplication::exit();
        }
    }

    qDebug() << qRadiansToDegrees(nextIter.angle)
            << nextIter.transf[0] << nextIter.transf[1];
    foreach (QVector2D p, gd.fig2) {
        qDebug() << gd.pointTransform(p, nextIter);
    }

    // https://habrahabr.ru/company/intel/blog/170729/
    // https://habrahabr.ru/post/125995/
    //

    //http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf
    //https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm
    //http://www.machinelearning.ru/wiki/index.php?title=%D0%9C%D0%B5%D1%82%D0%BE%D0%B4_%D0%9D%D1%8C%D1%8E%D1%82%D0%BE%D0%BD%D0%B0-%D0%93%D0%B0%D1%83%D1%81%D1%81%D0%B0
    //
}

void Window::gaussNewtonUniv()
{
    //prepareData();

    QVector<QVector2D> fig1;
    QVector<QVector2D> fig2;

    fig1.append(QVector2D(0, 0));
    fig1.append(QVector2D(1, 0));
    fig1.append(QVector2D(1, 1));
    fig1.append(QVector2D(0, 1));

    fig2.append(QVector2D(3.4, 0.6));   // custom
    fig2.append(QVector2D(4.4, 3.3));
    fig2.append(QVector2D(  2, 5));
    fig2.append(QVector2D(1.2, 2.5));

    gd.massCenter(gd.fig2);

    SimpleFunctor sf = SimpleFunctor(fig1, fig2,
                                     gd.massCenter(fig2),
                                     ProblemVector(0,0,0));


    gd.gaussNewtonUniv(sf);
}
