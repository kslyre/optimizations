#include "tests.h"
#include "window.h"
#include <QApplication>

void runTests()
{
    Tests tests;
    QTest::qExec(&tests);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    runTests();

    Window w;
    w.show();

    return a.exec();
}
