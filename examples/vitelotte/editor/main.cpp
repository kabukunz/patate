#include <iostream>

#include "GL/glew.h"

#include <QApplication>
#include <QGLFormat>

#include "document.h"
#include "mainWindow.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    if(argc > 2)
    {
        std::cerr << "Usage: " << argv[0] << " [FILENAME]\n";
        return 1;
    }

    QGLFormat glFormat;
    glFormat.setVersion(4, 0);
    glFormat.setSampleBuffers(true);
    glFormat.setSamples(16);
    QGLFormat::setDefaultFormat(glFormat);

    MainWindow window;

    window.resize(800, 600);
    window.show();

    if(argc == 2)
    {
        window.document()->loadMesh(argv[1]);
    }

    return app.exec();
}
