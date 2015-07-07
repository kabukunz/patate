/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
    // For some reasons, GLEW fails with a 4.0 context under Windows but need
    // it under Linux.
    // FIXME: clean this mess, and check under macOSX.
#ifndef _WIN32
    glFormat.setVersion(4, 0);
#endif
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
